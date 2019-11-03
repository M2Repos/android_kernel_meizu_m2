/* 
 * Author: Chunlei.Wang <Chunlei.Wang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <hwmsen_helper.h>
#include <sensors_io.h>
#include <hwmsensor.h>
#include <cust_acc.h>
#include <accel.h>

#include "lis3dh.h"

#define LIS3DH_AXIS_X 0
#define LIS3DH_AXIS_Y 1
#define LIS3DH_AXIS_Z 2
#define LIS3DH_AXES_NUM 3
#define LIS3DH_DATA_LEN 6

#define LIS3DH_DEV_NAME "lis3dh"

#define C_MAX_FIR_LENGTH (32)

#define GSE_FUN(f)
#define GSE_ERR(fmt, args...)
#define GSE_LOG(fmt, args...)

static const struct i2c_device_id lis3dh_i2c_id[] = {{LIS3DH_DEV_NAME, 0}, {}};

static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,lis3dh"},
};

static int lis3dh_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id);

static int lis3dh_i2c_detect(struct i2c_client *client,
		struct i2c_board_info *info);

static int lis3dh_i2c_remove(struct i2c_client *client);
static int lis3dh_suspend(struct i2c_client *client, pm_message_t msg);
static int lis3dh_resume(struct i2c_client *client);
static int lis3dh_local_init(void);
static int lis3dh_remove(void);
static int lis3dh_init_flag = -1;

enum ADX_TRC {
	ADX_TRC_FILTER = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL = 0x04,
	ADX_TRC_CALI = 0X08,
	ADX_TRC_INFO = 0X10,
};

struct scale_factor {
	u8 whole;
	u8 fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};

struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][LIS3DH_AXES_NUM];
	int sum[LIS3DH_AXES_NUM];
	int num;
	int idx;
};

static struct acc_init_info lis3dh_init_info = {
	.name = "lis3dh",
	.init = lis3dh_local_init,
	.uninit = lis3dh_remove,
};

struct lis3dh_i2c_data {
	struct i2c_client *client;
	struct acc_hw hw;
	struct hwmsen_convert cvt;

	/*misc*/
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[LIS3DH_AXES_NUM + 1];

	/*data*/
	s8 offset[LIS3DH_AXES_NUM + 1];
	s16 data[LIS3DH_AXES_NUM + 1];

	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
};

static struct i2c_driver lis3dh_i2c_driver = {
	.driver = {
		.name = LIS3DH_DEV_NAME,
		.of_match_table = accel_of_match,
	},
	.probe = lis3dh_i2c_probe,
	.remove = lis3dh_i2c_remove,
	.detect = lis3dh_i2c_detect,
	.suspend = lis3dh_suspend,
	.resume = lis3dh_resume,
	.id_table = lis3dh_i2c_id,
};


static struct i2c_client *lis3dh_i2c_client;
static struct lis3dh_i2c_data *obj_i2c_data;
static struct GSENSOR_VECTOR3D gsensor_gain;

static DEFINE_MUTEX(lis3dh_i2c_mutex);
static DEFINE_MUTEX(lis3dh_op_mutex);

static int sensor_suspend;

static bool enable_status;
static bool sensor_power = true;

static struct data_resolution lis3dh_data_resolution[] = {
	{{1, 0}, 1024},
	{{1, 9}, 512},
	{{3, 9}, 256},
};

static struct data_resolution lis3dh_offset_resolution = {{15, 6}, 64};

static int lis_i2c_read_block(struct i2c_client *client,
		u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {{0}, {0}};

	mutex_lock(&lis3dh_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&lis3dh_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis3dh_i2c_mutex);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err < 0) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	mutex_unlock(&lis3dh_i2c_mutex);
	return err; /* if success will return 0 */
}

static int lis_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	/*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err = 0, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	mutex_lock(&lis3dh_i2c_mutex);
	if (!client) {
		mutex_unlock(&lis3dh_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis3dh_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		GSE_ERR("send command error!!\n");
		mutex_unlock(&lis3dh_i2c_mutex);
		return -EFAULT;
	}

	mutex_unlock(&lis3dh_i2c_mutex);
	return err; /* if success will return transfer length */
}


static void dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x20;
	u8 regdata = 0;

	for (i = 0; i < 3 ; i++) {
		/* dump all */
		lis_i2c_read_block(client, addr, &regdata, 1);
		GSE_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}

static int lis3dh_setDataResolution(struct lis3dh_i2c_data *obj)
{
	int err;
	u8 dat;
	u16 reso;

	err = lis_i2c_read_block(obj->client, LIS3DH_REG_CTL_REG4, &dat, 0x01);
	if (err < 0) {
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso = (dat & 0x30) << 4;
	if (reso >= 0x3)
		reso = 0x2;

	if (reso < sizeof(lis3dh_data_resolution)/sizeof(lis3dh_data_resolution[0])) {
		obj->reso = &lis3dh_data_resolution[reso];
		return 0;
	} else {
		return -EINVAL;
	}
}

static int lis3dh_readData(struct i2c_client *client, s16 data[LIS3DH_AXES_NUM])
{
	struct lis3dh_i2c_data *priv = i2c_get_clientdata(client);
	u8 buf[LIS3DH_DATA_LEN] = {0};
	int err = 0;

	if (NULL == client) {
		err = -EINVAL;
	} else {
		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_X, buf, 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}
		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_X+1, &buf[1], 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}

		data[LIS3DH_AXIS_X] = (s16)((buf[0]+(buf[1]<<8))>>4);
		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_Y, &buf[2], 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}
		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_Y+1, &buf[3], 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}

		data[LIS3DH_AXIS_Y] =  (s16)((s16)(buf[2]+(buf[3]<<8))>>4);
		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_Z, &buf[4], 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}

		if ((lis_i2c_read_block(client, LIS3DH_REG_OUT_Z+1, &buf[5], 0x01)) < 0) {
			GSE_ERR("read Gsensor data register err!\n");
			return -1;
		}

		data[LIS3DH_AXIS_Z] = (s16)((buf[4] + (buf[5] << 8)) >> 4);

		data[LIS3DH_AXIS_X] &= 0xfff;
		data[LIS3DH_AXIS_Y] &= 0xfff;
		data[LIS3DH_AXIS_Z] &= 0xfff;

		if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z],
					data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
		}

		if (data[LIS3DH_AXIS_X]&0x800) {
			data[LIS3DH_AXIS_X] = ~data[LIS3DH_AXIS_X];
			data[LIS3DH_AXIS_X] &= 0xfff;
			data[LIS3DH_AXIS_X] += 1;
			data[LIS3DH_AXIS_X] = -data[LIS3DH_AXIS_X];
		}
		if (data[LIS3DH_AXIS_Y]&0x800) {
			data[LIS3DH_AXIS_Y] = ~data[LIS3DH_AXIS_Y];
			data[LIS3DH_AXIS_Y] &= 0xfff;
			data[LIS3DH_AXIS_Y] += 1;
			data[LIS3DH_AXIS_Y] = -data[LIS3DH_AXIS_Y];
		}
		if (data[LIS3DH_AXIS_Z]&0x800) {
			data[LIS3DH_AXIS_Z] = ~data[LIS3DH_AXIS_Z];
			data[LIS3DH_AXIS_Z] &= 0xfff;
			data[LIS3DH_AXIS_Z] += 1;
			data[LIS3DH_AXIS_Z] = -data[LIS3DH_AXIS_Z];
		}

		if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z],
					data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
		}

		if (atomic_read(&priv->filter)) {
			if (atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
				int idx, firlen = atomic_read(&priv->firlen);

				if (priv->fir.num < firlen) {
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_X] = data[LIS3DH_AXIS_X];
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Y] = data[LIS3DH_AXIS_Y];
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Z] = data[LIS3DH_AXIS_Z];
					priv->fir.sum[LIS3DH_AXIS_X] += data[LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] += data[LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] += data[LIS3DH_AXIS_Z];
					if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][LIS3DH_AXIS_X], priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Y], priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Z],
							priv->fir.sum[LIS3DH_AXIS_X], priv->fir.sum[LIS3DH_AXIS_Y], priv->fir.sum[LIS3DH_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				} else {
					idx = priv->fir.idx % firlen;
					priv->fir.sum[LIS3DH_AXIS_X] -= priv->fir.raw[idx][LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] -= priv->fir.raw[idx][LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] -= priv->fir.raw[idx][LIS3DH_AXIS_Z];
					priv->fir.raw[idx][LIS3DH_AXIS_X] = data[LIS3DH_AXIS_X];
					priv->fir.raw[idx][LIS3DH_AXIS_Y] = data[LIS3DH_AXIS_Y];
					priv->fir.raw[idx][LIS3DH_AXIS_Z] = data[LIS3DH_AXIS_Z];
					priv->fir.sum[LIS3DH_AXIS_X] += data[LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] += data[LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] += data[LIS3DH_AXIS_Z];
					priv->fir.idx++;
					data[LIS3DH_AXIS_X] = priv->fir.sum[LIS3DH_AXIS_X]/firlen;
					data[LIS3DH_AXIS_Y] = priv->fir.sum[LIS3DH_AXIS_Y]/firlen;
					data[LIS3DH_AXIS_Z] = priv->fir.sum[LIS3DH_AXIS_Z]/firlen;
					if (atomic_read(&priv->trace) & ADX_TRC_FILTER) {
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][LIS3DH_AXIS_X], priv->fir.raw[idx][LIS3DH_AXIS_Y], priv->fir.raw[idx][LIS3DH_AXIS_Z],
						priv->fir.sum[LIS3DH_AXIS_X], priv->fir.sum[LIS3DH_AXIS_Y], priv->fir.sum[LIS3DH_AXIS_Z],
						data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
					}
				}
			}
		}
	}
	return err;
}

static int lis3dh_resetCalibration(struct i2c_client *client)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}

static int lis3dh_readCalibration(struct i2c_client *client,
		int dat[LIS3DH_AXES_NUM])
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	dat[obj->cvt.map[LIS3DH_AXIS_X]] = obj->cvt.sign[LIS3DH_AXIS_X]*obj->cali_sw[LIS3DH_AXIS_X];
	dat[obj->cvt.map[LIS3DH_AXIS_Y]] = obj->cvt.sign[LIS3DH_AXIS_Y]*obj->cali_sw[LIS3DH_AXIS_Y];
	dat[obj->cvt.map[LIS3DH_AXIS_Z]] = obj->cvt.sign[LIS3DH_AXIS_Z]*obj->cali_sw[LIS3DH_AXIS_Z];

	return 0;
}

static int lis3dh_writeCalibration(struct i2c_client *client,
		int dat[LIS3DH_AXES_NUM])
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	s16 cali[LIS3DH_AXES_NUM];

	GSE_FUN();
	if (!obj || !dat) {
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}

	cali[obj->cvt.map[LIS3DH_AXIS_X]] = obj->cvt.sign[LIS3DH_AXIS_X]*obj->cali_sw[LIS3DH_AXIS_X];
	cali[obj->cvt.map[LIS3DH_AXIS_Y]] = obj->cvt.sign[LIS3DH_AXIS_Y]*obj->cali_sw[LIS3DH_AXIS_Y];
	cali[obj->cvt.map[LIS3DH_AXIS_Z]] = obj->cvt.sign[LIS3DH_AXIS_Z]*obj->cali_sw[LIS3DH_AXIS_Z];
	cali[LIS3DH_AXIS_X] += dat[LIS3DH_AXIS_X];
	cali[LIS3DH_AXIS_Y] += dat[LIS3DH_AXIS_Y];
	cali[LIS3DH_AXIS_Z] += dat[LIS3DH_AXIS_Z];

	obj->cali_sw[LIS3DH_AXIS_X] += obj->cvt.sign[LIS3DH_AXIS_X]*dat[obj->cvt.map[LIS3DH_AXIS_X]];
	obj->cali_sw[LIS3DH_AXIS_Y] += obj->cvt.sign[LIS3DH_AXIS_Y]*dat[obj->cvt.map[LIS3DH_AXIS_Y]];
	obj->cali_sw[LIS3DH_AXIS_Z] += obj->cvt.sign[LIS3DH_AXIS_Z]*dat[obj->cvt.map[LIS3DH_AXIS_Z]];

	return err;
}

static int lis3dh_setPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = LIS3DH_REG_CTL_REG1;
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return LIS3DH_SUCCESS;
	}

	if ((lis_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
		GSE_ERR("read power ctl register err!\n");
		return LIS3DH_ERR_I2C;
	}

	if (enable == true)
		databuf[0] &= ~LIS3DH_MEASURE_MODE;
	else
		databuf[0] |= LIS3DH_MEASURE_MODE;

	res = lis_i2c_write_block(client, LIS3DH_REG_CTL_REG1, databuf, 0x1);

	if (res <= 0) {
		GSE_LOG("set power mode failed!\n");
		return LIS3DH_ERR_I2C;
	} else if (atomic_read(&obj->trace) & ADX_TRC_INFO) {
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;
	return LIS3DH_SUCCESS;
}

static int lis3dh_setDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = LIS3DH_REG_CTL_REG4;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if ((lis_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] &= ~0x30;
	databuf[0] |= dataformat;

	res = lis_i2c_write_block(client, LIS3DH_REG_CTL_REG4, databuf, 0x1);

	if (res < 0)
		return LIS3DH_ERR_I2C;

	return lis3dh_setDataResolution(obj);
}

static int lis3dh_setBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	u8 addr = LIS3DH_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if ((lis_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] &= ~0xF0;
	databuf[0] |= bwrate;

	res = lis_i2c_write_block(client, LIS3DH_REG_CTL_REG1, databuf, 0x1);

	if (res < 0)
		return LIS3DH_ERR_I2C;

	return LIS3DH_SUCCESS;
}

/* enable data ready interrupt */
static int lis3dh_setIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	u8 addr = LIS3DH_REG_CTL_REG3;
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 2);

	if ((lis_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] = 0x00;

	res = lis_i2c_write_block(client, LIS3DH_REG_CTL_REG3, databuf, 0x01);
	if (res < 0)
		return LIS3DH_ERR_I2C;

	return LIS3DH_SUCCESS;
}

static int lis3dh_accel_init(struct i2c_client *client, int reset_cali)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[2] = {0, 0};

	/* first clear reg1 */
	databuf[0] = 0x0f;
	res = lis_i2c_write_block(client, LIS3DH_REG_CTL_REG1, databuf, 0x01);
	if (res < 0) {
		GSE_ERR("lis3dh_accel_init step 1!\n");
		return res;
	}

	res = lis3dh_setBWRate(client, LIS3DH_BW_100HZ); /* 400 or 100 no other choice */
	if (res < 0) {
		GSE_ERR("lis3dh_accel_init step 2!\n");
		return res;
	}

	res = lis3dh_setDataFormat(client, LIS3DH_RANGE_2G); /* 8g or 2G no oher choise */
	if (res < 0) {
		GSE_ERR("lis3dh_accel_init step 3!\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = lis3dh_setIntEnable(client, false);
	if (res < 0) {
		GSE_ERR("lis3dh_accel_init step 4!\n");
		return res;
	}

	res = lis3dh_setPowerMode(client, enable_status); /* false */
	if (res < 0) {
		GSE_ERR("lis3dh_accel_init step 5!\n");
		return res;
	}

	if (0 != reset_cali) {
		/* reset calibration only in power on */
		res = lis3dh_resetCalibration(client);
		if (res < 0)
			return res;
	}

	memset(&obj->fir, 0x00, sizeof(obj->fir));

	return LIS3DH_SUCCESS;
}

static int lis3dh_readChipInfo(struct i2c_client *client,
		char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS3DH Chip");
	return 0;
}

static int lis3dh_readSensorData(struct i2c_client *client,
		char *buf, int bufsize)
{
	struct lis3dh_i2c_data *obj =
			(struct lis3dh_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LIS3DH_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if (NULL == buf)
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_suspend == 1) {
		return 0;
	}

	res = lis3dh_readData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}

	obj->data[LIS3DH_AXIS_X] += obj->cali_sw[LIS3DH_AXIS_X];
	obj->data[LIS3DH_AXIS_Y] += obj->cali_sw[LIS3DH_AXIS_Y];
	obj->data[LIS3DH_AXIS_Z] += obj->cali_sw[LIS3DH_AXIS_Z];

	/*remap coordinate*/
	acc[obj->cvt.map[LIS3DH_AXIS_X]] = obj->cvt.sign[LIS3DH_AXIS_X]*obj->data[LIS3DH_AXIS_X];
	acc[obj->cvt.map[LIS3DH_AXIS_Y]] = obj->cvt.sign[LIS3DH_AXIS_Y]*obj->data[LIS3DH_AXIS_Y];
	acc[obj->cvt.map[LIS3DH_AXIS_Z]] = obj->cvt.sign[LIS3DH_AXIS_Z]*obj->data[LIS3DH_AXIS_Z];

	/* Out put the mg */
	acc[LIS3DH_AXIS_X] = acc[LIS3DH_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[LIS3DH_AXIS_Y] = acc[LIS3DH_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[LIS3DH_AXIS_Z] = acc[LIS3DH_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

        sprintf(buf, "%04x %04x %04x", acc[LIS3DH_AXIS_X], acc[LIS3DH_AXIS_Y], acc[LIS3DH_AXIS_Z]);
	if (atomic_read(&obj->trace) & ADX_TRC_IOCTL) { /* atomic_read(&obj->trace) & ADX_TRC_IOCTL */
		GSE_LOG("gsensor data: %s!\n", buf);
		dumpReg(client);
	}

	return 0;
}

static int lis3dh_readRawData(struct i2c_client *client, char *buf)
{
	struct lis3dh_i2c_data *obj = (struct lis3dh_i2c_data *)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return -EINVAL;

	res = lis3dh_readData(client, obj->data);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}

	sprintf(buf, "%04x %04x %04x", obj->data[LIS3DH_AXIS_X],
		obj->data[LIS3DH_AXIS_Y], obj->data[LIS3DH_AXIS_Z]);

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	char strbuf[LIS3DH_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	lis3dh_readChipInfo(client, strbuf, LIS3DH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	char strbuf[LIS3DH_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	lis3dh_readSensorData(client, strbuf, LIS3DH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[LIS3DH_AXES_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	err = lis3dh_readCalibration(client, tmp);
	if (err)
		return -EINVAL;

	mul = obj->reso->sensitivity/lis3dh_offset_resolution.sensitivity;
	len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
		obj->offset[LIS3DH_AXIS_X], obj->offset[LIS3DH_AXIS_Y], obj->offset[LIS3DH_AXIS_Z],
		obj->offset[LIS3DH_AXIS_X], obj->offset[LIS3DH_AXIS_Y], obj->offset[LIS3DH_AXIS_Z]);
	len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		obj->cali_sw[LIS3DH_AXIS_X], obj->cali_sw[LIS3DH_AXIS_Y], obj->cali_sw[LIS3DH_AXIS_Z]);

	len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
		obj->offset[LIS3DH_AXIS_X]*mul + obj->cali_sw[LIS3DH_AXIS_X],
		obj->offset[LIS3DH_AXIS_Y]*mul + obj->cali_sw[LIS3DH_AXIS_Y],
		obj->offset[LIS3DH_AXIS_Z]*mul + obj->cali_sw[LIS3DH_AXIS_Z],
		tmp[LIS3DH_AXIS_X], tmp[LIS3DH_AXIS_Y], tmp[LIS3DH_AXIS_Z]);

	return len;
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = lis3dh_i2c_client;
	int err, x, y, z;
	int dat[LIS3DH_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = lis3dh_resetCalibration(client);
		if (err)
			GSE_ERR("reset offset err = %d\n", err);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[LIS3DH_AXIS_X] = x;
		dat[LIS3DH_AXIS_Y] = y;
		dat[LIS3DH_AXIS_Z] = z;
		err = lis3dh_writeCalibration(client, dat);
		if (err)
			GSE_ERR("write calibration err = %d\n", err);
	} else
		GSE_ERR("invalid format\n");

	return count;
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj;
	u8 data;

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	lis_i2c_read_block(client, LIS3DH_REG_CTL_REG1, &data, 0x01);
	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++)
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][LIS3DH_AXIS_X], obj->fir.raw[idx][LIS3DH_AXIS_Y], obj->fir.raw[idx][LIS3DH_AXIS_Z]);

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[LIS3DH_AXIS_X], obj->fir.sum[LIS3DH_AXIS_Y], obj->fir.sum[LIS3DH_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[LIS3DH_AXIS_X]/len, obj->fir.sum[LIS3DH_AXIS_Y]/len, obj->fir.sum[LIS3DH_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
}

static ssize_t store_firlen_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (1 != sscanf(buf, "%d", &firlen)) {
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (0 == firlen) {
			atomic_set(&obj->fir_en, 0);
		} else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}

	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis3dh_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct lis3dh_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lis3dh_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
		obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id,
		obj->hw.power_vol);

	return len;
}

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(power, S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);

static struct driver_attribute *lis3dh_attr_list[] = {
	&driver_attr_chipinfo, /* chip information */
	&driver_attr_sensordata, /* dump sensor data */
	&driver_attr_cali, /* show calibration data */
	&driver_attr_power, /* show power reg */
	&driver_attr_firlen, /* filter length: 0: disable, others: enable */
	&driver_attr_trace, /* trace log */
	&driver_attr_status, /* device status */
};

static int lis3dh_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lis3dh_attr_list) / sizeof(lis3dh_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, lis3dh_attr_list[idx]);
		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n", lis3dh_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int lis3dh_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lis3dh_attr_list) / sizeof(lis3dh_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, lis3dh_attr_list[idx]);

	return err;
}

#if 0
int lis3dh_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value, sample_delay;
	struct lis3dh_i2c_data *priv = (struct lis3dh_i2c_data *)self;
	struct hwm_sensor_data *gsensor_data;
	char buff[LIS3DH_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Set delay parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			if (value <= 5)
				sample_delay = LIS3DH_BW_200HZ;
			else if (value <= 10)
				sample_delay = LIS3DH_BW_100HZ;
			else
				sample_delay = LIS3DH_BW_50HZ;

			mutex_lock(&lis3dh_op_mutex);
			err = lis3dh_setBWRate(priv->client, sample_delay);
			if (err != LIS3DH_SUCCESS) /* 0x2C->BW=100Hz */
				GSE_ERR("Set delay parameter error!\n");
			mutex_unlock(&lis3dh_op_mutex);
			if (value >= 50)
				atomic_set(&priv->filter, 0);
			else {
				priv->fir.num = 0;
				priv->fir.idx = 0;
				priv->fir.sum[LIS3DH_AXIS_X] = 0;
				priv->fir.sum[LIS3DH_AXIS_Y] = 0;
				priv->fir.sum[LIS3DH_AXIS_Z] = 0;
				atomic_set(&priv->filter, 1);
			}
		}
		break;

	case SENSOR_ENABLE:
		if ((buff_in == NULL) || (size_in < sizeof(int))) {
			GSE_ERR("Enable sensor parameter error!\n");
			err = -EINVAL;
		} else {
			value = *(int *)buff_in;
			mutex_lock(&lis3dh_op_mutex);
			GSE_LOG("Gsensor device enable function enable = %d, sensor_power = %d!\n", value, sensor_power);
			if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true))) {
				enable_status = sensor_power;
				GSE_LOG("Gsensor device have updated!\n");
			} else {
				enable_status = !sensor_power;
				err = lis3dh_setPowerMode(priv->client, !sensor_power);
				GSE_LOG("Gsensor not in suspend lis3dh_SetPowerMode!, enable_status = %d\n", enable_status);
			}
			mutex_unlock(&lis3dh_op_mutex);
		}
		break;

	case SENSOR_GET_DATA:
		if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
			GSE_ERR("get sensor data parameter error!\n");
			err = -EINVAL;
		} else {
			mutex_lock(&lis3dh_op_mutex);
			gsensor_data = (struct hwm_sensor_data *)buff_out;
			lis3dh_readSensorData(priv->client, buff, LIS3DH_BUFSIZE);
			sscanf(buff, "%x %x %x", &gsensor_data->values[0],
				&gsensor_data->values[1], &gsensor_data->values[2]);
			gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			gsensor_data->value_divide = 1000;
			mutex_unlock(&lis3dh_op_mutex);
		}
		break;

	default:
		GSE_ERR("gsensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;
}
#endif

static int lis3dh_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();
	mutex_lock(&lis3dh_op_mutex);
	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			mutex_unlock(&lis3dh_op_mutex);
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		/* read old data */
		err = lis3dh_setPowerMode(obj->client, false);
		if (err) {
				GSE_ERR("write power control fail!!\n");
				mutex_unlock(&lis3dh_op_mutex);
				return err;
			}

		atomic_set(&obj->suspend, 1);
	}
	sensor_suspend = 1;
	mutex_unlock(&lis3dh_op_mutex);
	return err;
}

static int lis3dh_resume(struct i2c_client *client)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();
	mutex_lock(&lis3dh_op_mutex);
	if (obj == NULL) {
		mutex_unlock(&lis3dh_op_mutex);
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	err = lis3dh_accel_init(client, 0);
	if (err) {
		mutex_unlock(&lis3dh_op_mutex);
		GSE_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->suspend, 0);
	sensor_suspend = 0;
	mutex_unlock(&lis3dh_op_mutex);
	return 0;
}

static int lis3dh_i2c_detect(struct i2c_client *client,
		struct i2c_board_info *info)
{
	strcpy(info->type, LIS3DH_DEV_NAME);
	return 0;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int lis3dh_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

static int lis3dh_enable_nodata(int en)
{
	int res = 0;
	bool power = false;

	if (1 == en)
		power = true;
	else if (0 == en)
		power = false;

	res = lis3dh_setPowerMode(obj_i2c_data->client, power);
	if (res != LIS3DH_SUCCESS) {
		GSE_ERR("lis3dh_setPowerMode fail!\n");
		return -1;
	}

	GSE_LOG("lis3dh_enable_nodata OK!\n");
	return 0;
}

static int lis3dh_set_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000000;

	GSE_LOG("lis3dh set delay = (%d) OK!\n", value);
	return 0;

}
static int lis3dh_flush(void)
{
	return acc_flush_report();
}

static int lis3dh_set_delay(u64 ns)
{
	int value = 0;
	int sample_delay = 0;
	int err;

	value = (int)ns / 1000000;
	if (value <= 5)
		sample_delay = LIS3DH_BW_200HZ;
	else if (value <= 10)
		sample_delay = LIS3DH_BW_100HZ;
	else
		sample_delay = LIS3DH_BW_50HZ;

	mutex_lock(&lis3dh_op_mutex);

	err = lis3dh_setBWRate(obj_i2c_data->client, sample_delay);
	if (err != LIS3DH_SUCCESS) { /* 0x2C->BW=100Hz */
		GSE_ERR("Set delay parameter error!\n");
	}

	mutex_unlock(&lis3dh_op_mutex);
	if (value >= 50) {
		atomic_set(&obj_i2c_data->filter, 0);
	} else {
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_X] = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[LIS3DH_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
	}

	GSE_LOG("lis3dh_set_delay (%d)\n", value);
	return 0;
}

static int lis3dh_get_data(int *x , int *y, int *z, int *status)
{
	char buff[LIS3DH_BUFSIZE];

	*x = *y = *z = 0;
	lis3dh_readSensorData(obj_i2c_data->client, buff, LIS3DH_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int lis3dh_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = lis3dh_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}

	err = lis3dh_set_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}

	return 0;
}
static int lis3dh_factory_get_data(int32_t data[3], int *status)
{
	return lis3dh_get_data(&data[0], &data[1], &data[2], status);
}

static int lis3dh_factory_get_raw_data(int32_t data[3])
{
	char strbuf[LIS3DH_BUFSIZE] = {0};

	lis3dh_readRawData(lis3dh_i2c_client, strbuf);
	GSE_LOG("support lis3dh_factory_get_raw_data!\n");
	return 0;
}

static int lis3dh_factory_enable_calibration(void)
{
	return 0;
}

static int lis3dh_factory_clear_cali(void)
{
	int err = 0;

	err = lis3dh_resetCalibration(lis3dh_i2c_client);
	if (err) {
		GSE_ERR("lis3dh_resetCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int lis3dh_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	/* obj */
	obj_i2c_data->cali_sw[LIS3DH_AXIS_X] += data[0];
	obj_i2c_data->cali_sw[LIS3DH_AXIS_Y] += data[1];
	obj_i2c_data->cali_sw[LIS3DH_AXIS_Z] += data[2];

	cali[LIS3DH_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[LIS3DH_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[LIS3DH_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;

	err = lis3dh_writeCalibration(lis3dh_i2c_client, cali);
	if (err) {
		GSE_ERR("lis3dh_writeCalibration failed!\n");
		return -1;
	}

	return 0;
}

static int lis3dh_factory_get_cali(int32_t data[3])
{
	data[0] = obj_i2c_data->cali_sw[LIS3DH_AXIS_X];
	data[1] = obj_i2c_data->cali_sw[LIS3DH_AXIS_Y];
	data[2] = obj_i2c_data->cali_sw[LIS3DH_AXIS_Z];
	return 0;
}

static int lis3dh_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops lis3dh_factory_fops = {
	.enable_sensor = lis3dh_factory_enable_sensor,
	.get_data = lis3dh_factory_get_data,
	.get_raw_data = lis3dh_factory_get_raw_data,
	.enable_calibration = lis3dh_factory_enable_calibration,
	.clear_cali = lis3dh_factory_clear_cali,
	.set_cali = lis3dh_factory_set_cali,
	.get_cali = lis3dh_factory_get_cali,
	.do_self_test = lis3dh_factory_do_self_test,
};

static struct accel_factory_public lis3dh_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &lis3dh_factory_fops,
};

static int lis3dh_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_client *new_client = NULL;
	struct lis3dh_i2c_data *obj = NULL;
	int err = 0;
	int retry = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};

	GSE_FUN();
	GSE_LOG("lis3dh_i2c_probe\n");

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GSE_ERR("get dts info fail\n");
		goto exit_kfree;
	}

	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	if (obj->hw.firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw.firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);

	lis3dh_i2c_client = new_client;

	for (retry = 0; retry < 3; retry++) {
		err = lis3dh_accel_init(new_client, 1);
		if (err) {
			GSE_ERR("lis3dh_device init cilent fail time: %d\n", retry);
			continue;
		}
	}

	if (err != 0)
		goto exit_init_failed;

	ctl.is_use_common_factory = false;

	/* factory */
	err = accel_factory_device_register(&lis3dh_factory_device);
	if (err) {
		GSE_ERR("acc_factory register failed.\n");
		goto exit_misc_device_register_failed;
	}

	err = lis3dh_create_attr(&(lis3dh_init_info.platform_diver_addr->driver));
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = lis3dh_open_report_data;
	ctl.enable_nodata = lis3dh_enable_nodata;
	ctl.set_delay  = lis3dh_set_delay;
	ctl.batch = lis3dh_set_batch;
	ctl.flush = lis3dh_flush;
	ctl.is_report_input_direct = false;

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = lis3dh_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	GSE_LOG("%s: OK\n", __func__);
	lis3dh_init_flag = 0;
	return 0;

exit_create_attr_failed:
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	lis3dh_init_flag = -1;
	obj = NULL;
	new_client = NULL;
	obj_i2c_data = NULL;
	lis3dh_i2c_client = NULL;
	return err;
}

static int lis3dh_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = lis3dh_delete_attr(&(lis3dh_init_info.platform_diver_addr->driver));
	if (err)
		GSE_ERR("lis3dh_delete_attr fail: %d\n", err);

	lis3dh_i2c_client = NULL;
	i2c_unregister_device(client);
	accel_factory_device_deregister(&lis3dh_factory_device);
	kfree(i2c_get_clientdata(client));
	return 0;
}


static int lis3dh_remove(void)
{
	GSE_FUN();
	i2c_del_driver(&lis3dh_i2c_driver);
	return 0;
}

static int lis3dh_local_init(void)
{
	GSE_FUN();

	if (i2c_add_driver(&lis3dh_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}

	if (lis3dh_init_flag == -1)
		return -1;

	return 0;
}


static int __init lis3dh_init(void)
{
	GSE_FUN();
	acc_driver_add(&lis3dh_init_info);
	return 0;
}

static void __exit lis3dh_exit(void)
{
	GSE_FUN();
}

module_init(lis3dh_init);
module_exit(lis3dh_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS3DH Accelerometer Driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
