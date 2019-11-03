
/* mmc3524x.c - mmc3524x compass driver
 *
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
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <hwmsensor.h>
//#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/types.h>

#include <hwmsen_helper.h>
//#include <batch.h>


#define MEMSIC_SINGLE_POWER 0

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_mag.h>
#include "mmc3524x.h"

#include "mag.h"
/*----------------------------------------------------------------------------*/
#define DEBUG 0
#define MMC3524X_DEV_NAME         "mmc3524x"
#define DRIVER_VERSION          "1.1.0"
/*----------------------------------------------------------------------------*/
#define MMC3524X_DEBUG		1
#define MMC3524X_DEBUG_MSG	1
#define MMC3524X_DEBUG_FUNC	1
#define MMC3524X_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define MMC3524X_RETRY_COUNT	3
#define MMC3524X_DEFAULT_DELAY	100
#define MMC3524X_BUFSIZE  0x20
#define MMC3524X_DELAY_RM	10	/* ms */
#define READMD			0

#if MMC3524X_DEBUG
#define MAGN_TAG		 "[MMC3524X] "
#define MAGN_ERR(fmt, args...)	pr_err(MAGN_TAG fmt, ##args)
#define MAGN_LOG(fmt, args...)	pr_debug(MAGN_TAG fmt, ##args)
#else
#define MAGN_TAG
#define MAGN_ERR(fmt, args...)	do {} while (0)
#define MAGN_LOG(fmt, args...)	do {} while (0)
#endif


#define CONFIG_MTK_I2C_EXTENSION 1

#if MMC3524X_DEBUG_MSG
#define MMCDBG(format, ...)	printk(KERN_INFO "mmc3524x " format "\n", ## __VA_ARGS__)
#else
#define MMCDBG(format, ...)
#endif

#if MMC3524X_DEBUG_FUNC
#define MMCFUNC(func) printk(KERN_INFO "mmc3524x " func " is called\n")
#else
#define MMCFUNC(func)
#endif

static struct i2c_client *this_client = NULL;

// calibration msensor and orientation data
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int mmcd_delay = MMC3524X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

static const struct i2c_device_id mmc3524x_i2c_id[] = {{MMC3524X_DEV_NAME,0},{}};

/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mmc3524x_i2c_remove(struct i2c_client *client);
static int mmc3524x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg);
static int mmc3524x_resume(struct i2c_client *client);
static int mmc3524x_local_init(void);
static int mmc3524x_remove(void);

typedef enum {
	MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
} MMC_TRC;

/* Define Delay time */
#define MMC3524X_DELAY_TM		10	/* ms */
#define MMC3524X_DELAY_SET		50	/* ms */
#define MMC3524X_DELAY_RESET	50  /* ms */
#define MMC3524X_DELAY_STDN		1	/* ms */

#define MMC3524X_RESET_INTV		250
static int otpMatrix[3] = {0};
static u32 read_idx = 0;
#define READMD	0

struct mmc3524x_i2c_data {
    struct i2c_client *client;
    struct mag_hw hw;
    atomic_t layout;
    atomic_t trace;
	int sensor_type;
	struct hwmsen_convert   cvt;
};

static int  mmc3524x_local_init(void);
static int mmc3524x_remove(void);

static int mmc3524x_init_flag = 0; // 0<==>OK -1 <==> fail


static struct mag_init_info mmc3524x_init_info = {
	 .name = "mmc3524x",
	 .init = mmc3524x_local_init,
	 .uninit = mmc3524x_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {{.compatible = "mediatek,mmc3524x"}};
#endif

static struct i2c_driver mmc3524x_i2c_driver = {
    .driver = {
     //   .owner = THIS_MODULE,
        .name  = MMC3524X_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe      = mmc3524x_i2c_probe,
	.remove     = mmc3524x_i2c_remove,
	.detect     = mmc3524x_i2c_detect,
	.suspend    = mmc3524x_suspend,
	.resume     = mmc3524x_resume,
	.id_table = mmc3524x_i2c_id,
};

static atomic_t dev_open_count;

/******************************************************************************************************************************/
static DEFINE_MUTEX(mmc3x30x_i2c_mutex);
int mag_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,   .flags = 0,
			.len = 1,   .buf = &beg
		},
		{
			.addr = client->addr,   .flags = I2C_M_RD,
			.len = 1,   .buf = data,
		}
	};

	if (!client)
		return -EINVAL;

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		printk("i2c_transfer error: (%d %p) %d\n", addr, data, err);
		err = -EIO;
	}

	err = 0;

	return err;
}

static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&mmc3x30x_i2c_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&mmc3x30x_i2c_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3x30x_i2c_mutex);
		MAGN_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MAGN_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&mmc3x30x_i2c_mutex);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{               /*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&mmc3x30x_i2c_mutex);
	if (!client) {
		mutex_unlock(&mmc3x30x_i2c_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&mmc3x30x_i2c_mutex);
		MAGN_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&mmc3x30x_i2c_mutex);
		MAGN_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&mmc3x30x_i2c_mutex);
	return err;
}

static int I2C_RxData(char *rxData, int length)
{
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = rxData[0];

	if ((rxData == NULL) || (length < 1))
		return -EINVAL;
	res = mag_i2c_read_block(client, addr, rxData, length);
	if (res < 0)
		return -1;
	return 0;

}

static int I2C_TxData(char *txData, int length)
{
	struct i2c_client *client = this_client;
	int res = 0;
	char addr = txData[0];
	u8 *buff = &txData[1];

	if ((txData == NULL) || (length < 2))
		return -EINVAL;
	res = mag_i2c_write_block(client, addr, buff, (length - 1));
	if (res < 0)
		return -1;
	return 0;
}

static int mmc3524x_read_otp(void)
{
	int stemp = 0, utemp = 0, rc = 0;
	unsigned char otpdata[4] = {0};

	otpdata[0] = MMC3524X_MAG_REG_ADDR_OTP;
	rc = I2C_RxData(otpdata, 4);
	if (rc < 0)
		return rc;

	otpMatrix[0] = 1000;

	stemp = ((otpdata[1]&0x03) << 4) | (otpdata[2] >> 4);

	if (stemp >= 32)
		stemp = 32 - stemp;

	otpMatrix[1] = stemp * 6 + 1000;

	stemp = otpdata[3] & 0x3f;
	if (stemp >= 32)
		stemp = 32 - stemp;
	utemp = stemp * 6 + 1000;
	otpMatrix[2] = utemp + (utemp * 3) / 10 + (utemp * 30 % 100 + utemp * 5) / 100;

	MAGN_LOG("memsic otp matrix = %d %d %d\n", otpMatrix[0], otpMatrix[1], otpMatrix[2]);

	return 0;
}

static int mmc3524x_processor_otp(int* in, int* out)
{
	out[0] = in[0] * otpMatrix[0];
	out[1] = in[1] * otpMatrix[1];
	out[2] = in[2] * otpMatrix[2];

	return 0;
}

static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
	int magraw[3] = {0};
	struct i2c_client *client = this_client;
	struct mmc3524x_i2c_data *clientdata = i2c_get_clientdata(client);

	if(size < 3)
	{
		MAGN_ERR("Invalid size value\n");
		return -1;
	}
	mutex_lock(&read_i2c_xyz);

	if (!(read_idx % MMC3524X_RESET_INTV))
	{
		/* Reset Sensor Periodly SET */
#if MEMSIC_SINGLE_POWER
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_REFILL;
		/* not check return value here, assume it always OK */
		I2C_TxData(data, 2);
		/* wait external capacitor charging done for next RM */
		msleep(MMC3524X_DELAY_SET);
#endif
		data[0] = MMC3524X_REG_CTRL;
		data[1] = MMC3524X_CTRL_SET;
		I2C_TxData(data, 2);
		msleep(MMC3524X_DELAY_STDN);

		data[0] = MMC3524X_REG_CTRL;
		data[1] = 0;
		I2C_TxData(data, 2);
		msleep(MMC3524X_DELAY_STDN);
	}


	read_idx++;
	data[0] = MMC3524X_REG_DATA;
	if(I2C_RxData(data, 6) < 0)
	{
		mutex_unlock(&read_i2c_xyz);
		MAGN_ERR("i2c rxdata failed\n");
		return -EFAULT;
	}
	magraw[0] = (data[1] << 8 | data[0]) - 32768;
	magraw[1] = (data[3] << 8 | data[2]) - (data[5] << 8 | data[4]);
	magraw[2] = (data[3] << 8 | data[2]) + (data[5] << 8 | data[4]) - 65536;

	mmc3524x_processor_otp(magraw, vec);

	/* send TM cmd before read */
	data[0] = MMC3524X_REG_CTRL;
	data[1] = MMC3524X_CTRL_TM;
	/* not check return value here, assume it always OK */
	I2C_TxData(data, 2);

	if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG)
	{
		MAGN_LOG("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
	}

	mutex_unlock(&read_i2c_xyz);
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err;
    int data_temp[3] = {0};

	struct mmc3524x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		MAGN_ERR("mmc3680x_i2c_data is null!!\n");
		return 0;
	}

	err = ECS_ReadXYZData(data, 3);
	if(err != 0)
	{
		MAGN_ERR("MMC3524X_IOC_TM failed\n");
		return -1;
	}
    data_temp[0] = data[0];
    data_temp[1] = data[1];
    data_temp[2] = data[2];

    MAGN_LOG("coridate before %d %d %d\n",data[0],data[1],data[2]);
    data[obj->cvt.map[0]] = obj->cvt.sign[0] * data_temp[0];
    data[obj->cvt.map[1]] = obj->cvt.sign[1] * data_temp[1];
    data[obj->cvt.map[2]] = obj->cvt.sign[2] * data_temp[2];
    MAGN_LOG("coridate after %d %d %d\n",data[0],data[1],data[2]);

	return err;
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[MMC3524X_BUFSIZE];

	ECS_GetRawData(sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static struct driver_attribute *mmc3524x_attr_list[] = {
	&driver_attr_sensordata
};

static int mmc3524x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mmc3524x_attr_list[idx])))
		{
			printk(KERN_ERR "driver_create_file (%s) = %d\n", mmc3524x_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}

static int mmc3524x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3524x_attr_list)/sizeof(mmc3524x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc3524x_attr_list[idx]);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_suspend(struct i2c_client *client, pm_message_t msg)
{
	return 0;
}
static int mmc3524x_resume(struct i2c_client *client)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, MMC3524X_DEV_NAME);
	return 0;
}

static int mmc3524x_enable(int en)
{
	int value = 0;
	int err = 0;
	value = en;

	if(value == 1)
	{
		atomic_set(&m_flag, 1);
		atomic_set(&open_flag, 1);
	}
	else
	{
		atomic_set(&m_flag, 0);
		if(atomic_read(&o_flag) == 0)
		{
			atomic_set(&open_flag, 0);
		}
	}
	wake_up(&open_wq);
	return err;
}

static int mmc3524x_set_delay(u64 ns)
{
    int value=0;
	value = (int)ns/1000/1000;
	if(value <= 20)
    {
        mmcd_delay = 20;
    }
    else{
        mmcd_delay = value;
	}

	return 0;
}
static int mmc3524x_open_report_data(int open)
{
	return 0;
}

static int mmc3524x_get_data(int* x ,int* y,int* z, int* status)
{
	int mag[3];
	ECS_GetRawData(mag);
	*x = mag[0];
	*y = mag[1];
	*z = mag[2];
	*status = 3;
	return 0;
}

static int mmc3524x_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int mmc3524x_flush(void)
{
	return mag_flush_report();
}

static int memsic_check_device(struct i2c_client *client)
{
	int rc;
	unsigned char rd_buffer[2] = {0};
	struct mmc3524x_i2c_data *data;

	data = i2c_get_clientdata(client);

	data->sensor_type = 0;

	rd_buffer[0] = MMC3524X_CHIPID_ADDR;
	rc = I2C_RxData(rd_buffer, 1);
	if (rc)
	{
		printk("[mmc35xx] read id fail\n");
	}
	printk("mmc3524x id = 0x%x\n", rd_buffer[0]);
	if ((rd_buffer[0] & 0x3f) == MEMSIC_SENSOR_MMC3530X)
	{
		data->sensor_type = MEMSIC_SENSOR_MMC3530X;
		printk("memsic current device is mmc3524 id = %d\n", rd_buffer[0]);
		return 0;
	}

	if ((rd_buffer[0] & 0x3f) == MEMSIC_SENSOR_MMC3524X)
	{
		data->sensor_type = MEMSIC_SENSOR_MMC3524X;
		printk("memsic current device is mmc3524 id = %d\n", rd_buffer[0]);
		return 0;
	}

	rd_buffer[0] = MMC36XX_CHIPID_ADDR;
	rc = I2C_RxData(rd_buffer, 1);
	if (rc)
	{
		printk("[mmc36xx] read id fail\n");
	}

	printk("mmc3524x 1111id = 0x%x\n", rd_buffer[0]);
	if ((rd_buffer[0] & 0x3f) == MEMSIC_SENSOR_MMC3630X)
	{
		data->sensor_type = MEMSIC_SENSOR_MMC3630X;
		printk("memsic current device is mmc36xx id = %d\n", rd_buffer[0]);
		return 0;
	}

	if ((data->sensor_type != MEMSIC_SENSOR_MMC3630X)
			&& (data->sensor_type != MEMSIC_SENSOR_MMC3530X)
			&& (data->sensor_type != MEMSIC_SENSOR_MMC3524X))
	{
		printk("unknown memsic sensor type \n");
		return -1;
	}

	return rc;
}

static int mmc35xx_init_sensor(struct i2c_client *client)
{
	unsigned char buffer[2] = {0};

#if MEMSIC_SINGLE_POWER
	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_REFILL;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device refill cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_SET);
#endif
	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_SET;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device set cmd failed\n");
		return -1;
	}
	msleep(1);
	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = 0;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device ctrl cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_SET);

	buffer[0] = MMC3524X_REG_BITS;
	buffer[1] = MMC3524X_BITS_SLOW_16;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device bit16 cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_TM);

	buffer[0] = MMC3524X_REG_CTRL;
	buffer[1] = MMC3524X_CTRL_TM;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device tm cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_TM);

	return 0;
}

static int mmc36xx_init_sensor(struct i2c_client *client)
{
	unsigned char buffer[2] = {0};

#if MEMSIC_SINGLE_POWER
	buffer[0] = MMC36XX_REG_CTRL0;
	buffer[1] = MMC3630X_CTRL_REFILL;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc36xx_device refill cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_SET);
#endif
	buffer[0] = MMC36XX_REG_CTRL0;
	buffer[1] = MMC3630X_CTRL_SET;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc36xx_device set cmd failed\n");
		return -1;
	}
	msleep(1);
	buffer[0] = MMC36XX_REG_CTRL0;
	buffer[1] = 0;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc3524x_device ctrl cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_SET);


	buffer[0] = MMC36XX_REG_CTRL0;
	buffer[1] = MMC3630X_CTRL_TM;
	if (I2C_TxData(buffer, 2) < 0)
	{
		printk(KERN_ERR "mmc36xx_device tm cmd failed\n");
		return -1;
	}
	msleep(MMC3524X_DELAY_TM);

	return 0;
}

static int mmc3524x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = mmc3524x_enable(enabledisable == true ? 1 : 0);
	if (err) {
		MAGN_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = mmc3524x_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MAGN_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int mmc3524x_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	mmc3524x_get_data(&data[0], &data[1], &data[2], status);
	data[0] /= 10240;
	data[1] /= 10240;
	data[2] /= 10240;
	return 0;
}
static int mmc3524x_factory_get_raw_data(int32_t data[3])
{
	MAGN_LOG("do not support mmc3524x_factory_get_raw_data!\n");
	return 0;
}
static int mmc3524x_factory_enable_calibration(void)
{
	return 0;
}
static int mmc3524x_factory_clear_cali(void)
{
	return 0;
}
static int mmc3524x_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int mmc3524x_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int mmc3524x_factory_do_self_test(void)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static struct mag_factory_fops mmc3524x_factory_fops = {
	.enable_sensor = mmc3524x_factory_enable_sensor,
	.get_data = mmc3524x_factory_get_data,
	.get_raw_data = mmc3524x_factory_get_raw_data,
	.enable_calibration = mmc3524x_factory_enable_calibration,
	.clear_cali = mmc3524x_factory_clear_cali,
	.set_cali = mmc3524x_factory_set_cali,
	.get_cali = mmc3524x_factory_get_cali,
	.do_self_test = mmc3524x_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/

static struct mag_factory_public mmc3524x_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &mmc3524x_factory_fops,
};

static int mmc3524x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mmc3524x_i2c_data *data;
	int err = 0;
	struct mag_control_path ctl={0};
	struct mag_data_path mag_data={0};

    printk("%s: ++++\n", __func__);

	if(!(data = kmalloc(sizeof(struct mmc3524x_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc3524x_i2c_data));

	err = get_mag_dts_func(client->dev.of_node, &data->hw);
	if (err < 0) {
		MAGN_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}

	err = hwmsen_get_convert(data->hw.direction, &data->cvt);
	if (err) {
		MAGN_ERR("invalid direction: %d\n", data->hw.direction);
		goto exit;
	}

	atomic_set(&data->layout, data->hw.direction);
	atomic_set(&data->trace, 0);

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);

	/*init_waitqueue_head(&data_ready_wq);*/
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;
    msleep(10);

	if (memsic_check_device(client) < 0)
	{
		err = -ENOTSUPP;
		goto exit_kfree;
	}

	if (data->sensor_type == MEMSIC_SENSOR_MMC3630X)
	{
		mmc36xx_init_sensor(client);
	}
	else if ((data->sensor_type == MEMSIC_SENSOR_MMC3530X) || (data->sensor_type == MEMSIC_SENSOR_MMC3524X))
	{
		mmc35xx_init_sensor(client);
	}

	if (mmc3524x_read_otp()) {
		MAGN_ERR("otp read failed, setting default");
		otpMatrix[0] = 1000;
		otpMatrix[1] = 1000;
		otpMatrix[2] = 1350;
	}

	/* Register sysfs attribute */
	if((err = mmc3524x_create_attr(&(mmc3524x_init_info.platform_diver_addr->driver))))
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = mag_factory_device_register(&mmc3524x_factory_device);
	if (err) {
		MAGN_ERR("misc device register failed, err = %d\n", err);
		goto exit_factory_device_register_failed;
	}

	ctl.is_use_common_factory = false;
	ctl.enable = mmc3524x_enable;
	ctl.set_delay  = mmc3524x_set_delay;
	ctl.open_report_data = mmc3524x_open_report_data;
	ctl.batch = mmc3524x_batch;
	ctl.flush = mmc3524x_flush;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw.is_batch_supported;
	memcpy(ctl.libinfo.libname, "memsicd3524x", sizeof(ctl.libinfo.libname));

	err = mag_register_control_path(&ctl);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_hwm_attach_failed;
	}

	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = mmc3524x_get_data;

	err = mag_register_data_path(&mag_data);
	if(err)
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_hwm_attach_failed;
	}

	MMCDBG("%s: OK\n", __func__);
	printk("mmc3524X IIC probe successful !");

	mmc3524x_init_flag = 1;
	return 0;

exit_hwm_attach_failed:
	mag_factory_device_deregister(&mmc3524x_factory_device);
exit_factory_device_register_failed:
	mmc3524x_delete_attr(&(mmc3524x_init_info.platform_diver_addr->driver));
exit_sysfs_create_group_failed:
exit_kfree:
	kfree(data);
exit:
	printk(KERN_ERR "%s: err = %d\n", __func__, err);
	mmc3524x_init_flag = -1;
	this_client = NULL;
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc3524x_i2c_remove(struct i2c_client *client)
{
	int err;

	if((err = mmc3524x_delete_attr(&(mmc3524x_init_info.platform_diver_addr->driver))))
	{
		printk(KERN_ERR "mmc3524x_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	mag_factory_device_deregister(&mmc3524x_factory_device);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int	mmc3524x_local_init(void)
{
	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&mmc3524x_i2c_driver))
	{
		MMCDBG("add driver error\n");
		return -1;
	}
	if(-1 == mmc3524x_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}

static int mmc3524x_remove(void)
{
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&mmc3524x_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init mmc3524x_init(void)
{
	mag_driver_add(&mmc3524x_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit mmc3524x_exit(void) {
}
/*----------------------------------------------------------------------------*/
module_init(mmc3524x_init);
module_exit(mmc3524x_exit);

MODULE_DESCRIPTION("mmc3524x compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

