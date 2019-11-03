/* qmcX983.c - qmcX983 compass driver
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


#include "mag.h"
#include "qmcX983.h"
#include <cust_mag.h>

#define DEBUG 1
#define QMCX983_DEV_NAME         "qmcX983"
#define DRIVER_VERSION          "3.4"

#define MAX_FAILURE_COUNT	3
#define QMCX983_RETRY_COUNT	3
#define	QMCX983_BUFSIZE		0x20

#define QMCX983_AD0_CMP		1

#define QMCX983_AXIS_X            0
#define QMCX983_AXIS_Y            1
#define QMCX983_AXIS_Z            2
#define QMCX983_AXES_NUM          3

#define QMCX983_DEFAULT_DELAY 100


#define QMC6983_A1_D1             0
#define QMC6983_E1		  1	
#define QMC6983_E1_Metal          2
#define QMC7983_Vertical          3
#define QMC7983_Slope             4

#define CALIBRATION_DATA_SIZE   28

#define MSE_TAG					"[QMC-Msensor] "
#define MSE_FUN(f)				pr_info(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)	pr_err(MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)	pr_info(MSE_TAG fmt, ##args)


static int chip_id = QMC6983_E1;
static struct i2c_client *this_client = NULL;
static short qmcd_delay = QMCX983_DEFAULT_DELAY;

// calibration msensor and orientation data
static struct mutex read_i2c_xyz;

static int OTP_Kx;
static int OTP_Ky;

static atomic_t open_flag = ATOMIC_INIT(0);
static unsigned char v_open_flag = 0x00;

static const struct i2c_device_id qmcX983_i2c_id[] = {{QMCX983_DEV_NAME,0},{}};

static int qmcX983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmcX983_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int qmcX983_i2c_remove(struct i2c_client *client);
static int qmcX983_suspend(struct i2c_client *client, pm_message_t msg);
static int qmcX983_resume(struct i2c_client *client);

typedef enum {
    QMC_FUN_DEBUG  = 0x01,
	QMC_DATA_DEBUG = 0x02,
	QMC_HWM_DEBUG  = 0x04,
	QMC_CTR_DEBUG  = 0x08,
	QMC_I2C_DEBUG  = 0x10,
} QMC_TRC;

struct qmcX983_i2c_data {
    struct i2c_client *client;
    struct mag_hw hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
	short xy_sensitivity;
	short z_sensitivity;
};

#define DATA_AVG_DELAY 6
#ifdef CONFIG_OF
static const struct of_device_id mag_of_match[] = {
    { .compatible = "mediatek,qmcX983", },
    {},
};
#endif

static struct i2c_driver qmcX983_i2c_driver = {
    .driver = {
        .name  = QMCX983_DEV_NAME,
#ifdef CONFIG_OF
	.of_match_table = mag_of_match,
#endif
    },
	.probe = qmcX983_i2c_probe,
	.remove = qmcX983_i2c_remove,
	.detect = qmcX983_i2c_detect,
	.suspend = qmcX983_suspend,
	.resume = qmcX983_resume,
	.id_table = qmcX983_i2c_id,
};

static int qmcX983_local_init(void);
static int qmcX983_local_remove(void);
static int qmcX983_init_flag =-1; // 0<==>OK -1 <==> fail

static struct mag_init_info qmcX983_init_info = {
        .name = QMCX983_DEV_NAME,
        .init = qmcX983_local_init,
        .uninit = qmcX983_local_remove,
};

static int mag_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&read_i2c_xyz);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		MSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&read_i2c_xyz);
	return err;

}

static int mag_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{	
	/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&read_i2c_xyz);
	if (!client) {
		mutex_unlock(&read_i2c_xyz);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&read_i2c_xyz);
		MSE_ERR("send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&read_i2c_xyz);
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


/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref QMCX983_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */
static int qmcX983_read_mag_xyz(int *data)
{
	int res;
	unsigned char mag_data[6];
	unsigned char databuf[6];
	int hw_d[3] = {0};

	int t1 = 0;
	unsigned char rdy = 0;
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *clientdata = i2c_get_clientdata(client);

    MSE_FUN();

	/* Check status register for data availability */
	while(!(rdy & 0x07) && (t1<3)){
		databuf[0]=STA_REG_ONE;
		res=I2C_RxData(databuf,1);
		rdy=databuf[0];
		MSE_LOG("QMCX983 Status register is (%02X)\n", rdy);
		t1 ++;
	}

	databuf[0] = OUT_X_L;

	res = I2C_RxData(databuf, 6);
	if(res != 0)
    {
		return -EFAULT;
	}

	memcpy(mag_data,databuf,sizeof(databuf));

	MSE_LOG("QMCX983 mag_data[%02x, %02x, %02x, %02x, %02x, %02x]\n",
		mag_data[0], mag_data[1], mag_data[2],
		mag_data[3], mag_data[4], mag_data[5]);

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	//Unit:mG  1G = 100uT = 1000mG
	hw_d[0] = hw_d[0] * 1000 / clientdata->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / clientdata->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / clientdata->z_sensitivity;

	MSE_LOG("Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);

	data[QMCX983_AXIS_X] = clientdata->cvt.sign[QMCX983_AXIS_X]*hw_d[clientdata->cvt.map[QMCX983_AXIS_X]];
	data[QMCX983_AXIS_Y] = clientdata->cvt.sign[QMCX983_AXIS_Y]*hw_d[clientdata->cvt.map[QMCX983_AXIS_Y]];
	data[QMCX983_AXIS_Z] = clientdata->cvt.sign[QMCX983_AXIS_Z]*hw_d[clientdata->cvt.map[QMCX983_AXIS_Z]];

	MSE_LOG("QMCX983 data [%d, %d, %d]\n", data[0], data[1], data[2]);

	return res;
}


/* Set the Gain range */
int qmcX983_set_range(short range)
{
	int err = 0;
	unsigned char data[2];
	struct qmcX983_i2c_data *obj = i2c_get_clientdata(this_client);

	int ran ;
	switch (range) {
	case QMCX983_RNG_2G:
		ran = RNG_2G;
		break;
	case QMCX983_RNG_8G:
		ran = RNG_8G;
		break;
	case QMCX983_RNG_12G:
		ran = RNG_12G;
		break;
	case QMCX983_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}

	obj->xy_sensitivity = 20000/ran;
	obj->z_sensitivity = 20000/ran;

	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xcf;
	data[0] |= (range << 4);
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);
	return err;

}

/* Set the sensor mode */
int qmcX983_set_mode(char mode)
{
	int err = 0;

	unsigned char data[2];
	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xfc;
	data[0] |= mode;
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	MSE_LOG("QMCX983 in qmcX983_set_mode, data[1] = [%02x]", data[1]);
	err = I2C_TxData(data, 2);

	return err;
}

int qmcX983_set_ratio(char ratio)
{
	int err = 0;

	unsigned char data[2];
	data[0] = 0x0b;//RATIO_REG;
	data[1] = ratio;
	err = I2C_TxData(data, 2);
	return err;
}

static void qmcX983_start_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1d;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);

}

static void qmcX983_stop_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1c;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);
}

static int qmcX983_init_hw(struct i2c_client *client)
{
	unsigned char data[2];
	int err;

	data[1] = 0x1;
	data[0] = 0x21;
	err = I2C_TxData(data, 2);

	data[1] = 0x40;
	data[0] = 0x20;
	err = I2C_TxData(data, 2);

  	if(chip_id != QMC6983_A1_D1)
	{

		data[1] = 0x80;
		data[0] = 0x29;
		err = I2C_TxData(data, 2); 		

		data[1] = 0x0c;
		data[0] = 0x0a;
		err = I2C_TxData(data, 2);				
	}
	
	if(chip_id == QMC6983_E1_Metal || chip_id == QMC7983_Slope)
	{
		data[1] = 0x80;
		data[0] = 0x1b;
		err = I2C_TxData(data, 2); 			
	}
	
	MSE_LOG("start measure!\n");
	qmcX983_start_measure(client);

	qmcX983_set_range(QMCX983_RNG_8G);
	qmcX983_set_ratio(QMCX983_SETRESET_FREQ_FAST);				//the ratio must not be 0, different with qmc5983
	usleep_range(20000,30000); //fixit for amr ready
	qmcX983_start_measure(client);

	return 0;
}

static int qmcX983_disable(struct i2c_client *client)
{
	MSE_LOG("stop measure!\n");
	qmcX983_stop_measure(client);

	return 0;
}

static atomic_t dev_open_count;

static int qmcX983_AxisInfoToPat(
	const uint8_t axis_order[3],
	const uint8_t axis_sign[3],
	int16_t *pat)
{
	/* check invalid input */
	if ((axis_order[0] < 0) || (axis_order[0] > 2) ||
	   (axis_order[1] < 0) || (axis_order[1] > 2) ||
	   (axis_order[2] < 0) || (axis_order[2] > 2) ||
	   (axis_sign[0] < 0) || (axis_sign[0] > 1) ||
	   (axis_sign[1] < 0) || (axis_sign[1] > 1) ||
	   (axis_sign[2] < 0) || (axis_sign[2] > 1) ||
	  ((axis_order[0] * axis_order[1] * axis_order[2]) != 0) ||
	  ((axis_order[0] + axis_order[1] + axis_order[2]) != 3)) {
		*pat = 0;
		return -1;
	}
	/* calculate pat
	 * BIT MAP
	 * [8] = sign_x
	 * [7] = sign_y
	 * [6] = sign_z
	 * [5:4] = order_x
	 * [3:2] = order_y
	 * [1:0] = order_z
	 */
	*pat = ((int16_t)axis_sign[0] << 8);
	*pat += ((int16_t)axis_sign[1] << 7);
	*pat += ((int16_t)axis_sign[2] << 6);
	*pat += ((int16_t)axis_order[0] << 4);
	*pat += ((int16_t)axis_order[1] << 2);
	*pat += ((int16_t)axis_order[2] << 0);
	return 0;
}

static int16_t qmcX983_SetCert(void)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *data = i2c_get_clientdata(client);
	uint8_t axis_sign[3] = {0};
	uint8_t axis_order[3] = {0};
	int16_t ret = 0;
	int i = 0;
	int16_t cert = 0x06;

	for (i = 0; i < 3; i++)
		axis_order[i] = (uint8_t)data->cvt.map[i];

	for (i = 0; i < 3; i++) {
		axis_sign[i] = (uint8_t)data->cvt.sign[i];
		if (axis_sign[i] > 0)
			axis_sign[i] = 0;
		else if (axis_sign[i] < 0)
			axis_sign[i] = 1;
	}
#if 0
	axis_order[0] = 0;
	axis_order[1] = 1;
	axis_order[2] = 2;
	axis_sign[0] = 0;
	axis_sign[1] = 0;
	axis_sign[2] = 0;
#endif
	ret = qmcX983_AxisInfoToPat(axis_order, axis_sign, &cert);
	if (ret != 0)
		return 0;
	return cert;
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];

	qmcX983_read_mag_xyz(sensordata);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *data = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw.direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct qmcX983_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;
	int ret = 0;

	ret = kstrtoint(buf, 10, &layout);
	if (ret == 0) {

		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw.direction, &data->cvt))
		{
			MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw.direction);
		}
		else
		{
			MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw.direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_OTP_value(struct device_driver *ddri, char *buf)
{	
	return sprintf(buf,"%d,%d\n",OTP_Kx,OTP_Ky);	
}

static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(otp, S_IRUGO, show_OTP_value, NULL);
static struct driver_attribute *qmcX983_attr_list[] = {
	&driver_attr_sensordata,
	&driver_attr_layout,
	&driver_attr_otp
};
static int qmcX983_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(qmcX983_attr_list);
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, qmcX983_attr_list[idx]);
		if(err < 0)
		{
			MSE_ERR("driver_create_file (%s) = %d\n", qmcX983_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
static int qmcX983_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)ARRAY_SIZE(qmcX983_attr_list);

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmcX983_attr_list[idx]);
	}

	return 0;
}

static int qmcX983_open_report_data(int en)
{
	return 0;
}
static int qmcX983_set_delay(u64 delay)
{
	int value = (int)delay/1000/1000;
	struct qmcX983_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(value <= 10)
		qmcd_delay = 10;

	qmcd_delay = value;

	return 0;
}
static int qmcX983_enable(int en)
{
	struct qmcX983_i2c_data *data = NULL;

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		/// we start measurement at here 
		qmcX983_init_hw(this_client);
	}
	else
	{
		qmcX983_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);

	MSE_ERR("qmcX983 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));
	return 0;
}


static int qmcX983_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int qmcX983_flush(void)
{
	return mag_flush_report();
}

static int qmcX983_get_data(int *x, int *y, int *z, int *status)
{
	struct qmcX983_i2c_data *data = NULL;
	int mag[3];

	if (unlikely(this_client == NULL)) {
		MSE_ERR("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) {
		MSE_ERR("data is null!\n");
		return -EINVAL;
	}
	qmcX983_read_mag_xyz(mag);
	*x = mag[0];//sensor_data[4] * CONVERT_M;
	*y = mag[1];//sensor_data[5] * CONVERT_M;
	*z = mag[2];//sensor_data[6] * CONVERT_M;
	*status = 3;//sensor_data[7];

#if DEBUG
	if(atomic_read(&data->trace) & QMC_HWM_DEBUG)
	{				
		MSE_LOG("Hwm get m-sensor data: %d, %d, %d,status %d!\n", *x, *y, *z, *status);
	}		
#endif

	return 0;
}

static int qmcX983_suspend(struct i2c_client *client, pm_message_t msg)
{
	return 0;
}

static int qmcX983_resume(struct i2c_client *client)
{
	return 0;
}

static int qmcX983_device_check(void){
	unsigned char databuf[2] = {0};
	int ret = 0; 
	
	databuf[0] = 0x0d;
	ret = I2C_RxData(databuf, 1);
	if(ret < 0){
		MSE_ERR("%s: I2C_RxData failed\n",__func__);
		return ret;
	}
	
	if(0xff == databuf[0]){
		chip_id = QMC6983_A1_D1;
	}else if(0x31 == databuf[0]){
		chip_id = QMC6983_E1;
	}else if(0x32 == databuf[0]){
		
		//read otp 0x30
		databuf[0] = 0x2e;
		databuf[1] = 0x01;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
		
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0){
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}		
		if(((databuf[0]&0x04 )>> 2))
		{
			chip_id = QMC6983_E1_Metal;
		}else{
			
			//read otp 0x3e
			databuf[0] = 0x2e;
			databuf[1] = 0x0f;
			ret = I2C_TxData(databuf,2);
			if(ret < 0)
			{
				MSE_ERR("%s: I2C_TxData failed\n",__func__);
				return ret;			
			}
			databuf[0] = 0x2f;
			ret = I2C_RxData(databuf, 1);
			if(ret < 0){
				MSE_ERR("%s: I2C_RxData failed\n",__func__);
				return ret;
			}
			if(0x02 == ((databuf[0]&0x3c)>>2)){
				chip_id = QMC7983_Vertical;
			}
			if(0x03 == ((databuf[0]&0x3c)>>2)){
				chip_id = QMC7983_Slope;
			}
		}
	}
	return ret;
}

static int qmcX983_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	struct qmcX983_i2c_data *data = NULL;
	int en = (enabledisable == true ? 1 : 0);
	int err;
    
	if (unlikely(this_client == NULL)) 
	{
		MSE_LOG("this_client is null!\n");
		return -EINVAL;
	}
		
	data = i2c_get_clientdata(this_client);
	if (unlikely(data == NULL)) 
	{
		MSE_LOG("data is null!\n");
		return -EINVAL;
	}

	if(en == 1)
	{
		v_open_flag |= 0x01;
		//we start measurement here.
		qmcX983_init_hw(this_client);
	}
	else
	{
		qmcX983_disable(this_client);
		v_open_flag &= 0x3e;
	}

	atomic_set(&open_flag, v_open_flag);
	
	MSE_LOG("qmcX983 v_open_flag = 0x%x,open_flag= 0x%x\n",v_open_flag, atomic_read(&open_flag));

	err = qmcX983_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		MSE_LOG("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int qmcX983_factory_get_data(int32_t data[3], int *status)
{
	/* get raw data */
	return qmcX983_get_data(&data[0], &data[1], &data[2], status);
}
static int qmcX983_factory_get_raw_data(int32_t data[3])
{
	MSE_LOG("do not support qmcX983_factory_get_raw_data!\n");
	return 0;
}
static int qmcX983_factory_enable_calibration(void)
{
	return 0;
}
static int qmcX983_factory_clear_cali(void)
{
	return 0;
}
static int qmcX983_factory_set_cali(int32_t data[3])
{
	return 0;
}
static int qmcX983_factory_get_cali(int32_t data[3])
{
	return 0;
}
static int qmcX983_factory_do_self_test(void)
{
	return 0;
}

/*----------------------------------------------------------------------------*/
static struct mag_factory_fops qmcX983_factory_fops = {
	.enable_sensor = qmcX983_factory_enable_sensor,
	.get_data = qmcX983_factory_get_data,
	.get_raw_data = qmcX983_factory_get_raw_data,
	.enable_calibration = qmcX983_factory_enable_calibration,
	.clear_cali = qmcX983_factory_clear_cali,
	.set_cali = qmcX983_factory_set_cali,
	.get_cali = qmcX983_factory_get_cali,
	.do_self_test = qmcX983_factory_do_self_test,
};
/*----------------------------------------------------------------------------*/

static struct mag_factory_public qmcX983_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &qmcX983_factory_fops,
};

static int qmcx983_get_OTP(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value[2] = {0};
	int ret = 0;

	if(chip_id == QMC6983_A1_D1)
	{
		OTP_Kx = 0;
		OTP_Ky = 0;
		return 0;
	}
	else
	{
		//read otp_kx
		databuf[0] = 0x2e;
		databuf[1] = 0x0a;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
    
		if(((value[0]&0x3f) >> 5) == 1)
		{
			OTP_Kx = (value[0]&0x1f)-32;
		}
		else
		{
			OTP_Kx = value[0]&0x1f;	
		}
		//read otp_ky
		databuf[0] = 0x2e;
		databuf[1] = 0x0d;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
		mdelay(10);
		databuf[0] = 0x2e;
		databuf[1] = 0x0f;
		ret = I2C_TxData(databuf,2);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        mdelay(10);
		databuf[0] = 0x2f;
		ret = I2C_RxData(databuf, 1);
		if(ret < 0)
		{
			MSE_ERR("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[1] = databuf[0];
		if((value[0] >> 7) == 1)
			OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
		else
			OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));	
	}
	return ret;
}


static int qmcX983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct qmcX983_i2c_data *data = NULL;

	int err = 0;
	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};
	
	MSE_FUN();

	data = kmalloc(sizeof(struct qmcX983_i2c_data), GFP_KERNEL);
	if(data == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}

	err = get_mag_dts_func(client->dev.of_node, &data->hw);
	if (err < 0) {
		MSE_ERR("%s. get dts info fail\n",__FUNCTION__);
		err = -EFAULT;
		goto exit_kfree;
	}

	err = hwmsen_get_convert(data->hw.direction, &data->cvt);	
	if (err) {
        	MSE_ERR("QMCX983 invalid direction: %d\n", data->hw.direction);
        	goto exit_kfree;
	}

	atomic_set(&data->layout, data->hw.direction);
	MSE_LOG("%s: direction: %d\n",__FUNCTION__,data->hw.direction);
	atomic_set(&data->trace, 0);

	mutex_init(&read_i2c_xyz);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;

	/* read chip id */
	err = qmcX983_device_check();
	if(err < 0)
	{
		MSE_LOG("%s check ID faild!\n",__FUNCTION__);
		err = -ENOTSUPP;
		goto exit_kfree;
	}
	err = qmcx983_get_OTP();
	if(err < 0)
	{
		MSE_LOG("%s get OTP faild!\n",__FUNCTION__);
		goto exit_kfree;
	}

	err = qmcX983_create_attr(&(qmcX983_init_info.platform_diver_addr->driver));
	if (err < 0)
	{
		MSE_ERR("create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	err = mag_factory_device_register(&qmcX983_factory_device);
	if (err) {
		MSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_factory_device_register_failed;
	}
	ctl.enable = qmcX983_enable;
	ctl.set_delay = qmcX983_set_delay;
	ctl.open_report_data = qmcX983_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = data->hw.is_batch_supported;
	ctl.batch = qmcX983_batch;
	ctl.flush = qmcX983_flush;
	ctl.libinfo.deviceid = chip_id;
	ctl.libinfo.layout = qmcX983_SetCert();
	memcpy(ctl.libinfo.libname, "calmodule_qmcX983", sizeof(ctl.libinfo.libname));

	err = mag_register_control_path(&ctl);
	if (err) {
		MAG_PR_ERR("register mag control path err\n");
		goto exit_hwm_attach_failed;
	}
	mag_data.div = CONVERT_M_DIV;
	mag_data.get_data = qmcX983_get_data;
	err = mag_register_data_path(&mag_data);
	if (err){
		MAG_PR_ERR("register data control path err\n");
		goto exit_hwm_attach_failed;
	}
	

    qmcX983_init_flag = 0;
	
	MSE_LOG("%s: OK\n", __func__);
	return 0;

exit_hwm_attach_failed:
	mag_factory_device_deregister(&qmcX983_factory_device);
exit_factory_device_register_failed:
	qmcX983_delete_attr(&(qmcX983_init_info.platform_diver_addr->driver));
exit_sysfs_create_group_failed:
exit_kfree:
	kfree(data);
exit:
	MSE_ERR("%s: err = %d\n", __func__, err);
	this_client = NULL;
	return err;
}

static int qmcX983_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, QMCX983_DEV_NAME);
    return 0;
}


static int qmcX983_i2c_remove(struct i2c_client *client)
{
	int err;

	err = qmcX983_delete_attr(&(qmcX983_init_info.platform_diver_addr->driver));
	if (err < 0)
	{
		MSE_ERR("qmcX983_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	mag_factory_device_deregister(&qmcX983_factory_device);
	return 0;
}
static int qmcX983_local_init(void)
{
	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&qmcX983_i2c_driver))
	{
		MSE_ERR("add driver error\n");
		return -EINVAL;
	}

    if(-1 == qmcX983_init_flag)
    {
        MSE_ERR("%s failed!\n",__func__);
        return -EINVAL;
    }

	return 0;
}


static int qmcX983_local_remove(void)
{

	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&qmcX983_i2c_driver);
	return 0;
}

static int __init qmcX983_init(void)
{
	mag_driver_add(&qmcX983_init_info);
	return 0;
}

static void __exit qmcX983_exit(void) {}
module_init(qmcX983_init);
module_exit(qmcX983_exit);

MODULE_AUTHOR("QST Corp");
MODULE_DESCRIPTION("qmcX983 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
