/* 
 * Author: lijunjiea <lijunjiea@awinic.com>
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

#include <linux/i2c.h>
#include <linux/stat.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>

#ifndef KEY_FINGERPRINT
#define KEY_FINGERPRINT 196
#endif

#define AW9201_NAME "AW9201_ts"

#define CALI_NUM	2
#define CALI_RAW_MIN	250
#define CALI_RAW_MAX	1750

#define AW_INFO(fmt, arg...)	pr_info("AW9201_ts: "fmt"\n", ##arg)
#define AW_ERROR(fmt, arg...)	pr_err("AW9201_ts: "fmt"\n", ##arg)

static struct pinctrl *pinctrl1;
static struct pinctrl_state *pins_default;
static struct pinctrl_state *eint_as_int;

unsigned char cali_flag = 0;
unsigned char cali_num = 0;
unsigned char cali_cnt = 0;
unsigned char cali_used = 0;
unsigned char old_cali_dir;
unsigned char old_ofr_cfg;

static int enabled = 1;
unsigned int rawdata_sum;

struct aw9201_i2c_setup_data {
	unsigned i2c_bus;
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

struct aw9201_ts_data {
	struct input_dev *input_dev;
	struct work_struct eint_work;
};

struct aw9201_ts_data *aw9201_ts;

static ssize_t aw9201_get_irqstate(struct device* cd, struct device_attribute *attr, char* buf);
static ssize_t aw9201_get_rawdata(struct device* cd, struct device_attribute *attr, char* buf);
static ssize_t aw9201_get_status(struct device* cd, struct device_attribute *attr, char* buf);
static ssize_t aw9201_set_status(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);

static DEVICE_ATTR(enable, S_IWUSR | S_IWGRP | S_IRUGO, aw9201_get_status, aw9201_set_status);
static DEVICE_ATTR(getstate, S_IWUSR | S_IWGRP | S_IRUGO, aw9201_get_irqstate, NULL);
static DEVICE_ATTR(rawdata, S_IWUSR | S_IWGRP | S_IRUGO, aw9201_get_rawdata, NULL);

static struct attribute *aw9201_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_getstate.attr,
	&dev_attr_rawdata.attr,
	NULL
};

static struct attribute_group aw9201_attribute_group = {.attrs = aw9201_attributes};

static unsigned char suspend_flag;
static int WorkMode = 1;

static struct i2c_client *this_client;
static struct aw9201_i2c_setup_data aw9201_ts_setup = {0, 0, 0, AW9201_NAME};

static const struct of_device_id aw9201_ts_match[] = {{.compatible = "mediatek,aw9201_ts"}};

static unsigned int i2c_write_reg(unsigned char addr, unsigned char reg_data)
{
	int ret;
	u8 wdbuf[512] = {addr, reg_data, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if (ret < 0)
		AW_ERROR("%s i2c read error ret=%d", __func__, ret);

	return ret;
}

static unsigned char i2c_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		AW_ERROR("%s i2c read error ret=%d", __func__, ret);

	return rdbuf[0];
}

unsigned char aw_autoCalibration(void)
{
	unsigned char cali_dir;
	unsigned char ofr_cfg;
	unsigned char rawdata[2];

	if (cali_num == 0)
		ofr_cfg = i2c_read_reg(0x07);
	else
		ofr_cfg = old_ofr_cfg;

	rawdata[0] = i2c_read_reg(0x20);
	rawdata[1] = i2c_read_reg(0x21);

	rawdata_sum = (cali_cnt == 0) ? (0) : (rawdata_sum + ((rawdata[0] << 8) | rawdata[1]));

	if (cali_cnt == 4) {
		if ((rawdata_sum >> 2) < CALI_RAW_MIN) {
			if ((ofr_cfg & 0x1F) == 0x00) {
				cali_dir = 0;
			} else {
				cali_dir = 2;
				ofr_cfg = ofr_cfg - 1;
			}
		} else if ((rawdata_sum >> 2) > CALI_RAW_MAX) {
			if ((ofr_cfg & 0x1F) == 0x1F) {
				cali_dir = 0;
			} else {
				cali_dir = 1;
				ofr_cfg = ofr_cfg + 1;
			}
		} else {
			cali_dir = 0;
		}

		if (cali_num > 0) {
			if (cali_dir != old_cali_dir) {
				cali_dir = 0;
				ofr_cfg = old_ofr_cfg;
			}
		}

		cali_flag = 0;
		if (cali_dir != 0)
			cali_flag = 1;

		if ((cali_flag == 0) && (cali_num == 0))
			cali_used = 0;
		else
			cali_used = 1;

		if (cali_flag == 0) {
			cali_num = 0;
			cali_cnt = 0;
			return 0;
		}

		i2c_write_reg(0x07, ofr_cfg);
		i2c_write_reg(0x01, 0x00);
		i2c_write_reg(0x01, 0x02);

		if (cali_num == (CALI_NUM - 1)) {
			cali_flag = 0;
			cali_num = 0;
			cali_cnt = 0;

			return 0;
		}

		old_cali_dir = cali_dir;
		old_ofr_cfg = ofr_cfg;

		cali_num++;
	}

	if (cali_cnt < 4)
		cali_cnt++;
	else
		cali_cnt = 0;

	return 1;
}

static void aw_normalMode(void)
{
	i2c_write_reg(0x00, 0x55);
	i2c_write_reg(0x01, 0x00);
	i2c_write_reg(0x04, 0x05);
	i2c_write_reg(0x05, 0x04);
	i2c_write_reg(0x06, 0x31);
	i2c_write_reg(0x07, 0x12);
	i2c_write_reg(0x08, 0x40);
	i2c_write_reg(0x0B, 0x08);
	i2c_write_reg(0x0D, 0x10);
	i2c_write_reg(0x0E, 0x01);
	i2c_write_reg(0x0F, 0x08);
	i2c_write_reg(0x2B, 0x4D);
	i2c_write_reg(0x01, 0x02);
	WorkMode = 2;
}

static void aw_sleepMode(void)
{
	i2c_write_reg(0x00, 0x55);
	i2c_write_reg(0x01, 0x00);
	WorkMode = 1;
}

static void aw_centerPress(void)
{
	if (enabled) {
		input_report_key(aw9201_ts->input_dev, KEY_FINGERPRINT, 1);
		input_sync(aw9201_ts->input_dev);
	}
}

static void aw_centerRelease(void)
{
	if (enabled) {
		input_report_key(aw9201_ts->input_dev, KEY_FINGERPRINT, 0);
		input_sync(aw9201_ts->input_dev);
	}
}

static void aw_sleepMode_proc(void)
{
	unsigned char buf;

	buf = i2c_read_reg(0x02);
	if (buf & 0x08) {
		if (buf & 0x02)
			aw_centerPress();
		else
			aw_centerRelease();
	}
}

static void aw_normalMode_proc(void)
{
	unsigned char buf;
	if (cali_flag) {
		aw_autoCalibration();
		if (cali_flag == 0) {
			i2c_write_reg(0x0D, 0x00);
			i2c_write_reg(0x01, 0x00);
			i2c_write_reg(0x01, 0x06);
		}
		return;
	}

	buf = i2c_read_reg(0x02);
	AW_INFO("Status 0x02 = %x", buf);
	if (buf & 0x08) {
		if (buf & 0x02)
			aw_centerPress();
		else
			aw_centerRelease();
	}
}

static int aw9201_ts_clear_intr(struct i2c_client *client)
{
	int res;

	i2c_read_reg(0x2D);
	res = i2c_read_reg(0x02);
	if (res < 0)
		return 1;
	else
		res = 0;

	return res;
}

static void aw9201_ts_eint_work(struct work_struct *work)
{
	switch(WorkMode) {
	case 1:
		aw_sleepMode_proc();
		break;
	case 2:
		aw_normalMode_proc();
		break;
	default:
		break;
	}

	aw9201_ts_clear_intr(this_client);
	enable_irq(aw9201_ts_setup.irq);
}

static irqreturn_t aw9201_ts_eint_func(void)
{
	if (aw9201_ts == NULL)
		return IRQ_HANDLED;

	disable_irq_nosync(aw9201_ts_setup.irq);
	schedule_work(&aw9201_ts->eint_work);
	return IRQ_HANDLED;
}

int aw9201_ts_setup_eint(void)
{
	int ret = 0;
	pinctrl_select_state(pinctrl1, eint_as_int);
	ret = request_irq(aw9201_ts_setup.irq,
				(irq_handler_t)aw9201_ts_eint_func,
				IRQF_TRIGGER_LOW,
				"aw9201-eint", NULL);
	if (ret > 0) {
		ret = -1;
	}

	return ret;
}

static int aw9201_ts_suspend(struct device *dev)
{
	if (WorkMode != 1) {
		aw_sleepMode();
		suspend_flag = 1;
	}

	disable_irq(aw9201_ts_setup.irq);
	return 0;
}

static int aw9201_ts_resume(struct device *dev)
{
	if (WorkMode != 2) {
		aw_normalMode();
		suspend_flag = 0;
		cali_flag = 1;
		cali_num = 0;
		cali_cnt = 0;
	}

	enable_irq(aw9201_ts_setup.irq);
	return 0;
}

static struct notifier_block aw9201_ts_fb_notifier;

static int aw9201_ts_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = NULL;
	int blank;
	int err = 0;

	evdata = data;
	if (event != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)evdata->data;
	AW_INFO("fb_notify(blank=%d)", blank);
	switch (blank) {
	case FB_BLANK_UNBLANK:
		AW_INFO("Screen on notify");
		err = aw9201_ts_resume(&this_client->dev);
		if (err)
			AW_ERROR("Failed to resume device");
		break;
	case FB_BLANK_POWERDOWN:
		AW_INFO("Screen off notify");
		err = aw9201_ts_suspend(&this_client->dev);
		if (err)
			AW_ERROR("Failed to suspend device");
		break;
	default:
		break;
	}
	return 0;
}

int aw9201_get_gpio_info(struct device *dev)
{
	int ret;

	AW_INFO("%s", __func__);
	pinctrl1 = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		AW_ERROR("Cannot find pinctrl1 ret=%d", ret);
		return ret;
	}

	pins_default = pinctrl_lookup_state(pinctrl1, "mback_eint_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		AW_ERROR("Cannot find eint_default pinctrl ret=%d", ret);
	}

	eint_as_int = pinctrl_lookup_state(pinctrl1, "mback_eint_as_int");
	if (IS_ERR(eint_as_int)) {
		ret = PTR_ERR(eint_as_int);
		AW_ERROR("Cannot find eint_as_int pinctrl ret=%d", ret);
		return ret;
	}

	return 0;
}

void aw9201_parse_dts_info(void)
{
	struct device_node *node = NULL;

	node = of_find_matching_node(node, aw9201_ts_match);
	if (node) {
		aw9201_ts_setup.irq = irq_of_parse_and_map(node, 0);
		AW_INFO("%s irq=%d", __func__, aw9201_ts_setup.irq);
	} else
		AW_ERROR("%s can't find compatible device tree node", __func__);
}

static int aw9201_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	int err = 0;
	unsigned char reg_value, reg_value1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw9201_ts = kzalloc(sizeof(*aw9201_ts), GFP_KERNEL);
	if (!aw9201_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	aw9201_ts_fb_notifier.notifier_call = aw9201_ts_fb_notifier_callback;
	if (fb_register_client(&aw9201_ts_fb_notifier))
		AW_ERROR("Register fb_notifier failed!");

	aw9201_parse_dts_info();
	aw9201_get_gpio_info(&client->dev);
	msleep(10);

	client->timing = 400;
	this_client = client;
	i2c_set_clientdata(client, aw9201_ts);

	AW_INFO("I2C addr=%x", client->addr);

	reg_value = i2c_read_reg(0x00);
	AW_INFO("Chip ID=0x%4x", reg_value);

	if (reg_value != 0x33) {
		err = -ENODEV;
		goto exit_create_singlethread;
	}

	INIT_WORK(&aw9201_ts->eint_work, aw9201_ts_eint_work);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		AW_ERROR("Failed to allocate input device");
		goto exit_input_dev_alloc_failed;
	}

	aw9201_ts->input_dev = input_dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);

	__set_bit(KEY_FINGERPRINT, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);

	input_dev->name = AW9201_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	suspend_flag = 0;

	msleep(50);

	if (sysfs_create_group(&input_dev->dev.kobj, &aw9201_attribute_group))
		AW_ERROR("Failed to create sysfs nodes");

	WorkMode = 2;
	aw_normalMode();
	cali_flag = 1;
	cali_num = 0;
	cali_cnt = 0;
	reg_value1 = i2c_read_reg(0x01);
	AW_INFO("GCR = 0x%2x", reg_value1);

	aw9201_ts_setup_eint();
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	cancel_work_sync(&aw9201_ts->eint_work);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(aw9201_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	AW_INFO("%s ret=%d", __func__, err);
	return err;
}

static int aw9201_ts_remove(struct i2c_client *client)
{
	struct aw9201_ts_data *aw9201_ts = i2c_get_clientdata(client);
	input_unregister_device(aw9201_ts->input_dev);
	kfree(aw9201_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id aw9201_ts_id[] = {
	{AW9201_NAME, 0},{}
};

static struct i2c_driver aw9201_ts_driver = {
	.probe		= aw9201_ts_probe,
	.remove		= aw9201_ts_remove,
	.id_table	= aw9201_ts_id,
	.driver	= {
		.name	= AW9201_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = aw9201_ts_match,
	},
};

static ssize_t aw9201_get_status(struct device* cd, struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	sprintf(buf, "%d\n", enabled);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t aw9201_set_status(struct device *cd, struct device_attribute *attr,
                                   const char *buf, size_t len)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
    if (value != 0 && value != 1)
		return -1;
	else
		enabled = value;

	AW_INFO("%s: enabled=%d", __func__, enabled);

	return len;
}

static ssize_t aw9201_get_rawdata(struct device* cd, struct device_attribute *attr, char* buf)
{
	unsigned int rawdata;
	ssize_t len = 0;
	disable_irq(aw9201_ts_setup.irq);

	len += snprintf(buf + len, PAGE_SIZE - len, "rawdata:\n");

	buf[0] = i2c_read_reg(0x20);
	buf[1] = i2c_read_reg(0x21);
	rawdata = (unsigned int)((buf[0] << 8) | buf[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", rawdata);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	enable_irq(aw9201_ts_setup.irq);
	return len;
}

static ssize_t aw9201_get_irqstate(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char touch,key;

	ssize_t len = 0;
	disable_irq(aw9201_ts_setup.irq);
	len += snprintf(buf + len, PAGE_SIZE - len, "touch:\n");

	touch = i2c_read_reg(0x02);
	if ((touch & 0x2) == 0x2)
		key = 1;
	else
		key = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d, ", key);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	enable_irq(aw9201_ts_setup.irq);
	return len;
}

static int __init aw9201_ts_init(void)
{
	int ret = i2c_add_driver(&aw9201_ts_driver);
	AW_INFO("%s ret=%d", __func__, ret);
	return ret;
}

static void __exit aw9201_ts_exit(void)
{
	i2c_del_driver(&aw9201_ts_driver);
}

module_init(aw9201_ts_init);
module_exit(aw9201_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AW9201 Touchkey Driver");
MODULE_AUTHOR("<lijunjiea@awinic.com>");
