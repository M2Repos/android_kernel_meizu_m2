/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include "kd_flashlight.h"
#include "kd_camera_typedef.h"

int led_vendor_id = 0;

#define TAG_NAME "leds_sky81294.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_DBG(a, ...)

static struct i2c_client *SKY81294_i2c_client = NULL;

struct SKY81294_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

struct SKY81294_chip_data {
	struct i2c_client *client;
	struct SKY81294_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static int SKY81294_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x\n", reg);
	return ret;
}

static int SKY81294_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

static int SKY81294_chip_init(struct SKY81294_chip_data *chip)
{
	s32 regVal6 = 0;
	regVal6 = SKY81294_read_reg(SKY81294_i2c_client, 0x06);
	if (regVal6 < 0) {
		PK_DBG("SKY81294 i2c bus read error\n");
		return -1;
	} else {
		led_vendor_id = 0x8129;
	}

	return 0;
}

int is_ic_sky81294(void)
{
	return led_vendor_id;
}

void SKY81294_torch_set_level(int level)
{
	char buf[2];
	PK_DBG("SKY81294 %s,%d,%d,level",__func__,__LINE__,level);
	buf[0] = 0x02;
	buf[1] = level;  
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
	buf[0] = 0x03;
	buf[1] = 0x01;
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);	  
}

void SKY81294_torch_mode(int g_duty)
{
	char buf[2];
	PK_DBG("SKY81294 %s,%d,%d,torch", __func__, __LINE__, g_duty);
	buf[0] = 0x02;
	buf[1] = 0x02;    
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
	buf[0] = 0x03;
	buf[1] = 0x01;
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}

void SKY81294_flash_mode(int g_duty)
{
	char buf[2];
	PK_DBG("SKY81294 %s,%d,%d,flash", __func__, __LINE__, g_duty);
	buf[0] = 0x00;
	buf[1] = g_duty;
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
	buf[0] = 0x03;
	buf[1] = 0x02;
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);   
}

void SKY81294_shutdown_mode(void)
{
	char buf[2];
	buf[0] = 0x03;
	buf[1] = 0x00;
	SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}

static int SKY81294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct SKY81294_chip_data *chip;
	struct SKY81294_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("SKY81294_probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR "SKY81294 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct SKY81294_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL) {
		PK_DBG("SKY81294 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct SKY81294_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	SKY81294_i2c_client = client;
	chip->pdata  = pdata;
	if(SKY81294_chip_init(chip) < 0)
		goto err_chip_init;

	PK_DBG("SKY81294 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("SKY81294 probe is failed \n");
	return -ENODEV;
}

static int SKY81294_remove(struct i2c_client *client)
{
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	if(chip->no_pdata)
		kfree(chip->pdata);

	kfree(chip);
	return 0;
}

#define SKY81294_NAME "leds-SKY81294"
static const struct i2c_device_id SKY81294_id[] = {{SKY81294_NAME, 0}};

static const struct of_device_id SKY81294_of_match[] = {
	{.compatible = "mediatek,strobe_main_2"},
};

static struct i2c_driver SKY81294_i2c_driver = {
	.driver = {
		   .name = SKY81294_NAME,
		   .of_match_table = SKY81294_of_match,
		   },
	.probe = SKY81294_probe,
	.remove = SKY81294_remove,
	.id_table = SKY81294_id,
};

static int __init SKY81294_init(void)
{
	printk("SKY81294_init\n");
	return i2c_add_driver(&SKY81294_i2c_driver);
}

static void __exit SKY81294_exit(void)
{
	i2c_del_driver(&SKY81294_i2c_driver);
}

module_init(SKY81294_init);
module_exit(SKY81294_exit);

MODULE_DESCRIPTION("Flash driver for SKY81294");
MODULE_AUTHOR("<liukun@wind-mobi.com>");
MODULE_LICENSE("GPL v2");
