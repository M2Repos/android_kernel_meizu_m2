/* 
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include "focaltech_core.h"

#include "tpd.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

static int tpd_halt = 0;
//static int up_count = 0;
static int point_num = 0;

#define TPD_SUPPORT_POINTS	10

struct i2c_client *i2c_client = NULL;
struct task_struct *thread_tpd = NULL;

u8 *tpd_i2c_dma_va = NULL;
dma_addr_t tpd_i2c_dma_pa = 0;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;

#ifdef USB_CHARGE_DETECT
u8 b_usb_plugin = 0;
int ctp_is_probe = 0;
#endif

#if FTS_GESTRUE_EN
int touchscreen_suspended = 0;
u32 ft_gesture_wakeup_enable = 0;
static struct proc_dir_entry *ft5446_proc_entry = NULL;
#endif

unsigned int tpd_rst_gpio_number = 0;
static unsigned int tpd_int_gpio_number = 1;
static unsigned int touch_irq = 0;

struct touch_info {
	int y[TPD_SUPPORT_POINTS];
	int x[TPD_SUPPORT_POINTS];
	int p[TPD_SUPPORT_POINTS];
	int id[TPD_SUPPORT_POINTS];
	int count;
};

u8 *g_dma_buff_va = NULL;
dma_addr_t g_dma_buff_pa = 0;

static void msg_dma_alloct(void) {
	if (NULL == g_dma_buff_va) {
		tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		g_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 128, &g_dma_buff_pa, GFP_KERNEL);
	}

	if (!g_dma_buff_va)
		TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
}

static void msg_dma_release(void) {
	if (g_dma_buff_va) {
		dma_free_coherent(NULL, 128, g_dma_buff_va, g_dma_buff_pa);
		g_dma_buff_va = NULL;
		g_dma_buff_pa = 0;
		TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
}

static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);

static const struct i2c_device_id ft5446_tpd_id[] = {{"ft5446", 0}, {} };
static const struct of_device_id ft5446_dt_match[] = {{.compatible = "mediatek,ft5446"}};
MODULE_DEVICE_TABLE(of, ft5446_dt_match);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ft5446_dt_match),
		.name = "ft5446"
	},
	.probe = tpd_probe,
	.remove = tpd_remove,
	.id_table = ft5446_tpd_id,
	.detect = tpd_i2c_detect
};

static int of_get_ft5446_platform_data(struct device *dev) {
	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ft5446_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}

	TPD_DMESG("g_vproc_en_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("g_vproc_vsel_gpio_number %d\n", tpd_int_gpio_number);
	return 0;
}

#ifdef USB_CHARGE_DETECT
void tpd_usb_plugin(u8 plugin) {
	b_usb_plugin = plugin;

	if (!ctp_is_probe)
		return;
	printk("Fts usb detect: %s %d.\n",__func__, b_usb_plugin);

	if (i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &b_usb_plugin) < 0)
		printk("Fts usb detect write err: %s %d.\n",__func__,b_usb_plugin);
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif

static void tpd_down(int x, int y, int p, int id) {
	TPD_DEBUG("%s x:%d y:%d p:%d\n", __func__, x, y, p);
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
}

static void tpd_up(int x, int y) {
	TPD_DEBUG("%s x:%d y:%d\n", __func__, x, y);
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo) {
	int i = 0;
	char data[128] = {0};
	u16 high_byte, low_byte;
	char reg;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	reg = 0;
	fts_i2c_read(i2c_client, &reg, 1, data, 64);
	
	/*get the number of the touch points*/
	point_num = data[2] & 0x0f;
	
	for(i = 0; i < point_num; i++)  
	{
		cinfo->p[i] = data[6 * i + 3] >> 6;
		cinfo->id[i] = data[6 * i + 5] >> 4;
		high_byte = data[6 * i + 3];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[6 * i + 4];
		cinfo->x[i] = high_byte | low_byte;
		high_byte = data[6 * i + 5];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[6 * i + 6];
		cinfo->y[i] = high_byte | low_byte;
	}

	printk("MIKE: tpd cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);
	return true;
};


int i2c_master_send_fts(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;
	msg.timing = 400 ; //client->timing;
	msg.ext_flag = client->ext_flag;

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 1) ? count : ret;
}



int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=0;

	// for DMA I2c transfer

	mutex_lock(&i2c_access);

	if((NULL!=client) && (writelen>0) && (writelen<=128))
	{
		// DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		if((ret=i2c_master_send_fts(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
			//dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,*g_dma_buff_pa);
			printk("i2c write failed\n");
		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}

	// DMA Read

	if((NULL!=client) && (readlen>0) && (readlen<=128))

	{
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;

		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);

		memcpy(readbuf, g_dma_buff_va, readlen);

		client->addr = (client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}

	mutex_unlock(&i2c_access);

	return ret;
}

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int i = 0;
	int ret = 0;

	if (writelen <= 8) {
		client->ext_flag = client->ext_flag & (~I2C_DMA_FLAG);
		return i2c_master_send_fts(client, writebuf, writelen);
	}
	else if((writelen > 8)&&(NULL != tpd_i2c_dma_va))
	{
		for (i = 0; i < writelen; i++)
			tpd_i2c_dma_va[i] = writebuf[i];

		client->addr = (client->addr & I2C_MASK_FLAG )| I2C_DMA_FLAG;
		ret = i2c_master_send_fts(client, (unsigned char *)tpd_i2c_dma_pa, writelen);
		client->addr = client->addr & I2C_MASK_FLAG & ~I2C_DMA_FLAG;
		return ret;
	}
	return 1;
}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

#if FTS_GESTRUE_EN
void fts_read_gestruedata(void) {
	unsigned char buf[1028] = {0};
	int ret = -1;
	int gesture_id = 0;
	short pointnum = 0;
	buf[0] = 0xd3;

	ret = fts_i2c_read(i2c_client, buf, 1, buf, 8);
	if (ret < 0)
	{
		printk( "%s [FTS]read touchdata failed.\n", __func__);
		return;
	}

	if (buf[0] != 0xfe)
		gesture_id = buf[0];

	pointnum = (short)(buf[1]) & 0xff;
	printk("[FTS] the pointnum = %d\n",pointnum);
	buf[0] = 0xd3;
	if ((pointnum * 4 + 38) < 255)
		ret = fts_i2c_read(i2c_client, buf, 1, buf, (pointnum * 4 + 38));
	else {
		ret = fts_i2c_read(i2c_client, buf, 1, buf, 255);
		ret = fts_i2c_read(i2c_client, buf, 0, buf + 255, (pointnum * 4 + 38) - 255);
	}

	if (ret < 0) {
		printk( "%s [FTS]read touchdata failed.\n", __func__);
		return;
	}

	if (gesture_id == 0x24 /* DT2W */ && touchscreen_suspended) {
		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
	}
}
#endif

static int touch_event_handler(void *unused)
{
	int i = 0;
	#if FTS_GESTRUE_EN
	int ret = 0;
	u8 state = 0;
	#endif

	struct touch_info cinfo, pinfo;
	struct sched_param param = { .sched_priority = 4 };

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);

		tpd_flag = 0;

		set_current_state(TASK_RUNNING);

#if FTS_GESTRUE_EN
		if (ft_gesture_wakeup_enable) {
			ret = fts_read_reg(i2c_client, 0xd0, &state);
			if (ret < 0)
				printk("[FTS][Touch] read value fail");
			printk("[FTS]tpd fts_read_gestruedata state=%d\n",state);
			if (state == 1) {
				fts_read_gestruedata();
				continue;
			}
		}
#endif

		TPD_DEBUG("touch_event_handler start\n");

		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if (point_num > 0) {
				for (i = 0; i < point_num; i++)
					tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
			} else {
				tpd_up(cinfo.x[0], cinfo.y[0]);
			}
			input_sync(tpd->dev);

		}	
	} while (!kthread_should_stop());

	TPD_DEBUG("touch_event_handler exit\n");

	return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));

		TPD_DMESG("debounce:ints[0]=%d,ints[1]=%d", ints[0], ints[1]);

		gpio_set_debounce(ints[0], ints[1]);
		touch_irq = irq_of_parse_and_map(node, 0);

		TPD_DMESG("touch_irq=%d", touch_irq);

		ret = request_irq(touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		if (ret > 0)
			TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = 0;
	int reset_count = 0;
	char data;
	int err = 0;
	u8 ver;

	i2c_client = client;
	printk("wwm:i2c_client->addr=0x%02x\n", i2c_client->addr);
	if (i2c_client->addr != 0x38)
		i2c_client->addr = 0x38;

	of_get_ft5446_platform_data(&client->dev);

	TPD_DMESG("mtk_tpd: tpd_probe ft5446\n");

	retval = regulator_enable(tpd->reg);

	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);

	tpd_gpio_as_int(tpd_int_gpio_number);

	msleep(100);	
	msg_dma_alloct();

#if FTS_GESTRUE_EN
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
#endif


reset_proc:
	/* Reset CTP */
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);
	err = fts_read_reg(i2c_client, 0x00, &data);

	TPD_DMESG("fts_i2c:err %d,data:%d\n", err,data);
	if (err < 0 || data != 0) {
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
		if (++reset_count < 3)
			goto reset_proc;

		retval	= regulator_disable(tpd->reg);
		if (retval)
			printk("focaltech tpd_probe regulator_disable() failed!\n");

		regulator_put(tpd->reg);

		msg_dma_release();
		gpio_free(tpd_rst_gpio_number);
		gpio_free(tpd_int_gpio_number);
		return -1;
	}
	tpd_load_status = 1;
	tpd_irq_registration();

	thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread_tpd)) {
		retval = PTR_ERR(thread_tpd);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread_tpd: %d\n", retval);
	}

	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < 0) ? "FAIL" : "PASS");

	fts_read_reg(client, 0xA6, &ver);
	TPD_DMESG(TPD_DEVICE " fts_read_reg version : %d\n", ver);
	tpd->dev->id.version = ver;

#ifdef USB_CHARGE_DETECT
	if (ctp_is_probe == 0) {
		 i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &b_usb_plugin);
		ctp_is_probe =1;
	}
#endif


	return 0;
}

#if FTS_GESTRUE_EN
static ssize_t ft5446_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
    int value;
	char data_buf[50] = {0};

	if ((NULL == filp) || (NULL == buff))
		return -1;

	if ((len <= 0) || (len > 50))
		return -1;

	if (copy_from_user(data_buf, buff, len))
		return -EFAULT;

	value = simple_strtoul(data_buf, NULL, 16);
	if (value != 0 && value != 1) {
		printk("ft5446_proc_write, invalid value.\n");
		return -1;
	} else
		ft_gesture_wakeup_enable = value;

	printk("ft5446_proc_write has been writed to %d.\n", value);\

	return len;
}

static int ft5446_proc_show(struct seq_file *m, void *v)
{
	printk("ft5446_proc_show: ft_gesture_wakeup_enable=%d\n", ft_gesture_wakeup_enable);
	seq_printf(m, "%d\n", ft_gesture_wakeup_enable);
	return 0;
}

static int ft5446_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5446_proc_show, NULL);
}

static const struct file_operations ft5446_proc_fops = {
	.open = ft5446_proc_open,
	.write = ft5446_proc_write,
	.read = seq_read
};
#endif

static int tpd_remove(struct i2c_client *client)
{
	TPD_DEBUG("TPD removed\n");
		msg_dma_release();

	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);
	return 0;
}

static int tpd_local_init(void)
{
	int retval;

	TPD_DMESG("Focaltech ft5446 I2C Touchscreen Driver...\n");

	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (CFG_MAX_TOUCH_POINTS - 1), 0, 0);

	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

#if FTS_GESTRUE_EN
	ft5446_proc_entry = proc_create("enable_dt2w", 0666, NULL, &ft5446_proc_fops);
	if (ft5446_proc_entry == NULL)
		printk("create_proc_entry enable_dt2w failed!\n");
#endif

	return 0;
}

static void tpd_resume(struct device *h)
{
	int retval = 0;

	TPD_DEBUG("TPD wake up\n");
	printk("TPD wake up\n");

	#if FTS_GESTRUE_EN
	if (ft_gesture_wakeup_enable) {
		fts_write_reg(i2c_client,0xd0,0x00);
		fts_write_reg(i2c_client,0xd1,0x00);
		fts_write_reg(i2c_client,0xd2,0x00);
	}
	#endif

	regulator_disable(tpd->reg);
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(5);

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	msleep(5);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(400);


#if FTS_GESTRUE_EN
	touchscreen_suspended = 0;
#endif

	tpd_halt = 0;
	enable_irq(touch_irq);

#ifdef USB_CHARGE_DETECT
	msleep(120);
	tpd_usb_plugin(b_usb_plugin);
#endif

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	input_sync(tpd->dev);
}

static void tpd_suspend(struct device *h)
{
	int retval = 0;

	static char data = 0x3;

	#if FTS_GESTRUE_EN
	int i = 0;
	u8 state = 0;
	#endif
	
	printk("[FTS]TPD enter sleep\n");
	tpd_halt = 1;
	
	#if FTS_GESTRUE_EN
	if (ft_gesture_wakeup_enable) {
			fts_write_reg(i2c_client, 0xd0, 0x01);
		  	fts_write_reg(i2c_client, 0xd1, 0xff);
			fts_write_reg(i2c_client, 0xd2, 0xff);
			fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			fts_write_reg(i2c_client, 0xd7, 0xff);
			fts_write_reg(i2c_client, 0xd8, 0xff);
			msleep(10);

			for (i = 0; i < 10; i++) {
			  	fts_read_reg(i2c_client, 0xd0, &state);
				if (state == 1) {
					touchscreen_suspended = 1;
					return;
				} else {
					fts_write_reg(i2c_client, 0xd0, 0x01);
					fts_write_reg(i2c_client, 0xd1, 0xff);
		 			fts_write_reg(i2c_client, 0xd2, 0xff);
					fts_write_reg(i2c_client, 0xd5, 0xff);
					fts_write_reg(i2c_client, 0xd6, 0xff);
					fts_write_reg(i2c_client, 0xd7, 0xff);
				  	fts_write_reg(i2c_client, 0xd8, 0xff);
					msleep(10);
				}
			}

		if (i >= 9) {
			printk("[FTS]TPD gesture write 0x01 to d0 fail\n");
			return;
		}
	}
	#endif

	disable_irq(touch_irq);
	fts_write_reg(i2c_client, 0xA5, data);  /* TP enter sleep mode */
	retval = regulator_disable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to disable reg-vgp6: %d\n", retval);
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "ft5446",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume
};

static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek ft5446 touch driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("MediaTek ft5446 touch driver init failed\n");

	return 0;
}

static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek ft5446 touch driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
	#if FTS_GESTRUE_EN
	if (ft5446_proc_entry != NULL) {
		remove_proc_entry("enable_dt2w", NULL);
		ft5446_proc_entry = NULL;
	}
	#endif
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
