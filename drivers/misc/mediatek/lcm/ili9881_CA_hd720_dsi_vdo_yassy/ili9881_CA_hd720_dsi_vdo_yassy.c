#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/string.h>
#include "lcm_drv.h"

#define LCM_ID_ILI9881	0x9881

#define FRAME_HEIGHT	(1280)
#define FRAME_WIDTH	(720)

#define REGFLAG_DELAY	0xFC
#define REGFLAG_END_OF_TABLE	0xFD

#define GPIO_LCM_PWR_EN	(GPIO119 | 0x80000000)

#define MDELAY(n)	(lcm_util.mdelay(n))
#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static LCM_UTIL_FUNCS lcm_util;

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF, 3, {0x98, 0x81, 0x3}},
	{0x1, 1, {0x8}},
	{0x2, 1, {0x0}},
	{0x3, 1, {0x73}},
	{0x4, 1, {0x73}},
	{0x5, 1, {0x14}},
	{0x6, 1, {0x6}},
	{0x7, 1, {0x2}},
	{0x8, 1, {0x5}},
	{0x9, 1, {0x0}},
	{0xA, 1, {0xC}},
	{0xB, 1, {0x0}},
	{0xC, 1, {0x1C}},
	{0xD, 1, {0x1C}},
	{0xE, 1, {0x0}},
	{0xF, 1, {0xC}},
	{0x10, 1, {0xC}},
	{0x11, 1, {0x1}},
	{0x12, 1, {0x1}},
	{0x13, 1, {0x1B}},
	{0x14, 1, {0xB}},
	{0x15, 1, {0x0}},
	{0x16, 1, {0x0}},
	{0x17, 1, {0x0}},
	{0x18, 1, {0x0}},
	{0x19, 1, {0x0}},
	{0x1A, 1, {0x0}},
	{0x1B, 1, {0x0}},
	{0x1C, 1, {0x0}},
	{0x1D, 1, {0x0}},
	{0x1E, 1, {0xC8}},
	{0x1F, 1, {0x80}},
	{0x20, 1, {0x2}},
	{0x21, 1, {0x0}},
	{0x22, 1, {0x2}},
	{0x23, 1, {0x0}},
	{0x24, 1, {0x0}},
	{0x25, 1, {0x0}},
	{0x26, 1, {0x0}},
	{0x27, 1, {0x0}},
	{0x28, 1, {0xFB}},
	{0x29, 1, {0x43}},
	{0x2A, 1, {0x0}},
	{0x2B, 1, {0x0}},
	{0x2C, 1, {0x7}},
	{0x2D, 1, {0x7}},
	{0x2E, 1, {0xFF}},
	{0x2F, 1, {0xFF}},
	{0x30, 1, {0x11}},
	{0x31, 1, {0x0}},
	{0x32, 1, {0x0}},
	{0x33, 1, {0x0}},
	{0x34, 1, {0x84}},
	{0x35, 1, {0x80}},
	{0x36, 1, {0x7}},
	{0x37, 1, {0x0}},
	{0x38, 1, {0x0}},
	{0x39, 1, {0x0}},
	{0x3A, 1, {0x0}},
	{0x3B, 1, {0x0}},
	{0x3C, 1, {0x0}},
	{0x3D, 1, {0x0}},
	{0x3E, 1, {0x0}},
	{0x3F, 1, {0x0}},
	{0x40, 1, {0x0}},
	{0x41, 1, {0x0}},
	{0x42, 1, {0x0}},
	{0x43, 1, {0x80}},
	{0x44, 1, {0x8}},
	{0x50, 1, {0x1}},
	{0x51, 1, {0x23}},
	{0x52, 1, {0x45}},
	{0x53, 1, {0x67}},
	{0x54, 1, {0x89}},
	{0x55, 1, {0xAB}},
	{0x56, 1, {0x1}},
	{0x57, 1, {0x23}},
	{0x58, 1, {0x45}},
	{0x59, 1, {0x67}},
	{0x5A, 1, {0x89}},
	{0x5B, 1, {0xAB}},
	{0x5C, 1, {0xCD}},
	{0x5D, 1, {0xEF}},
	{0x5E, 1, {0x10}},
	{0x5F, 1, {0x2}},
	{0x60, 1, {0x2}},
	{0x61, 1, {0x2}},
	{0x62, 1, {0x2}},
	{0x63, 1, {0x2}},
	{0x64, 1, {0x2}},
	{0x65, 1, {0x2}},
	{0x66, 1, {0x8}},
	{0x67, 1, {0x9}},
	{0x68, 1, {0x2}},
	{0x69, 1, {0x10}},
	{0x6A, 1, {0x12}},
	{0x6B, 1, {0x11}},
	{0x6C, 1, {0x13}},
	{0x6D, 1, {0xC}},
	{0x6E, 1, {0xE}},
	{0x6F, 1, {0xD}},
	{0x70, 1, {0xF}},
	{0x71, 1, {0x6}},
	{0x72, 1, {0x7}},
	{0x73, 1, {0x2}},
	{0x74, 1, {0x2}},
	{0x75, 1, {0x2}},
	{0x76, 1, {0x2}},
	{0x77, 1, {0x2}},
	{0x78, 1, {0x2}},
	{0x79, 1, {0x2}},
	{0x7A, 1, {0x2}},
	{0x7B, 1, {0x2}},
	{0x7C, 1, {0x7}},
	{0x7D, 1, {0x6}},
	{0x7E, 1, {0x2}},
	{0x7F, 1, {0x11}},
	{0x80, 1, {0x13}},
	{0x81, 1, {0x10}},
	{0x82, 1, {0x12}},
	{0x83, 1, {0xF}},
	{0x84, 1, {0xD}},
	{0x85, 1, {0xE}},
	{0x86, 1, {0xC}},
	{0x87, 1, {0x9}},
	{0x88, 1, {0x8}},
	{0x89, 1, {0x2}},
	{0x8A, 1, {0x2}},
	{0xFF, 3, {0x98, 0x81, 0x4}},
	{0x6C, 1, {0x15}},
	{0x6E, 1, {0x28}},
	{0x6F, 1, {0x35}},
	{0x3A, 1, {0xA4}},
	{0x8D, 1, {0x1F}},
	{0x87, 1, {0xBA}},
	{0x26, 1, {0x76}},
	{0xB2, 1, {0xD1}},
	{0x69, 1, {0x57}},
	{0x41, 1, {0x88}},
	{0x33, 1, {0x44}},
	{0xFF, 3, {0x98, 0x81, 0x1}},
	{0x22, 1, {0x38}},
	{0x31, 1, {0x0}},
	{0x34, 1, {0x1}},
	{0x40, 1, {0x33}},
	{0x53, 1, {0x66}},
	{0x55, 1, {0x8F}},
	{0x50, 1, {0x96}},
	{0x51, 1, {0x96}},
	{0x60, 1, {0x14}},
	{0xA0, 1, {0x1F}},
	{0xA1, 1, {0x37}},
	{0xA2, 1, {0x45}},
	{0xA3, 1, {0x16}},
	{0xA4, 1, {0x19}},
	{0xA5, 1, {0x2C}},
	{0xA6, 1, {0x1E}},
	{0xA7, 1, {0x21}},
	{0xA8, 1, {0xBB}},
	{0xA9, 1, {0x1D}},
	{0xAA, 1, {0x29}},
	{0xAB, 1, {0xA0}},
	{0xAC, 1, {0x1C}},
	{0xAD, 1, {0x19}},
	{0xAE, 1, {0x4D}},
	{0xAF, 1, {0x22}},
	{0xB0, 1, {0x29}},
	{0xB1, 1, {0x56}},
	{0xB2, 1, {0x63}},
	{0xB3, 1, {0x39}},
	{0xC0, 1, {0x0}},
	{0xC1, 1, {0x36}},
	{0xC2, 1, {0x46}},
	{0xC3, 1, {0x15}},
	{0xC4, 1, {0x1A}},
	{0xC5, 1, {0x2B}},
	{0xC6, 1, {0x21}},
	{0xC7, 1, {0x1F}},
	{0xC8, 1, {0xBB}},
	{0xC9, 1, {0x1D}},
	{0xCA, 1, {0x29}},
	{0xCB, 1, {0x9F}},
	{0xCC, 1, {0x1B}},
	{0xCD, 1, {0x19}},
	{0xCE, 1, {0x4D}},
	{0xCF, 1, {0x22}},
	{0xD0, 1, {0x27}},
	{0xD1, 1, {0x57}},
	{0xD2, 1, {0x64}},
	{0xD3, 1, {0x39}},
	{0xFF, 3, {0x98, 0x81, 0x0}},
	{0x36, 1, {0x1}},
	{0x35, 1, {0x0}},
	{0x11, 1, {0x0}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0x0}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x0, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Sleep mode off
	{0x28, 1, {0x0}},
	{REGFLAG_DELAY, 20, {}},

	// Sleep mode on
	{0x10, 1, {0x0}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x0, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dsi.HS_TRAIL = 10;
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.PLL_CLOCK = 207;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.cont_clock = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.esd_check_enable = 1;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_blanking_pixel = 60;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_sync_active = 60;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.lcm_esd_check_table[0].cmd = 0xA;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.packet_size = 256;
	params->dsi.ssc_disable = 1;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.vertical_backporch = 24;
	params->dsi.vertical_frontporch = 16;
	params->dsi.vertical_sync_active = 8;
	params->height = FRAME_HEIGHT;
	params->physical_height = 110;
	params->physical_width = 62;
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	cmd = 0x3;
	data = 0x0;

	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);

	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(10);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0, id1 = 0, id2 = 0;
	unsigned char buffer[3];
	unsigned int array[16];

	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);

	MDELAY(10);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x53902;
	array[1] = 0x8198FFFF;
	array[2] = 0x1;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10);

	array[0] = 0x23700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0x0, buffer, 1);
	id1 = buffer[0];

	read_reg_v2(0x1, buffer, 1);
	id2 = buffer[1];

	id2 = buffer[1];
	id = (id1 << 8) | id2;

	if (id == LCM_ID_ILI9881)
		return 1;
	else
		return 0;
}

static unsigned int lcm_esd_check(void)
{
	unsigned char buffer[8] = {0};
	unsigned int array[4];

	array[0] = 0x13700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xA, buffer, 8);

	if (buffer[0] != 0x9C)
		return 1;
	else
		return 0;
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	return 1;
}

LCM_DRIVER ili9881_CA_hd720_dsi_vdo_yassy_lcm_drv = {
	.name = "ili9881_CA_hd720_dsi_vdo_yassy",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover
};
