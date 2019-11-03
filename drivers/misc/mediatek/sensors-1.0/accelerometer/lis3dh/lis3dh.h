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

#ifndef LIS3DH_H
#define LIS3DH_H

#include <linux/ioctl.h>

extern struct acc_hw *lis3dh_get_cust_acc_hw(void);

#define LIS3DH_I2C_SLAVE_ADDR	0x32

/* LIS3DH Register Map (Please refer to LIS3DH Specifications) */
#define LIS3DH_REG_CTL_REG1		0x20
#define LIS3DH_REG_CTL_REG2		0x21
#define LIS3DH_REG_CTL_REG3		0x22
#define LIS3DH_REG_CTL_REG4		0x23
#define LIS3DH_REG_DATAX0		0x28
#define LIS3DH_REG_OUT_X		0x28
#define LIS3DH_REG_OUT_Y		0x2A
#define LIS3DH_REG_OUT_Z		0x2C

#define LIS3DH_FIXED_DEVID		0xE5

#define LIS3DH_BW_200HZ			0x60
#define LIS3DH_BW_100HZ			0x50
#define LIS3DH_BW_50HZ			0x40

#define	LIS3DH_FULLRANG_LSB		0XFF

#define LIS3DH_MEASURE_MODE		0x08
#define LIS3DH_DATA_READY		0x07

#define LIS3DH_RANGE_2G			0x00
#define LIS3DH_RANGE_4G			0x10
#define LIS3DH_RANGE_8G			0x20

#define LIS3DH_SELF_TEST		0x10

#define LIS3DH_STREAM_MODE		0x80
#define LIS3DH_SAMPLES_15		0x0F

#define LIS3DH_FS_8G_LSB_G		0x20
#define LIS3DH_FS_4G_LSB_G		0x10
#define LIS3DH_FS_2G_LSB_G		0x00

#define LIS3DH_LEFT_JUSTIFY		0x04
#define LIS3DH_RIGHT_JUSTIFY	0x00

#define LIS3DH_SUCCESS				 0
#define LIS3DH_ERR_I2C				-1
#define LIS3DH_ERR_STATUS			-3
#define LIS3DH_ERR_SETUP_FAILURE	-4
#define LIS3DH_ERR_GETGSENSORDATA	-5
#define LIS3DH_ERR_IDENTIFICATION	-6

#define LIS3DH_BUFSIZE				256

#endif
