/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/types.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k3l2_eeprom.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include "kd_camera_typedef.h"

#define PFX "S5K3L2_EEPROM_FMT"

/* #define CAM_CALGETDLT_DEBUG */
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)     pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALINF(x, ...)
#define CAM_CALDB(x, ...)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/


static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;


static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

#define I2C_SPEED 100
#define MAX_LSC_SIZE 1508
#define MAX_OTP_SIZE 2048
static int s5k3l2_eeprom_read = 0;
extern void S5K3L2_write_cmos_sensor_eeprom(unsigned int addr, unsigned int para);
extern unsigned short S5K3L2_read_cmos_sensor_eeprom(unsigned int addr);
typedef struct {
#if 0
	u16	   ChipInfo; //chip id, lot Id, Chip No. Etc
	u8     IdGroupWrittenFlag; //"Bit[7:6]: Flag of WB_Group_0  00: empty  01: valid group 11 or 10: invalid group"
	u8     ModuleInfo; //MID, 0x02 for truly
	u8     Year;
	u8     Month;
	u8     Day;
	u8     LensInfo;
	u8     VcmInfo;
	u8     DriverIcInfo;
	u8     LightTemp;
#endif
    u8     flag;
	u32    CaliVer;//0xff000b01
	u16    SerialNum;
	u8     Version;//0x01
	u8     AwbAfInfo;//0xF
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u16    AfInfinite;
	u16    AfMacro;
	u16    LscSize;
	u8   Lsc[MAX_LSC_SIZE];
}OTP_MTK_TYPE;

typedef union {
        u8 Data[MAX_OTP_SIZE];
        OTP_MTK_TYPE       MtkOtpData;
} OTP_DATA;

#if 0
void otp_clear_flag(void){
	spin_lock(&g_CAM_CALLock);
	_otp_read = 0;
	spin_unlock(&g_CAM_CALLock);
}
#endif

OTP_DATA s5k3l2_eeprom_data = {{0}};
u8 af_inf_data=0;


static int read_cmos_sensor(unsigned short slave_id, unsigned int addr, u8 *data)
{
	char pu_send_cmd[2] = {(char)(addr & 0xFF) };
	kdSetI2CSpeed(I2C_SPEED);
	return iReadRegI2C(pu_send_cmd, 1, data, 1, slave_id);/* 0 for good */
}



int read_s5k3l2_eeprom(u8 slv_id, u16 offset, u8* data)
{
    int ret = 0;

	ret = read_cmos_sensor(slv_id,offset,data);
	CAM_CALDB("OTP read slv_id 0x%x offset 0x%x  data 0x%x\n", slv_id,offset,*data);

	return ret;
}

int read_s5k3l2_eeprom_size(u8 slv_id, u16 offset, u8* data,int size)
{
	int i = 0;
	for(i = 0; i < size; i++){
		if(read_s5k3l2_eeprom(slv_id, offset+i, data+i) != 0)
			return -1;
	}
	return 0;
}
void S5K3L2_write_cmos_sensor_eeprom(unsigned int addr, unsigned int para)
{
		char puSendCmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
		
		kdSetI2CSpeed(300);
		iWriteRegI2C(puSendCmd, 3, 0x20);
}
unsigned short S5K3L2_read_cmos_sensor_eeprom(unsigned int addr)
{
		unsigned short get_byte=0;
		char puSendCmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
			
		kdSetI2CSpeed(300);
		iReadRegI2C(puSendCmd, 2, (u8*)&get_byte, 1, 0x20);
			
		return get_byte&0x00ff;
}

#define CAL_VERSION_MAGIC ""
int read_s5k3l2_eeprom_mtk_fmt(void)
{
	int i = 0;
	int offset = 0;
	unsigned int checksum1 = 0;	
	unsigned int checksum2 = 0;
	CAM_CALINF("OTP readed =%d \n",s5k3l2_eeprom_read);
	if(1 == s5k3l2_eeprom_read ) {
		CAM_CALDB("OTP readed ! skip\n");
		return 1;
	}
	spin_lock(&g_CAM_CALLock);
	s5k3l2_eeprom_read = 1;
	spin_unlock(&g_CAM_CALLock);
	offset = 0;
	//read 2048 byte
    for(i = 0xA0; i<0xAE; i+=2 )
	{
	 read_s5k3l2_eeprom_size(i,0x00, &s5k3l2_eeprom_data.Data[offset],256);	 
	 offset += 256;
    }
	//checksum1-19
	for(i=1;i<=19;i++)
	checksum1 += s5k3l2_eeprom_data.Data[i];	
	CAM_CALINF("liukun OTP01 checksum1 =%d \n",checksum1);
	checksum1 = (checksum1)%0xFFFF +1;
	checksum2 = ((s5k3l2_eeprom_data.Data[20])<<8) + (s5k3l2_eeprom_data.Data[21]);
	CAM_CALINF("liukun OTP01 checksum2 =%d \n",checksum2);
	if (checksum1 != checksum2)
	{
		CAM_CALERR("checksum 1-19 fail\n");
	}
	//checksum23-34
	checksum1 = 0;
	for(i=23;i<=34;i++)
	checksum1 += s5k3l2_eeprom_data.Data[i];	
	CAM_CALINF("liukun OTP02 checksum1 =%d \n",checksum1);
	checksum1 = (checksum1)%0xFFFF +1;
	checksum2 = ((s5k3l2_eeprom_data.Data[35])<<8) + (s5k3l2_eeprom_data.Data[36]);
	CAM_CALINF("liukun OTP02 checksum2 =%d \n",checksum2);
	if (checksum1 != checksum2)
	{
		CAM_CALERR("checksum 23-34 fail\n");
    	//return -1;
	}
	//checksum38-43
    checksum1 = 0;
	for(i=38;i<=43;i++)
	checksum1 += s5k3l2_eeprom_data.Data[i];	
	CAM_CALINF("liukun OTP03 checksum1 =%d \n",checksum1);
	checksum1 = (checksum1)%0xFFFF +1;
	checksum2 = ((s5k3l2_eeprom_data.Data[44])<<8) + (s5k3l2_eeprom_data.Data[45]);
	CAM_CALINF("liukun OTP03 checksum2 =%d \n",checksum2);
	af_inf_data = s5k3l2_eeprom_data.Data[41];
	CAM_CALINF("liukun af_inf_data =%d \n",af_inf_data);
	if (checksum1 != checksum2)
	{
		CAM_CALERR("checksum 38-43 fail\n");
    	//return -1;
	}
	//checksum47-1555
    checksum1 = 0;
	for(i=47;i<=1555;i++)
	checksum1 += s5k3l2_eeprom_data.Data[i];	
	CAM_CALINF("liukun OTP04 checksum1 =%d \n",checksum1);
	checksum1 = (checksum1)%0xFFFF +1;
	checksum2 = ((s5k3l2_eeprom_data.Data[1556])<<8) + (s5k3l2_eeprom_data.Data[1557]);
	CAM_CALINF("liukun OTP04 checksum2 =%d \n",checksum2);
	if (checksum1 != checksum2)
	{
		CAM_CALERR("checksum 47-1555 fail\n");
    	//return -1;
	}
	return 0;
}

void read_s5k3l2_eeprom_awb(void)
{
 unsigned int rg_ratio;
 unsigned int bg_ratio;
 unsigned int grgb_ratio;
 unsigned int golden_rg_ratio;
 unsigned int golden_bg_ratio;
 unsigned int golden_grgb_ratio;
 unsigned int R_gain, Gr_gain, Gb_gain, B_gain;
 rg_ratio = ((s5k3l2_eeprom_data.Data[23])<<8) + (s5k3l2_eeprom_data.Data[24]);
 bg_ratio = ((s5k3l2_eeprom_data.Data[25])<<8) + (s5k3l2_eeprom_data.Data[26]);
 grgb_ratio = ((s5k3l2_eeprom_data.Data[27])<<8) + (s5k3l2_eeprom_data.Data[28]);
 golden_rg_ratio = ((s5k3l2_eeprom_data.Data[29])<<8) + (s5k3l2_eeprom_data.Data[30]);
 golden_bg_ratio = ((s5k3l2_eeprom_data.Data[31])<<8) + (s5k3l2_eeprom_data.Data[32]);
 golden_grgb_ratio = ((s5k3l2_eeprom_data.Data[33])<<8) + (s5k3l2_eeprom_data.Data[34]);
 CAM_CALINF("liukun OTP rg_ratio =%x,bg_ratio=%x,grgb_ratio=%x\n",rg_ratio,bg_ratio,grgb_ratio);
 CAM_CALINF("liukun OTP golden_rg_ratio =%x,golden_bg_ratio=%x,golden_grgb_ratio=%x\n",golden_rg_ratio,golden_bg_ratio,golden_grgb_ratio);
 
 Gr_gain = 256;
 Gb_gain = 256 * golden_grgb_ratio / grgb_ratio;
 Gb_gain = (512 * Gb_gain) / (Gb_gain + Gr_gain);
 Gr_gain = (512 * Gr_gain) / (Gb_gain + Gr_gain);
 
 R_gain = 256 * golden_rg_ratio / rg_ratio;
 B_gain = 256 * golden_bg_ratio / bg_ratio;
 CAM_CALINF("liukun OTP Gr_gain =%x,R_gain=%x,B_gain=%x,Gb_gain=%x\n",Gr_gain,R_gain,B_gain,Gb_gain);
 S5K3L2_write_cmos_sensor_eeprom(0x020e, Gr_gain >> 8); //g_gain
 S5K3L2_write_cmos_sensor_eeprom(0x020f, Gr_gain & 0xFFFF); //g_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0210, R_gain >> 8); //r_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0211, R_gain & 0xFFFF); //r_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0212, B_gain >> 8); //b_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0213, B_gain & 0xFFFF); //b_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0214, Gb_gain >> 8); //g_gain
 S5K3L2_write_cmos_sensor_eeprom(0x0215, Gb_gain & 0xFFFF); //g_gain   
}
u8 read_s5k3l2_eeprom_vendor_id(void)
{
  CAM_CALINF("liukun OTP vendor id =%x \n",s5k3l2_eeprom_data.Data[1]);
  return s5k3l2_eeprom_data.Data[1];
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, (compat_uptr_t __user *)&data->pu1Params);
    err |= put_user(p, (compat_uptr_t __user *)&data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k3l2eeprom_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[CAMERA SENSOR] s5k3l2_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{
    memcpy((void *)data,(void *)&s5k3l2_eeprom_data.Data[offset],size);
	CAM_CALDB("selective_read_region offset =%x size %d data read = %d\n", offset,size, *data);
    return size;
}



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR(" ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            CAM_CALDB("[liukun CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[liukun CAM_CAL] length %d \n", ptempbuf->u4Length);
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, s5k3l2_EEPROM_DEVICE_ID,ptempbuf->u4Length);
            CAM_CALDB("[liukun CAM_CAL] After read Working buffer data  0x%x \n", *pu1Params);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[liukun CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[liukun CAM_CAL] to user  Working buffer address 0x%p \n", pu1Params);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[liukun CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("liukun CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("liukun Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = s5k3l2eeprom_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
//#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR(" Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR(" Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
    CAM_CALDB("CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB(" register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB(" Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("failed to register s5k3l2_eeprom driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("failed to register s5k3l2_eeprom driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);