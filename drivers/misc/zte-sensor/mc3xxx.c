/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
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
 *
 *****************************************************************************/ 

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#include <mach/system.h>
#include <mach/hardware.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h> 
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#if 0
#define GSENSOR_DBG(format, args...)   printk(KERN_INFO "mc3xxx:%s( )_%d_: " format, \
	__FUNCTION__ , __LINE__, ## args);
#else
#define GSENSOR_DBG(format, args...);
#endif

//=== CONFIGURATIONS ==========================================================
//#define DOT_CALI
#define _MC3XXX_DEBUG_ON_
//=============================================================================
#ifdef _MC3XXX_DEBUG_ON_
    #define mcprintkreg(x...)     printk(x)
    #define mcprintkfunc(x...)    printk(x)
    #define GSE_ERR(x...) 	      printk(x)
    #define GSE_LOG(x...) 	      printk(x)
#else
    #define mcprintkreg(x...)
    #define mcprintkfunc(x...)
    #define GSE_ERR(x...)
    #define GSE_LOG(x...)
#endif

//=============================================================================
#define SENSOR_NAME              "mc3xxx"
#define SENSOR_DRIVER_VERSION    "1.1.0"
#define SENSOR_DATA_SIZE         3
#define GRAVITY_1G_VALUE    1024

//=============================================================================
#define SENSOR_DMARD_IOCTL_BASE          234

#define IOCTL_SENSOR_SET_DELAY_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 100)
#define IOCTL_SENSOR_GET_DELAY_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 101)
#define IOCTL_SENSOR_GET_STATE_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 102)
#define IOCTL_SENSOR_SET_STATE_ACCEL     _IO(SENSOR_DMARD_IOCTL_BASE, 103)
#define IOCTL_SENSOR_GET_DATA_ACCEL      _IO(SENSOR_DMARD_IOCTL_BASE, 104)

#define IOCTL_MSENSOR_SET_DELAY_MAGNE    _IO(SENSOR_DMARD_IOCTL_BASE, 200)
#define IOCTL_MSENSOR_GET_DATA_MAGNE     _IO(SENSOR_DMARD_IOCTL_BASE, 201)
#define IOCTL_MSENSOR_GET_STATE_MAGNE    _IO(SENSOR_DMARD_IOCTL_BASE, 202)
#define IOCTL_MSENSOR_SET_STATE_MAGNE    _IO(SENSOR_DMARD_IOCTL_BASE, 203)

#define IOCTL_SENSOR_GET_NAME            _IO(SENSOR_DMARD_IOCTL_BASE, 301)
#define IOCTL_SENSOR_GET_VENDOR          _IO(SENSOR_DMARD_IOCTL_BASE, 302)

#define IOCTL_SENSOR_GET_CONVERT_PARA    _IO(SENSOR_DMARD_IOCTL_BASE, 401)
#define SENSOR_CALIBRATION               _IOWR(SENSOR_DMARD_IOCTL_BASE, 402, int[SENSOR_DATA_SIZE])

//=============================================================================
#define MC3XXX_CONVERT_PARAMETER    (1.5f * (9.80665f) / 256.0f)
#define MC3XXX_DIPLAY_VENDOR        "mCube"
//=============================================================================
#define MC3XXX_DATA_LEN    6

#define MC3XXX_XOUT_REG						0x00
#define MC3XXX_YOUT_REG						0x01
#define MC3XXX_ZOUT_REG						0x02
#define MC3XXX_Tilt_Status_REG				0x03
#define MC3XXX_SAMPLING_RATE_STATUS_REG 	0x04
#define MC3XXX_SLEEP_COUNT_REG 				0x05
#define MC3XXX_INTERRUPT_ENABLE_REG 		0x06
#define MC3XXX_MODE_FEATURE_REG				0x07
#define MC3XXX_SAMPLE_RATE_REG				0x08
#define MC3XXX_TAP_DETECTION_ENABLE_REG		0x09
#define MC3XXX_TAP_DWELL_REJECT_REG			0x0a
#define MC3XXX_DROP_CONTROL_REG				0x0b
#define MC3XXX_SHAKE_DEBOUNCE_REG			0x0c
#define MC3XXX_XOUT_EX_L_REG				0x0d
#define MC3XXX_XOUT_EX_H_REG				0x0e
#define MC3XXX_YOUT_EX_L_REG				0x0f
#define MC3XXX_YOUT_EX_H_REG				0x10
#define MC3XXX_ZOUT_EX_L_REG				0x11
#define MC3XXX_ZOUT_EX_H_REG				0x12
#define MC3XXX_CHIP_ID_REG					0x18
#define MC3XXX_RANGE_CONTROL_REG			0x20
#define MC3XXX_SHAKE_THRESHOLD_REG			0x2B
#define MC3XXX_UD_Z_TH_REG					0x2C
#define MC3XXX_UD_X_TH_REG					0x2D
#define MC3XXX_RL_Z_TH_REG					0x2E
#define MC3XXX_RL_Y_TH_REG					0x2F
#define MC3XXX_FB_Z_TH_REG					0x30
#define MC3XXX_DROP_THRESHOLD_REG			0x31
#define MC3XXX_TAP_THRESHOLD_REG			0x32
#define MC3XXX_PCODE_REG					0x3B

#define MC3XXX_RESOLUTION_LOW     1
#define MC3XXX_RESOLUTION_HIGH    2

#define MC3XXX_WAKE       1
#define MC3XXX_SNIFF      2
#define MC3XXX_STANDBY    3

/***********************************************
 *** RETURN CODE
 ***********************************************/
#define MC3XXX_RETCODE_SUCCESS                 (0)
#define MC3XXX_RETCODE_ERROR_I2C               (-1)
#define MC3XXX_RETCODE_ERROR_STATUS            (-3)
#define MC3XXX_RETCODE_ERROR_SETUP             (-4)
#define MC3XXX_RETCODE_ERROR_GET_DATA          (-5)
#define MC3XXX_RETCODE_ERROR_IDENTIFICATION    (-6)


/***********************************************
 *** PRODUCT ID
 ***********************************************/
#define MC3XXX_PCODE_3210     0x90
#define MC3XXX_PCODE_3230     0x19
#define MC3XXX_PCODE_3250     0x88
#define MC3XXX_PCODE_3410     0xA8
#define MC3XXX_PCODE_3410N    0xB8
#define MC3XXX_PCODE_3430     0x29
#define MC3XXX_PCODE_3430N    0x39
#define MC3XXX_PCODE_3510B    0x40
#define MC3XXX_PCODE_3530B    0x30
#define MC3XXX_PCODE_3510C    0x10
#define MC3XXX_PCODE_3530C    0x60

//=============================================================================
#define MC3XXX_I2C_NAME            SENSOR_NAME
#define SENSOR_DEV_COUNT           1
#define SENSOR_DURATION_MAX        200
#define SENSOR_DURATION_MIN        10
#define SENSOR_DURATION_DEFAULT    100

#define MAX_RETRY     20
#define INPUT_FUZZ    0
#define INPUT_FLAT    0

#define MC3XXX_BUFSIZE    256

static unsigned char    s_bResolution = 0x00;
static unsigned char    s_bPCODE      = 0x00;

static       unsigned short     mc3xxx_i2c_auto_probe_addr[] = { 0x4C, 0x6C, 0x4E, 0x6D, 0x6E, 0x6F };
typedef struct {
	unsigned short	x;		/**< X axis */
	unsigned short	y;		/**< Y axis */
	unsigned short	z;		/**< Z axis */
} GSENSOR_VECTOR3D;

static GSENSOR_VECTOR3D gsensor_gain = { 0 };
static struct miscdevice mc3xxx_device;
//=============================================================================
#ifdef DOT_CALI
static unsigned char is_new_mc34x0 = 0;
static unsigned char is_mc3250 = 0;
//#define CALIB_PATH              "/data/data/com.mcube.acc/files/mcube-calib.txt"
#define DATA_PATH            		  "/sdcard/mcube-register-map.txt"
static char file_path[MC3XXX_BUFSIZE] = "/data/data/com.mcube.acc/files/mcube-calib.txt";
//static char factory_path[MC3XXX_BUFSIZE] ="/data/data/com.mcube.acc/files/fac-calib.txt";

static struct file * fd_file = NULL;
static mm_segment_t oldfs = { 0 };
static unsigned char offset_buf[9] = { 0 };
static signed int offset_data[3] = { 0 };
static signed int gain_data[3] = { 0 };
static signed int enable_RBM_calibration = 0;

#define GSENSOR                                0x95
#define GSENSOR_IOCTL_INIT                     _IO(GSENSOR,  0x01)
#define GSENSOR_IOCTL_READ_CHIPINFO            _IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA          _IOR(GSENSOR, 0x03, int)
#define GSENSOR_IOCTL_READ_RAW_DATA            _IOR(GSENSOR, 0x06, int)
//#define GSENSOR_IOCTL_SET_CALI                 _IOW(GSENSOR, 0x06, SENSOR_DATA)
#define GSENSOR_IOCTL_GET_CALI                 _IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI                 _IO(GSENSOR, 0x08)
#define GSENSOR_MCUBE_IOCTL_READ_RBM_DATA      _IOR(GSENSOR, 0x09, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_SET_RBM_MODE       _IO(GSENSOR, 0x0a)
#define GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE     _IO(GSENSOR, 0x0b)
#define GSENSOR_MCUBE_IOCTL_SET_CALI           _IOW(GSENSOR, 0x0c, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_REGISTER_MAP       _IO(GSENSOR, 0x0d)

typedef struct{
	int x;
	int y;
	int z;
}SENSOR_DATA;

static int load_cali_flg = 0;
static unsigned char mc3xxx_current_placement = MC3XXX_BOTTOM_LEFT_UP; // current soldered placement

enum mc3xxx_orientation
{
    MC3XXX_TOP_LEFT_DOWN = 0,
    MC3XXX_TOP_RIGHT_DOWN,
    MC3XXX_TOP_RIGHT_UP,
    MC3XXX_TOP_LEFT_UP,
    MC3XXX_BOTTOM_LEFT_DOWN,
    MC3XXX_BOTTOM_RIGHT_DOWN,
    MC3XXX_BOTTOM_RIGHT_UP,
    MC3XXX_BOTTOM_LEFT_UP
};
struct mc3xxx_hwmsen_convert
{
    signed char sign[3];
    unsigned char map[3];
};

// Transformation matrix for chip mounting position
static const struct mc3xxx_hwmsen_convert mc3xxx_cvt[] =
{
    {{ 1,  1,  1}, {0, 1, 2}},    // 0: top   , left-down
    {{-1,  1,  1}, {1, 0, 2}},    // 1: top   , right-down
    {{-1, -1,  1}, {0, 1, 2}},    // 2: top   , right-up
    {{ 1, -1,  1}, {1, 0, 2}},    // 3: top   , left-up
    {{-1,  1, -1}, {0, 1, 2}},    // 4: bottom, left-down
    {{ 1,  1, -1}, {1, 0, 2}},    // 5: bottom, right-down
    {{ 1, -1, -1}, {0, 1, 2}},    // 6: bottom, right-up
    {{-1, -1, -1}, {1, 0, 2}},    // 7: bottom, left-up
};

//=============================================================================
#define REMAP_IF_MC3250_READ(nDataX, nDataY)                        \
            if (MC3XXX_PCODE_3250 == s_bPCODE)                      \
            {                                                       \
                int    _nTemp = 0;                                  \
                                                                    \
                _nTemp = nDataX;                                    \
                nDataX = nDataY;                                    \
                nDataY = -_nTemp;                                    \
                GSE_LOG("[%s] 3250 read remap\n", __FUNCTION__);    \
            }

#define REMAP_IF_MC3250_WRITE(nDataX, nDataY)                        \
            if (MC3XXX_PCODE_3250 == s_bPCODE)                       \
            {                                                        \
                int    _nTemp = 0;                                   \
                                                                     \
                _nTemp = nDataX;                                     \
                nDataX = -nDataY;                                    \
                nDataY = _nTemp;                                      \
                GSE_LOG("[%s] 3250 write remap\n", __FUNCTION__);    \
            }

#define REMAP_IF_MC34XX_N(nDataX, nDataY)                                                \
            if ((MC3XXX_PCODE_3410N == s_bPCODE) || (MC3XXX_PCODE_3430N == s_bPCODE))    \
            {                                                                            \
                nDataX = -nDataX;                                                        \
                nDataY = -nDataY;                                                        \
                GSE_LOG("[%s] 34X0N remap\n", __FUNCTION__);                             \
            }

#define REMAP_IF_MC35XX(nDataX, nDataY)                                                                                                                          \
            if ((MC3XXX_PCODE_3510B == s_bPCODE) || (MC3XXX_PCODE_3510C == s_bPCODE) || (MC3XXX_PCODE_3530B == s_bPCODE) || (MC3XXX_PCODE_3530C == s_bPCODE))    \
            {                                                                                                                                                    \
                nDataX = -nDataX;                                                                                                                                \
                nDataY = -nDataY;                                                                                                                                \
                GSE_LOG("[%s] 35X0 remap\n", __FUNCTION__);                                                                                                      \
            }
#endif  // END OF #ifdef DOT_CALI

#define IS_MC35XX()    ((MC3XXX_PCODE_3510B == s_bPCODE) || (MC3XXX_PCODE_3510C == s_bPCODE) || (MC3XXX_PCODE_3530B == s_bPCODE) || (MC3XXX_PCODE_3530C == s_bPCODE))

struct mc3xxx_data {
	struct mutex lock;
	struct i2c_client *client;
	struct work_struct  work;
	struct workqueue_struct *mc3xxx_wq;
	struct hrtimer timer;
	struct input_dev *input_dev;
	int enabled;
	volatile unsigned int duration; 

	int axis_map_x;
	int axis_map_y;
	int axis_map_z;
	int negate_x;
	int negate_y;
	int negate_z;
	bool vdd_enabled;
	bool vio_enabled;
	struct regulator *vdd;
	struct regulator *vio;
};

static int mc3xxx_enable(struct mc3xxx_data *data, int enable);
static int mc3xxx_power_init(struct mc3xxx_data *data, bool on);

#ifdef CONFIG_OF
#define MC3XXX_VDD_MIN_UV        2700000
#define MC3XXX_VDD_MAX_UV        3300000
#define MC3XXX_VIO_MIN_UV        1700000
#define MC3XXX_VIO_MAX_UV        1950000
#endif

/*****************************************
 *** mc3xxx_validate_sensor_IC
 //checks if the product is mCube or not.
 // Thus, the return code is TRUE/FALSE instead of Product-ID.
 *****************************************/
static int mc3xxx_validate_sensor_IC(unsigned char bPCode)
{
	GSENSOR_DBG();
    if (   (MC3XXX_PCODE_3210  == bPCode) || (MC3XXX_PCODE_3230  == bPCode)
        || (MC3XXX_PCODE_3250  == bPCode)
        || (MC3XXX_PCODE_3410  == bPCode) || (MC3XXX_PCODE_3430  == bPCode)
        || (MC3XXX_PCODE_3410N == bPCode) || (MC3XXX_PCODE_3430N == bPCode)
        || (MC3XXX_PCODE_3510B == bPCode) || (MC3XXX_PCODE_3530B == bPCode)
        || (MC3XXX_PCODE_3510C == bPCode) || (MC3XXX_PCODE_3530C == bPCode) )
    {
        return (MC3XXX_RETCODE_SUCCESS);
    }    

    return (MC3XXX_RETCODE_ERROR_IDENTIFICATION);
}
/*****************************************
 *** MC3XXX_SetResolution
 *****************************************/
static void mc3xxx_set_resolution(void)
{
	GSENSOR_DBG();
    switch (s_bPCODE)
    {
    case MC3XXX_PCODE_3230:
    case MC3XXX_PCODE_3430:
    case MC3XXX_PCODE_3430N:
    case MC3XXX_PCODE_3530B:
    case MC3XXX_PCODE_3530C:
         s_bResolution = MC3XXX_RESOLUTION_LOW;
         break;

    case MC3XXX_PCODE_3210:
    case MC3XXX_PCODE_3250:
    case MC3XXX_PCODE_3410:
    case MC3XXX_PCODE_3410N:
    case MC3XXX_PCODE_3510B:
    case MC3XXX_PCODE_3510C:
         s_bResolution = MC3XXX_RESOLUTION_HIGH;
         break;

    default:
         GSE_ERR("ERR: no resolution assigned!\n");
         break;
    }

    GSE_LOG("[%s] s_bResolution: %d\n", __FUNCTION__, s_bResolution);
}

/*****************************************
 *** MC3XXX_ConfigRegRange
 *****************************************/
static void mc3xxx_config_regRange(struct i2c_client *client)
{
    unsigned char    _baDataBuf[2] = { 0 };

    _baDataBuf[0] = MC3XXX_RANGE_CONTROL_REG;
    _baDataBuf[1] = 0x3F;
	GSENSOR_DBG();
    if (MC3XXX_RESOLUTION_LOW == s_bResolution)
        _baDataBuf[1] = 0x32;

    if ((MC3XXX_PCODE_3510B == s_bPCODE) || (MC3XXX_PCODE_3510C == s_bPCODE))
        _baDataBuf[1] = 0xA5;
    else if ((MC3XXX_PCODE_3530B == s_bPCODE) || (MC3XXX_PCODE_3530C == s_bPCODE))
        _baDataBuf[1] = 0x02;
	
	i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);

    GSE_LOG("[%s] set 0x%X\n", __FUNCTION__, _baDataBuf[1]);
}

/*****************************************
 *** MC3XXX_SetGain
 *****************************************/
static void mc3xxx_set_gain(void)
{
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;
	GSENSOR_DBG();
    if (MC3XXX_RESOLUTION_LOW == s_bResolution)
    {
        gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 86;

        if ((MC3XXX_PCODE_3530B == s_bPCODE) || (MC3XXX_PCODE_3530C == s_bPCODE))
        {
            gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 64;
        }
    }
    
    GSE_LOG("[%s] gain: %d / %d / %d\n", __FUNCTION__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
}

static ssize_t mc3xxx_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	struct mc3xxx_data *mc3xxx = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", mc3xxx->enabled);
}

//=============================================================================
static ssize_t mc3xxx_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mc3xxx_data *mc3xxx = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	
	printk("mc3xxx set poll enable %ld\n", val);
	mc3xxx_enable(mc3xxx, val);

	return count;
}

//=============================================================================
static ssize_t mc3xxx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mc3xxx_data *mc3xxx = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", mc3xxx->duration);
}

//=============================================================================
static ssize_t mc3xxx_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mc3xxx_data *mc3xxx = dev_get_drvdata(dev);
	unsigned long data = 0;
	
	if(strict_strtoul(buf, 10, &data))
		return -EINVAL;

	if (data > SENSOR_DURATION_MAX)
		data = SENSOR_DURATION_MAX;
	if (data < SENSOR_DURATION_MIN)
		data = SENSOR_DURATION_MIN;

	mc3xxx->duration = data;

	return count;
}

//=============================================================================

static struct device_attribute mc3xxx_attrs[] =
{
	__ATTR(enable_device, S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_enable_show, mc3xxx_enable_store),
	__ATTR(pollrate_ms, S_IRUGO | S_IWUSR | S_IWGRP, mc3xxx_delay_show, mc3xxx_delay_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(mc3xxx_attrs); i++) 
                if (device_create_file(dev, mc3xxx_attrs + i))
                        goto error;
        return 0;

error:
        for ( ; i >= 0; i--) 
                device_remove_file(dev, mc3xxx_attrs + i);
        dev_err(dev, "%s:Unable to create interface\n", __func__);
        return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(mc3xxx_attrs); i++) 
                device_remove_file(dev, mc3xxx_attrs + i);
        return 0;
}

int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode) 
{
	int comres = 0;
	unsigned char data = 0;
	GSENSOR_DBG();
	if (mode < 4)
	{
		data = (0x40 | mode);
		comres = i2c_smbus_write_byte_data(client, MC3XXX_MODE_FEATURE_REG, data);
	} 

	return comres;
}

//=============================================================================
static int mc3xxx_chip_init(struct i2c_client *client)
{
	unsigned char  _baDataBuf[2] = { 0 };
	GSENSOR_DBG();
	
	mc3xxx_set_mode(client, MC3XXX_STANDBY);	
	//Louis, test code ==================================================
	// config HIGH resolution
	if ((0x10 == s_bPCODE) || (0x60 == s_bPCODE))
		s_bPCODE = 0x60;
	//Louis, test code ==================================================
	
	mc3xxx_set_resolution();
		
	_baDataBuf[0] = MC3XXX_SAMPLE_RATE_REG;
	_baDataBuf[1] = 0x00;
	if (IS_MC35XX())
		_baDataBuf[1] = 0x0A;
	i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);
	_baDataBuf[0] = MC3XXX_TAP_DETECTION_ENABLE_REG;
	_baDataBuf[1] = 0x00;
	i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);
	_baDataBuf[0] = MC3XXX_INTERRUPT_ENABLE_REG;
	_baDataBuf[1] = 0x00;
	i2c_smbus_write_byte_data(client, _baDataBuf[0], _baDataBuf[1]);
	
	mc3xxx_config_regRange(client);
	mc3xxx_set_gain();	
	return 0;
}

#ifdef DOT_CALI
//=============================================================================
struct file *openFile(char *path, int flag, int mode) 
{ 
	struct file *fp = NULL; 
	GSENSOR_DBG();
	fp = filp_open(path, flag, mode); 

	if (IS_ERR(fp) || !fp->f_op) 
	{
		GSE_LOG("Calibration File filp_open return NULL\n");
		return NULL; 
	}

	return fp; 
} 
 
//=============================================================================
int readFile(struct file *fp, char *buf, int readlen) 
{ 
	GSENSOR_DBG();
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

//=============================================================================
int writeFile(struct file *fp, char *buf, int writelen) 
{ 
	GSENSOR_DBG();
	if (fp->f_op && fp->f_op->write) 
		return fp->f_op->write(fp,buf,writelen, &fp->f_pos); 
	else 
		return -1; 
}
 
//=============================================================================
int closeFile(struct file *fp) 
{ 
	GSENSOR_DBG();
	filp_close(fp, NULL); 

	return 0; 
} 

//=============================================================================
void initKernelEnv(void) 
{ 
	GSENSOR_DBG();
	oldfs = get_fs(); 
	set_fs(KERNEL_DS);
	printk(KERN_INFO "initKernelEnv\n");
} 

//=============================================================================
int MC3XXX_WriteCalibration(struct i2c_client *client, int dat[3])
{
	int err = 0;
	u8 buf[9] = { 0 };
	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
    int temp_cali_dat[3] = { 0 };
	const struct mc3xxx_hwmsen_convert *pCvt = NULL;

    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;
    s32 dwRangePosLimit  = 0x1FFF;
    s32 dwRangeNegLimit  = -0x2000;

	pCvt = &mc3xxx_cvt[mc3xxx_current_placement];
	GSENSOR_DBG();
	temp_cali_dat[pCvt->map[0]] = pCvt->sign[0] * dat[0];
	temp_cali_dat[pCvt->map[1]] = pCvt->sign[1] * dat[1];
	temp_cali_dat[pCvt->map[2]] = pCvt->sign[2] * dat[2];

	temp_cali_dat[0] = ((temp_cali_dat[0] * gsensor_gain.x) / GRAVITY_1G_VALUE);
	temp_cali_dat[1] = ((temp_cali_dat[1] * gsensor_gain.y) / GRAVITY_1G_VALUE);
	temp_cali_dat[2] = ((temp_cali_dat[2] * gsensor_gain.z) / GRAVITY_1G_VALUE);

	REMAP_IF_MC3250_READ(temp_cali_dat[0], temp_cali_dat[1]);
	REMAP_IF_MC34XX_N(temp_cali_dat[0], temp_cali_dat[1]);
	REMAP_IF_MC35XX(temp_cali_dat[0], temp_cali_dat[1]);

    dat[0] = temp_cali_dat[0];
    dat[1] = temp_cali_dat[1];
    dat[2] = temp_cali_dat[2];
 
	GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", dat[0], dat[1], dat[2]);

    // read register 0x21~0x29
	err  = i2c_smbus_read_i2c_block_data(client , 0x21 , 3 , &buf[0]);
	err |= i2c_smbus_read_i2c_block_data(client , 0x24 , 3 , &buf[3]);
	err |= i2c_smbus_read_i2c_block_data(client , 0x27 , 3 , &buf[6]);

	

	if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
        dwRangePosLimit  = 0x3FFF;
        dwRangeNegLimit  = -0x4000;
    }

	// get x,y,z offset
    tmp = ((buf[1] & bMsbFilter) << 8) + buf[0];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    x_off = tmp;
    
    tmp = ((buf[3] & bMsbFilter) << 8) + buf[2];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    y_off = tmp;
    
    tmp = ((buf[5] & bMsbFilter) << 8) + buf[4];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    z_off = tmp;
					
	// get x,y,z gain
	x_gain = ((buf[1] >> 7) << 8) + buf[6];
	y_gain = ((buf[3] >> 7) << 8) + buf[7];
	z_gain = ((buf[5] >> 7) << 8) + buf[8];
								
	// prepare new offset
	x_off = x_off + 16 * dat[0] * 256 * 128 / 3 / gsensor_gain.x / (40 + x_gain);
	y_off = y_off + 16 * dat[1] * 256 * 128 / 3 / gsensor_gain.y / (40 + y_gain);
	z_off = z_off + 16 * dat[2] * 256 * 128 / 3 / gsensor_gain.z / (40 + z_gain);

	// range check
	if (x_off > dwRangePosLimit) 
	    x_off = dwRangePosLimit;
	else if (x_off < dwRangeNegLimit)
	    x_off = dwRangeNegLimit;

	if (y_off > dwRangePosLimit) 
	    y_off = dwRangePosLimit;
	else if (y_off < dwRangeNegLimit)
	    y_off = dwRangeNegLimit;

	if (z_off > dwRangePosLimit) 
	    z_off = dwRangePosLimit;
	else if (z_off < dwRangeNegLimit)
	    z_off = dwRangeNegLimit;

	//storege the cerrunt offset data with DOT format
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	//storege the cerrunt Gain data with GOT format
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);
	GSE_LOG("%d %d ======================\n\n ", gain_data[0], x_gain);

	buf[0] = 0x43;
	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = x_off & 0xff;
	buf[1] = ((x_off >> 8) & bMsbFilter) | (x_gain & 0x0100 ? 0x80 : 0);
	buf[2] = y_off & 0xff;
	buf[3] = ((y_off >> 8) & bMsbFilter) | (y_gain & 0x0100 ? 0x80 : 0);
	buf[4] = z_off & 0xff;
	buf[5] = ((z_off >> 8) & bMsbFilter) | (z_gain & 0x0100 ? 0x80 : 0);

	i2c_smbus_write_i2c_block_data(client, 0x21,   2, &buf[0]);
	i2c_smbus_write_i2c_block_data(client, 0x21+2, 2, &buf[2]);
	i2c_smbus_write_i2c_block_data(client, 0x21+4, 2, &buf[4]);
	
	buf[0] = 0x41;
	i2c_smbus_write_byte_data(client, 0x07,buf[0]);

    msleep(50);

	return err;
}

//=============================================================================
int mcube_read_cali_file(struct i2c_client *client)
{
	int cali_data[3] = { 0 };
	int err = 0;
	char buf[64] = { 0 };
	GSENSOR_DBG();
	GSE_LOG("%s %d\n",__func__,__LINE__);

	initKernelEnv();

	fd_file = openFile(file_path, O_RDONLY, 0); 

	if (fd_file == NULL) 
	{
		GSE_LOG("mc3xxx fail to open\n");
		cali_data[0] = 0;
		cali_data[1] = 0;
		cali_data[2] = 0;

		return -1;
	}
	else
	{
		memset(buf, 0, sizeof(buf)); 

		if ((err = readFile(fd_file, buf, sizeof(buf))) > 0) 
			GSE_LOG("buf:%s\n",buf); 
		else 
			GSE_LOG("read file error %d\n",err); 

		set_fs(oldfs); 
		closeFile(fd_file); 

		sscanf(buf, "%d %d %d", &cali_data[0], &cali_data[1], &cali_data[2]);
		GSE_LOG("cali_data: %d %d %d\n", cali_data[0], cali_data[1], cali_data[2]); 	
				
		MC3XXX_WriteCalibration(client, cali_data);
	}

	return 0;
}

//=============================================================================
static int mcube_write_log_data(struct i2c_client *client, u8 data[0x3f])
{
	#define _WRT_LOG_DATA_BUFFER_SIZE    (66 * 50)

	s16 rbm_data[3]={0}, raw_data[3]={0};
	int err =0;
	char *_pszBuffer = NULL;
	int n=0,i=0;
	GSENSOR_DBG();
	initKernelEnv();
	fd_file = openFile(DATA_PATH ,O_RDWR | O_CREAT,0); 
	if (fd_file == NULL) 
	{
		GSE_LOG("mcube_write_log_data fail to open\n");	
	}
	else
	{
		rbm_data[0] = (s16)((data[0x0d]) | (data[0x0e] << 8));
		rbm_data[1] = (s16)((data[0x0f]) | (data[0x10] << 8));
		rbm_data[2] = (s16)((data[0x11]) | (data[0x12] << 8));

		raw_data[0] = (rbm_data[0] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		raw_data[1] = (rbm_data[1] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		raw_data[2] = (rbm_data[2] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];

		_pszBuffer = kzalloc(_WRT_LOG_DATA_BUFFER_SIZE, GFP_KERNEL);
		if (NULL == _pszBuffer)
		{
			GSE_ERR("fail to allocate memory for buffer\n");
    		closeFile(fd_file); 
			return -1;
		}
		memset(_pszBuffer, 0, _WRT_LOG_DATA_BUFFER_SIZE); 

		n += sprintf(_pszBuffer+n, "G-sensor RAW X = %d  Y = %d  Z = %d\n", raw_data[0] ,raw_data[1] ,raw_data[2]);
		n += sprintf(_pszBuffer+n, "G-sensor RBM X = %d  Y = %d  Z = %d\n", rbm_data[0] ,rbm_data[1] ,rbm_data[2]);
		for(i=0; i<64; i++)
		{
		n += sprintf(_pszBuffer+n, "mCube register map Register[%x] = 0x%x\n",i,data[i]);
		}
		msleep(50);		
		if ((err = writeFile(fd_file,_pszBuffer,n))>0) 
			GSE_LOG("buf:%s\n",_pszBuffer); 
		else 
			GSE_LOG("write file error %d\n",err); 

		kfree(_pszBuffer);

		set_fs(oldfs); 
		closeFile(fd_file); 
	}
	return 0;
}

//=============================================================================
void MC3XXX_rbm(struct i2c_client *client, int enable)
{
    char buf1[3] = { 0 };
	GSENSOR_DBG();
    if (enable == 1)
    {
        buf1[0] = 0x43; 
        i2c_smbus_write_byte_data(client, 0x07, buf1[0]);

        buf1[0] = 0x02; 
        i2c_smbus_write_byte_data(client, 0x14, buf1[0]);

        buf1[0] = 0x41; 
        i2c_smbus_write_byte_data(client, 0x07, buf1[0]);

        enable_RBM_calibration = 1;

        GSE_LOG("set rbm!!\n");

        msleep(220);
    }
    else if(enable == 0 )  
    {
        buf1[0] = 0x43; 
        i2c_smbus_write_byte_data(client, 0x07, buf1[0]);

        buf1[0] = 0x00; 
        i2c_smbus_write_byte_data(client, 0x14, buf1[0]);

        buf1[0] = 0x41; 
        i2c_smbus_write_byte_data(client, 0x07, buf1[0]);

        enable_RBM_calibration = 0;

        GSE_LOG("clear rbm!!\n");

        msleep(220);
    }
}

//=============================================================================
int MC3XXX_ReadOffset(struct i2c_client *client,s16 ofs[3])
{    
	int err = 0;
	u8 off_data[6] = { 0 };
	GSENSOR_DBG();
	if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
	{
		err = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, MC3XXX_DATA_LEN, off_data);

		ofs[0] = ((s16)(off_data[0]))|((s16)(off_data[1])<<8);
		ofs[1] = ((s16)(off_data[2]))|((s16)(off_data[3])<<8);
		ofs[2] = ((s16)(off_data[4]))|((s16)(off_data[5])<<8);
	}
	else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
	{
		err = i2c_smbus_read_i2c_block_data(client, 0, 3, off_data);

		ofs[0] = (s8)off_data[0];
		ofs[1] = (s8)off_data[1];
		ofs[2] = (s8)off_data[2];			
	}
	REMAP_IF_MC3250_READ(ofs[0], ofs[1]);
	REMAP_IF_MC34XX_N(ofs[0], ofs[1]);
	REMAP_IF_MC35XX(ofs[0], ofs[1]);

	GSE_LOG("MC3XXX_ReadOffset %d %d %d\n", ofs[0], ofs[1], ofs[2]);

    return err;  
}

//=============================================================================
int MC3XXX_ResetCalibration(struct i2c_client *client)
{
	u8 buf[6] = { 0 };
	s16 tmp = 0;
	int err = 0;
    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;
	GSENSOR_DBG();
	buf[0] = 0x43;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if(err)
	{
		GSE_ERR("error 0x07: %d\n", err);
	}

	err = i2c_smbus_write_i2c_block_data(client, 0x21, 6, offset_buf);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}
	
	buf[0] = 0x41;
	err = i2c_smbus_write_byte_data(client, 0x07, buf[0]);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
	}

	msleep(20);

	
	if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
    }

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[0] = tmp;
					
	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];
	if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	offset_data[1] = tmp;
					
	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[2] = tmp;	

	return 0;  
}

//=============================================================================
int MC3XXX_ReadCalibration(struct i2c_client *client, int dat[3])
{
    signed short MC_offset[3 + 1] = { 0 };    // +1: for 4-byte alignment
    int err = 0;
	GSENSOR_DBG();
	memset(MC_offset, 0, sizeof(MC_offset));

	err = MC3XXX_ReadOffset(client, MC_offset);

    if (err)
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    dat[0] = MC_offset[0];
    dat[1] = MC_offset[1];
    dat[2] = MC_offset[2];  
                                      
    return 0;
}

//=============================================================================
int MC3XXX_ReadData(struct i2c_client *client, s16 buffer[3])
{
	unsigned char buf[6] = { 0 };
	signed char buf1[6] = { 0 };
	char rbm_buf[6] = { 0 };
	int ret = 0;
	GSENSOR_DBG();
	if (enable_RBM_calibration == 0)
	{
		//err = hwmsen_read_block(client, addr, buf, 0x06);
	}
	else if (enable_RBM_calibration == 1)
	{		
		memset(rbm_buf, 0, 6);
        i2c_smbus_read_i2c_block_data(client, 0x0d  , 2, &rbm_buf[0]);
        i2c_smbus_read_i2c_block_data(client, 0x0d+2, 2, &rbm_buf[2]);
        i2c_smbus_read_i2c_block_data(client, 0x0d+4, 2, &rbm_buf[4]);
	}

	if (enable_RBM_calibration == 0)
	{
		if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
		
		{
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_EX_L_REG, 6, buf);
			
			buffer[0] = (signed short)((buf[0])|(buf[1]<<8));
			buffer[1] = (signed short)((buf[2])|(buf[3]<<8));
			buffer[2] = (signed short)((buf[4])|(buf[5]<<8));
		}

		else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
		{
			ret = i2c_smbus_read_i2c_block_data(client, MC3XXX_XOUT_REG, 3, buf1);
				
			buffer[0] = (signed short)buf1[0];
			buffer[1] = (signed short)buf1[1];
			buffer[2] = (signed short)buf1[2];
		}

	}
	else if (enable_RBM_calibration == 1)
	{
		buffer[0] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
		buffer[1] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
		buffer[2] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

		if(gain_data[0] == 0)
		{
			buffer[0] = 0;
			buffer[1] = 0;
			buffer[2] = 0;

			return 0;
		}

		buffer[0] = (buffer[0] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		buffer[1] = (buffer[1] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		buffer[2] = (buffer[2] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];	
	}
	
	return 0;
}

//=============================================================================
int MC3XXX_ReadRawData(struct i2c_client *client, char * buf)
{
	int res = 0;
	s16 raw_buf[3] = { 0 };
	GSENSOR_DBG();
	if (!buf || !client)
	{
		return -EINVAL;
	}
	
	mc3xxx_set_mode(client, MC3XXX_WAKE);
	res = MC3XXX_ReadData(client, &raw_buf[0]);
	if(res)
	{     
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	else
	{
    	const struct mc3xxx_hwmsen_convert *pCvt = &mc3xxx_cvt[mc3xxx_current_placement];
        raw_buf[0] = ((raw_buf[0] * GRAVITY_1G_VALUE) / gsensor_gain.x);
        raw_buf[1] = ((raw_buf[1] * GRAVITY_1G_VALUE) / gsensor_gain.y);
        raw_buf[2] = ((raw_buf[2] * GRAVITY_1G_VALUE) / gsensor_gain.z);

        if (is_new_mc34x0)
        {
            raw_buf[0] = -raw_buf[0];
            raw_buf[1] = -raw_buf[1];
        }
        else if (is_mc3250)
        {
            s16    temp = 0;

            temp = raw_buf[0];

            raw_buf[0] = raw_buf[1];
            raw_buf[1] = -temp;
        }

        raw_buf[0] = pCvt->sign[0] * raw_buf[pCvt->map[0]];
        raw_buf[1] = pCvt->sign[1] * raw_buf[pCvt->map[1]];
        raw_buf[2] = pCvt->sign[2] * raw_buf[pCvt->map[2]];

		sprintf(buf, "%04x %04x %04x", raw_buf[0], raw_buf[1], raw_buf[2]);
	}

	return 0;
}

//=============================================================================
static int MC3XXX_ReadRegMap(struct i2c_client *client)
{
	u8 data[128] = {0};
	u8 addr = 0x00;
	int err = 0;
	int i = 0;
	GSENSOR_DBG();
	if(NULL == client)
	{
		err = -EINVAL;
		return err;
	}

    for (i = 0; i < 11; i++)
    {
    	err |= i2c_smbus_read_i2c_block_data(client, addr, 6, &data[addr]);
    	addr +=6;
    }

	for(i = 0; i < 64; i++)
		printk(KERN_INFO "mcube register map Register[%x] = 0x%x\n", i ,data[i]);

	msleep(50);	
	
	mcube_write_log_data(client, data);

	msleep(50);
     	
	return err;
}

//=============================================================================
void MC3XXX_Reset(struct i2c_client *client) 
{
	s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
	s32 x_off = 0, y_off = 0, z_off = 0;
	u8 buf[3] = { 0 };
	int err = 0;
	GSENSOR_DBG();
	buf[0] = 0x43;
  	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = 0x6d;
  	i2c_smbus_write_byte_data(client, 0x1b, buf[0]);
  	
	buf[0] = 0x43;
  	i2c_smbus_write_byte_data(client, 0x1b, buf[0]);

	msleep(5);
	
	buf[0] = 0x43;
  	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = 0x80;
  	i2c_smbus_write_byte_data(client, 0x1c, buf[0]);

	buf[0] = 0x80;
  	i2c_smbus_write_byte_data(client, 0x17, buf[0]);
	
	msleep(5);

	buf[0] = 0x41;
  	i2c_smbus_write_byte_data(client, 0x07, buf[0]);

	buf[0] = 0x00;
  	i2c_smbus_write_byte_data(client, 0x1c, buf[0]);

	buf[0] = 0x00;
  	i2c_smbus_write_byte_data(client, 0x17, buf[0]);

	msleep(5);

	memset(offset_buf, 0, 9);

	err = i2c_smbus_read_i2c_block_data(client, 0x21, 9, offset_buf);

    tmp = ((offset_buf[1] & 0x3f) << 8) + offset_buf[0];
    if (tmp & 0x2000)
        tmp |= 0xc000;
    x_off = tmp;

    tmp = ((offset_buf[3] & 0x3f) << 8) + offset_buf[2];
    if (tmp & 0x2000)
        tmp |= 0xc000;
    y_off = tmp;

    tmp = ((offset_buf[5] & 0x3f) << 8) + offset_buf[4];
    if (tmp & 0x2000)
        tmp |= 0xc000;
    z_off = tmp;
					
	// get x,y,z gain
	x_gain = ((offset_buf[1] >> 7) << 8) + offset_buf[6];
	y_gain = ((offset_buf[3] >> 7) << 8) + offset_buf[7];
	z_gain = ((offset_buf[5] >> 7) << 8) + offset_buf[8];
							
	//storege the cerrunt offset data with DOT format
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	//storege the cerrunt Gain data with GOT format
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);

	GSE_LOG("offser gain = %d %d %d %d %d %d======================\n\n ",
		gain_data[0], gain_data[1], gain_data[2], offset_data[0], offset_data[1], offset_data[2]);
}

#endif

//=============================================================================
int mc3xxx_read_accel_xyz(struct mc3xxx_data *data, int *acc)
{
	int comres = 0;
	s16 raw_data[3] = { 0 };
	
#ifdef DOT_CALI       
    comres = MC3XXX_ReadData(data->client, &raw_data[0]);
#else
    unsigned char raw_buf[6] = { 0 };
    signed char raw_buf1[3] = { 0 };

	if(MC3XXX_RESOLUTION_HIGH == s_bResolution)
        {
            comres = i2c_smbus_read_i2c_block_data(data->client, MC3XXX_XOUT_EX_L_REG, 6, raw_buf);
            
            raw_data[0] = (signed short)((raw_buf[0])|(raw_buf[1]<<8));
            raw_data[1] = (signed short)((raw_buf[2])|(raw_buf[3]<<8));
            raw_data[2] = (signed short)((raw_buf[4])|(raw_buf[5]<<8));
        }

	else if(MC3XXX_RESOLUTION_LOW == s_bResolution)
        {
            comres = i2c_smbus_read_i2c_block_data(data->client, MC3XXX_XOUT_REG, 3, raw_buf1);
            
            raw_data[0] = (signed short)raw_buf1[0];
            raw_data[1] = (signed short)raw_buf1[1];
            raw_data[2] = (signed short)raw_buf1[2];
        }
    #endif
	
	raw_data[0] = ((raw_data[0] * GRAVITY_1G_VALUE) / gsensor_gain.x);
	raw_data[1] = ((raw_data[1] * GRAVITY_1G_VALUE) / gsensor_gain.y);
	raw_data[2] = ((raw_data[2] * GRAVITY_1G_VALUE) / gsensor_gain.z);

	acc[0] = ((data->negate_x) ? (-raw_data[data->axis_map_x]): (raw_data[data->axis_map_x]));
	acc[1] = ((data->negate_y) ? (-raw_data[data->axis_map_y]): (raw_data[data->axis_map_y]));
	acc[2] = ((data->negate_z) ? (-raw_data[data->axis_map_z]): (raw_data[data->axis_map_z]));
	
	return comres;
}

//=============================================================================
static int mc3xxx_measure(struct mc3xxx_data *data, int *accel)
{
    #ifdef DOT_CALI
        int ret = 0;

        if( load_cali_flg > 0)
        {
            ret = mcube_read_cali_file(data->client);

            if(ret == 0)
                load_cali_flg = ret;
            else 
                load_cali_flg--;

            GSE_LOG("load_cali %d\n",ret); 
        }  
    #endif
	GSENSOR_DBG();

	mc3xxx_read_accel_xyz(data, accel);
	
	return 0;
}

//=============================================================================
static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *data = container_of(work, struct mc3xxx_data, work);
	int accel[3] = { 0 };
	GSENSOR_DBG();
	mc3xxx_measure(data, accel);
	
	input_report_abs(data->input_dev, ABS_X, accel[0]);
	input_report_abs(data->input_dev, ABS_Y, accel[1]);
	input_report_abs(data->input_dev, ABS_Z, accel[2]);
	input_sync(data->input_dev);
}

//=============================================================================
static enum hrtimer_restart mc3xxx_timer_func(struct hrtimer *timer)
{
	struct mc3xxx_data *data = container_of(timer, struct mc3xxx_data, timer);

	queue_work(data->mc3xxx_wq, &data->work);

	hrtimer_start(&data->timer, ktime_set(0, data->duration * 1000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

//=============================================================================
static int mc3xxx_enable(struct mc3xxx_data *data, int enable)
{
	if(enable)
	{
		mc3xxx_power_init(data, true);
		mc3xxx_chip_init(data->client);
		mc3xxx_set_mode(data->client, MC3XXX_WAKE);
		hrtimer_start(&data->timer, ktime_set(0, data->duration * 1000000), HRTIMER_MODE_REL);
		data->enabled = true;
	}
	else
	{
		hrtimer_cancel(&data->timer);
		mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
		data->enabled = false;
		mc3xxx_power_init(data, false);
	}

	return 0;
}

//=============================================================================
static long mc3xxx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
#ifdef DOT_CALI
	void __user *data1 = NULL;
	char strbuf[256] = { 0 };
	int cali[3] = { 0 };
	SENSOR_DATA sensor_data = { 0 };
#endif
	struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);
	struct mc3xxx_data *data = NULL;

	data = i2c_get_clientdata(client);
	switch (cmd) {		
#ifdef DOT_CALI		
	case GSENSOR_IOCTL_READ_SENSORDATA:	
	case GSENSOR_IOCTL_READ_RAW_DATA:
	case GSENSOR_MCUBE_IOCTL_READ_RBM_DATA:
        mutex_lock(&data->lock);
		MC3XXX_ReadRawData(client, strbuf);
        mutex_unlock(&data->lock);

		if (copy_to_user((void __user *) arg, &strbuf, strlen(strbuf)+1)) {
			printk("failed to copy sense data to user space.");
			return -EFAULT;
		}
		break;

	case GSENSOR_MCUBE_IOCTL_SET_CALI:
		data1 = (void __user *)arg;
		if(data1 == NULL)
		{
			ret = -EINVAL;
			break;	  
		}
		if(copy_from_user(&sensor_data, data1, sizeof(sensor_data)))
		{
			ret = -EFAULT;
			break;	  
		}
		else
		{
			cali[0] = sensor_data.x;
			cali[1] = sensor_data.y;
			cali[2] = sensor_data.z;	
				
            mutex_lock(&data->lock);
			ret = MC3XXX_WriteCalibration(client, cali);			 
            mutex_unlock(&data->lock);
		}
		break;
		
	case GSENSOR_IOCTL_CLR_CALI:
        mutex_lock(&data->lock);
		ret = MC3XXX_ResetCalibration(client);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_IOCTL_GET_CALI:	
		data1 = (unsigned char*)arg;
		if(data1 == NULL)
		{
			ret = -EINVAL;
			break;	  
		}

		if((ret = MC3XXX_ReadCalibration(client, cali)))
		{
			GSE_LOG("fwq mc3xxx MC3XXX_ReadCalibration error!!!!\n");
			break;
		}

		sensor_data.x = cali[0];
		sensor_data.y = cali[1];
		sensor_data.z = cali[2];

		if(copy_to_user(data1, &sensor_data, sizeof(sensor_data)))
		{
			ret = -EFAULT;
			break;
		}		
		break;	

	case GSENSOR_MCUBE_IOCTL_SET_RBM_MODE:
        mutex_lock(&data->lock);
		MC3XXX_rbm(client, 1);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE:
        mutex_lock(&data->lock);
		MC3XXX_rbm(client, 0);
        mutex_unlock(&data->lock);
		break;

	case GSENSOR_MCUBE_IOCTL_REGISTER_MAP:
        MC3XXX_ReadRegMap(client);
		break;		
#endif		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

//=============================================================================
static int mc3xxx_open(struct inode *inode, struct file *filp)
{
	return nonseekable_open(inode, filp);
}

//=============================================================================
static int mc3xxx_release(struct inode *inode, struct file *filp)
{
	return 0;
}

//=============================================================================
static struct file_operations sensor_fops =
{
    .owner          = THIS_MODULE,
    .open       	= mc3xxx_open,
    .release    	= mc3xxx_release,
    .unlocked_ioctl = mc3xxx_ioctl,
};

//=============================================================================
static int mc3xxx_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	if (data != NULL){
		hrtimer_cancel(&data->timer);
		if (data->enabled)
			mc3xxx_set_mode(data->client, MC3XXX_STANDBY);
	}
	return 0;
}

//=============================================================================
static int mc3xxx_resume(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	if (data != NULL){
		if (data->enabled){
			mc3xxx_chip_init(data->client); 
#ifdef DOT_CALI
			MC3XXX_ResetCalibration(data->client); 
			mcube_read_cali_file(data->client); 
#endif
			mc3xxx_set_mode(data->client, MC3XXX_WAKE);
			hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}
	return 0;
}

//=============================================================================
static struct miscdevice mc3xxx_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name  = SENSOR_NAME,
    .fops  = &sensor_fops,
};

/*****************************************
 *** _mc3xxx_i2c_auto_probe
 *****************************************/
static int mc3xxx_i2c_auto_probe(struct i2c_client *client)
{
	unsigned char    _baDataBuf[2] = {0};
	int              _nProbeAddrCount = (sizeof(mc3xxx_i2c_auto_probe_addr) / sizeof(mc3xxx_i2c_auto_probe_addr[0]));
	int              _nCount = 0;

	//i2c_smbus_read_byte_data(client, 0x3B);
	s_bPCODE = 0x00;
	GSENSOR_DBG();

	for (_nCount = 0; _nCount < _nProbeAddrCount; _nCount++)
	{
		client->addr = mc3xxx_i2c_auto_probe_addr[_nCount];
		GSE_LOG("[%s] probing addr: 0x%X\n", __FUNCTION__, client->addr);
		_baDataBuf[0] = MC3XXX_PCODE_REG;
		if (0 > i2c_master_send(client, &(_baDataBuf[0]), 1))
		{
			GSE_ERR("ERR: addr: 0x%X fail to communicate-2!\n", client->addr);
			continue;
		}

		if (0 > i2c_master_recv(client, &(_baDataBuf[0]), 1))
		{
			GSE_ERR("ERR: addr: 0x%X fail to communicate-3!\n", client->addr);
			continue;
		}

		GSE_LOG("[%s] addr: 0x%X ok to read REG(0x3B): 0x%X\n", __FUNCTION__, client->addr, _baDataBuf[0]);

		if (MC3XXX_RETCODE_SUCCESS == mc3xxx_validate_sensor_IC(_baDataBuf[0]))
		{
			GSE_LOG("[%s] addr: 0x%X confirmed ok to use.\n", __FUNCTION__, client->addr);
			s_bPCODE = _baDataBuf[0];
			return (MC3XXX_RETCODE_SUCCESS);
		}
	}

	return (MC3XXX_RETCODE_ERROR_I2C);
}

#ifdef CONFIG_OF
static int mc3xxx_power_init(struct mc3xxx_data *data, bool on)
{
        int rc;
        int vol_uv;
        int enabled = 0;

        if (!on) {
                if (data->vdd_enabled) {
                        regulator_disable(data->vdd);
                        data->vdd_enabled = false;
                }
                if (data->vio_enabled) {
                        regulator_disable(data->vio);
                        data->vio_enabled = false;
                }

                regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);
                regulator_set_voltage(data->vio, 0, MC3XXX_VIO_MAX_UV);
        } else {
        		if (!(data->vdd)){
                	data->vdd = regulator_get(&data->client->dev, "vdd");
                	if (IS_ERR(data->vdd)) {
                        	rc = PTR_ERR(data->vdd);
                        	dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
                       	 return rc;
                	}
        		}
                vol_uv = regulator_get_voltage(data->vdd);
                enabled = regulator_is_enabled(data->vdd);
                dev_info(&data->client->dev, "vdd current voltate is %duv, enabled = %d ", vol_uv, enabled);
                if (((vol_uv <= MC3XXX_VDD_MIN_UV) || (vol_uv >= MC3XXX_VDD_MAX_UV))
                 						&& (regulator_count_voltages(data->vdd) > 0)) {
                        rc = regulator_set_voltage(data->vdd, MC3XXX_VDD_MIN_UV,
                                        MC3XXX_VDD_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vdd rc=%d\n",
                                                rc);
                                goto reg_vdd_put;
                        }
                }
                if (enabled != 1) {
                        rc = regulator_enable(data->vdd);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator vdd enable failed rc=%d\n", rc);
                                goto reg_vdd_put;
                        }
                        data->vdd_enabled = true;
                }

				if (!(data->vio)){
                	data->vio = regulator_get(&data->client->dev, "vio");
                	if (IS_ERR(data->vio)) {
                        	rc = PTR_ERR(data->vio);
                        	dev_err(&data->client->dev,
                                        "Regulator get failed vio rc=%d\n", rc);
                        	goto reg_vdd_dis;

					}
				}

                vol_uv = regulator_get_voltage(data->vio);
                enabled = regulator_is_enabled(data->vio);
                dev_info(&data->client->dev, "vio current voltate is %duv, enabled = %d \n", vol_uv, enabled);
                if (((vol_uv <= MC3XXX_VIO_MIN_UV) || (vol_uv >= MC3XXX_VIO_MAX_UV))
                        		&& (regulator_count_voltages(data->vio) > 0)) {
                        rc = regulator_set_voltage(data->vio, MC3XXX_VIO_MIN_UV,
                                        MC3XXX_VIO_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vio rc=%d\n", rc);
                                goto reg_vio_put;
                        }
                }
                if (enabled != 1) {
					rc = regulator_enable(data->vio);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator vio enable failed rc=%d\n", rc);
                                goto reg_vio_put;
                        }
                        data->vio_enabled = true;
                        msleep(10);
                }
        }
        return 0;

reg_vio_put:
        regulator_put(data->vio);
reg_vdd_dis:
        if (data->vdd_enabled){
                regulator_disable(data->vdd);
                data->vdd_enabled = false;
        }
reg_vdd_put:
        regulator_put(data->vdd);
        return rc;
}

static int mc3xxx_parse_dt(struct device *dev,
                                struct mc3xxx_data *data)
{
        struct device_node *np = dev->of_node;
        u32 temp_val;
        int rc;

        rc = of_property_read_u32(np, "mc,axis-map-x", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map_x\n");
                return rc;
        } else {
                data->axis_map_x = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "mc,axis-map-y", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis_map_y\n");
                return rc;
        } else {
                data->axis_map_y = (u8)temp_val;
        }

        rc = of_property_read_u32(np, "mc,axis-map-z", &temp_val);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read axis-map-z\n");
                return rc;
        } else {
                data->axis_map_z = (u8)temp_val;
        }

        data->negate_x = of_property_read_bool(np, "mc,negate-x");

        data->negate_y = of_property_read_bool(np, "mc,negate-y");

        data->negate_z = of_property_read_bool(np, "mc,negate-z");

        return 0;
}

static int mc3xxx_parse_gpio(struct device *dev)
{
        struct device_node *np = dev->of_node;
        u32 temp_val;
        int rc;

        temp_val = of_get_named_gpio(np, "gpio-irq-pin", 0);
        if (gpio_is_valid(temp_val)) {
                rc = gpio_request(temp_val, "mc3xxx_irq");
                if (rc) {
                        dev_err(dev, "irq gpio request failed");
                } else
                        gpio_direction_input(temp_val);
        }
        return 0;
}
#else 
static int mc3xxx_power_init(struct mc3xxx_data *data, bool on)
{
	return 0;
}
static int mc3xxx_parse_dt(struct device *dev,
                                struct mc3xxx_data *data)
{
        return -ENODEV;
}

static int mc3xxx_parse_gpio(struct device *dev)
{
        return 0;
}

#endif
//=============================================================================
static int mc3xxx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct mc3xxx_data *data = NULL;
	GSENSOR_DBG();

    if (!i2c_check_functionality(client->adapter,
                     I2C_FUNC_SMBUS_BYTE |I2C_FUNC_SMBUS_BYTE_DATA |I2C_FUNC_SMBUS_WORD_DATA)) {
    	dev_err(&client->dev, "client not smb-i2c capable:1\n");
    	ret = -EIO;
    	goto err_check_functionality_failed;
	}

    if (!i2c_check_functionality(client->adapter,
                   	I2C_FUNC_SMBUS_I2C_BLOCK)){
   		dev_err(&client->dev, "client not smb-i2c capable:2\n");
    	ret = -EIO;
   		goto err_check_functionality_failed;
	}

	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if(data == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	data->client = client;
    ret = mc3xxx_parse_dt(&client->dev, data);
    if (ret) {
        dev_err(&client->dev,
        	"Unable to parse platfrom data err=%d\n", ret);
        goto err_parse_dt;
    }

	data->vdd_enabled = false;
    data->vio_enabled = false;
    ret = mc3xxx_power_init(data, true);
    if (ret < 0) {
   		dev_err(&client->dev, "power init failed! err=%d", ret);
    	goto err_parse_dt;
    }

	if (MC3XXX_RETCODE_SUCCESS != mc3xxx_i2c_auto_probe(client))
	{
		dev_err(&client->dev,"fail to probe mCube sensor!\n");
		goto err_power_deinit;
	}
	mc3xxx_parse_gpio(&client->dev);
	
#ifdef DOT_CALI
	load_cali_flg = 30;
#endif
	
	data->mc3xxx_wq = create_singlethread_workqueue("mc3xxx_wq");
	if (!data->mc3xxx_wq)
	{
		ret = -ENOMEM;
		goto err_create_workqueue_failed;
	}
	INIT_WORK(&data->work, mc3xxx_work_func);

	data->duration = SENSOR_DURATION_DEFAULT;

	i2c_set_clientdata(client, data);	

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		ret = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_X, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Y, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(data->input_dev, ABS_Z, -32*8, 32*8, INPUT_FUZZ, INPUT_FLAT);

	data->input_dev->name = "zte_acc";
        data->input_dev->id.bustype = BUS_I2C;
        data->input_dev->dev.parent = &client->dev;

	ret = input_register_device(data->input_dev);
	if (ret) {
		goto exit_input_register_device_failed;
	}

#ifdef DOT_CALI
    MC3XXX_Reset(client);
	mutex_init(&data->lock);
#endif
	
    mc3xxx_device.parent = &client->dev;
 
	ret = misc_register(&mc3xxx_device);
	if (ret) {
		goto exit_misc_device_register_failed;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
        dev_err(&client->dev,
                   "ucube sysfs register failed\n");
        goto exit_sysfs_create_failed;
    }

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = mc3xxx_timer_func;

	printk("mc3xxx probe ok \n");

	return 0;

exit_sysfs_create_failed:
	misc_deregister(&mc3xxx_device);
exit_misc_device_register_failed:
	input_unregister_device(data->input_dev);
	goto exit_input_dev_alloc_failed;
exit_input_register_device_failed:
	input_free_device(data->input_dev);
exit_input_dev_alloc_failed:
	destroy_workqueue(data->mc3xxx_wq);	
err_create_workqueue_failed:
err_power_deinit:
	mc3xxx_power_init(data, false);
	regulator_put(data->vdd);
    regulator_put(data->vio);
err_parse_dt:
	kfree(data);	
err_alloc_data_failed:
err_check_functionality_failed:
	printk("mc3xxx probe failed \n");
	return ret;
}

//=============================================================================
static int mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	hrtimer_cancel(&data->timer);
	input_unregister_device(data->input_dev);	
	misc_deregister(&mc3xxx_device);
	remove_sysfs_interfaces(&client->dev);
	destroy_workqueue(data->mc3xxx_wq);
	mc3xxx_power_init(data, false);
	regulator_put(data->vdd);
    regulator_put(data->vio);
	kfree(data);
	return 0;
}


static ssize_t acc_info_read_proc(char *page, char **start, off_t off, 
                                int count, int *eof, void *data)
{
        return sprintf(page, "0x%x\n", s_bPCODE);
}

static struct proc_dir_entry *acc_info_proc_file;
static int __init create_acc_info_proc_file(void)
{
        if(0x0 == s_bPCODE) {
                printk("not create mc3xxx proc file due to unconnected\n");
                return 0;
        }    
        acc_info_proc_file = create_proc_entry("driver/accel", 0644, NULL);
        printk("goes to create_acc_info_proc_file\n");
        if (acc_info_proc_file) {
                acc_info_proc_file->read_proc = acc_info_read_proc;
        } else{
                printk(KERN_INFO "proc file create failed!\n");
        }    
        return 0;
}

//=============================================================================
static struct of_device_id mc3xxx_match_table[] = {
	{ .compatible = "mc,mc3xxx", },
	{ },
};

static const struct i2c_device_id mc3xxx_id[] =
{
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static struct i2c_driver mc3xxx_driver =
{
    .driver = {
                  .owner = THIS_MODULE,
                  .name	 = SENSOR_NAME,
                  .of_match_table = mc3xxx_match_table,
              },

    .id_table	  = mc3xxx_id,
    .probe		  = mc3xxx_probe,
    .remove		  = mc3xxx_remove,
    .suspend	  = mc3xxx_suspend,
    .resume		  = mc3xxx_resume,
};

module_i2c_driver(mc3xxx_driver);
late_initcall(create_acc_info_proc_file);

MODULE_DESCRIPTION("mc3xxx accelerometer driver");
MODULE_AUTHOR("mCube-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(SENSOR_DRIVER_VERSION);

