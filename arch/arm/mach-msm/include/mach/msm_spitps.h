/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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

#ifndef _MSM_SPITPS_H_
#define _MSM_SPITPS_H_

struct msm_sh_spitps_platform_data {
	int  gpio_irq;
	int  gpio_hssp_clk;
	int  gpio_hssp_data;
	int  gpio_reset;
	int  gpio_standby;
	int (*gpio_setup)(void);
	void (*gpio_teardown)(void);
};

#define	SH_TOUCH_I2C_DEVNAME	"SH_touchpanel"
#define	SH_TOUCH_SPI_DEVNAME	"SH_touchpanel"

#define	SH_TOUCH_IRQ			92
#define	SH_TOUCH_HSSP_CLK		88
#define	SH_TOUCH_HSSP_DATA		89
#define	SH_TOUCH_RESET			130
#define	SH_TOUCH_STBY			131

#define	SH_TOUCH_MAX_X			639
#define	SH_TOUCH_MAX_Y			1094
#define	SH_TOUCH_MAX_Z			255
#define	SH_TOUCH_MAX_DISTANCE	1268
#define	SH_TOUCH_MAX_DIAGONAL	1268
#define	SH_TOUCH_VKEY_MAX_X		SH_TOUCH_MAX_X
#define	SH_TOUCH_VKEY_MAX_Y		(SH_TOUCH_MAX_Y-135)

#define	FIRMDATA_SIZE			32768

#define TPSIF_DEV_NAME			"tpsif"
#define	TPSIF_DEV_FULLNAME		"/dev/tpsif"

#define TPSDRV_IOC_MAGIC            0xE0

#define	TPSDEV_ENABLE 				_IO  (TPSDRV_IOC_MAGIC, 0)
#define	TPSDEV_DISABLE				_IO  (TPSDRV_IOC_MAGIC, 1)
#define	TPSDEV_FW_VERSION			_IO  (TPSDRV_IOC_MAGIC, 2)
#define	TPSDEV_FW_DOWNLOAD			_IO  (TPSDRV_IOC_MAGIC, 3)
#define	TPSDEV_FW_UPDATE			_IO  (TPSDRV_IOC_MAGIC, 4)
#define	TPSDEV_START_TESTMODE		_IOW (TPSDRV_IOC_MAGIC, 5, int)
#define	TPSDEV_STOP_TESTMODE		_IO  (TPSDRV_IOC_MAGIC, 6)
#define	TPSDEV_GET_SENSOR			_IOR (TPSDRV_IOC_MAGIC, 7, int)
#define	TPSDEV_SET_FIRMPARAM		_IOW (TPSDRV_IOC_MAGIC, 8, int)
#define	TPSDEV_CALIBRATION_PARAM	_IOW (TPSDRV_IOC_MAGIC, 9, int)
#define	TPSDEV_SLEEP_ON				_IO  (TPSDRV_IOC_MAGIC, 10)
#define	TPSDEV_SLEEP_OFF			_IO  (TPSDRV_IOC_MAGIC, 11)
#define	TPSDEV_FW_UPDATE2			_IO  (TPSDRV_IOC_MAGIC, 12)

#define	TPSDEV_UPDATE_START			_IOR (TPSDRV_IOC_MAGIC, 13, int)
#define	TPSDEV_UPDATE_STOP			_IO  (TPSDRV_IOC_MAGIC, 14)
#define	TPSDEV_GPIO_RESET			_IO  (TPSDRV_IOC_MAGIC, 15)
#define	TPSDEV_GPIO_HSSPCLK			_IOW (TPSDRV_IOC_MAGIC, 16, int)
#define	TPSDEV_GPIO_HSSPDATA		_IOW (TPSDRV_IOC_MAGIC, 17, int)
#define	TPSDEV_GPIO_CLK_INOUT		_IOW (TPSDRV_IOC_MAGIC, 18, int)
#define	TPSDEV_GPIO_DATA_INOUT		_IOW (TPSDRV_IOC_MAGIC, 19, int)
#define	TPSDEV_RUN_CLOCK			_IOW (TPSDRV_IOC_MAGIC, 20, int)
#define	TPSDEV_RECV_BYTE			_IOR (TPSDRV_IOC_MAGIC, 21, int)
#define	TPSDEV_SEND_BYTE			_IOW (TPSDRV_IOC_MAGIC, 22, int)
#define	TPSDEV_DETECT_HILO			_IOW (TPSDRV_IOC_MAGIC, 23, int)
#define	TPSDEV_RECALIBRATION_IDAC	_IO  (TPSDRV_IOC_MAGIC, 24)

typedef struct
{
	uint8_t bData;
	int nNumBits;
	uint8_t bReset;
} Tps_send_data;

typedef struct
{
	uint16_t wPosX;						/* X Pos */
	uint16_t wPosY;						/* Y Pos */
	uint8_t  bSensData[160];			/* Sensor data */
	uint8_t  bCounter;					/* New Data Counter */
} TpsSensorData;


void msm_spitps_flipchange(int nFlipState);
void msm_tps_setsleep(int nIsSleep);
void msm_tps_shutdown(void);

#endif
