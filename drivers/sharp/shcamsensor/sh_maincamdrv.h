/* drivers/sharp/shcamsensor/sh_maincamdrv.h  (Camera Driver)
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION
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
#ifndef __SH_MAINCAM_H
#define __SH_MAINCAM_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "linux/types.h"

#include <mach/board.h>
#include <media/msm_camera.h>

#if 1
typedef unsigned int   uint32;
typedef unsigned short uint16;
typedef unsigned char  uint8;
typedef signed int     int32;
typedef signed short   int16;
typedef signed char    int8;
#endif

#define SH_MAINCAM_IOCTL_MAGIC 'm'

#define SH_MAINCAM_IOCTL_READ_IRQ_KIND				_IOR(SH_MAINCAM_IOCTL_MAGIC,  1, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_ENABLE_IRQ					_IOW(SH_MAINCAM_IOCTL_MAGIC,  2, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_DISABLE_IRQ				_IOW(SH_MAINCAM_IOCTL_MAGIC,  3, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_CAMIF_PAD_RESET			_IOW(SH_MAINCAM_IOCTL_MAGIC,  4, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_REQUEST_IRQ				_IOW(SH_MAINCAM_IOCTL_MAGIC,  5, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_FREE_IRQ					_IOW(SH_MAINCAM_IOCTL_MAGIC,  6, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_GPIO_CTRL					_IOW(SH_MAINCAM_IOCTL_MAGIC,  7, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_EVENT_REQUEST_ISR			_IOW(SH_MAINCAM_IOCTL_MAGIC,  8, struct sh_maincam_event_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_FRAMEEVENT_REQUEST_ISR		_IOW(SH_MAINCAM_IOCTL_MAGIC,  9, struct sh_maincam_event_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_PRODUCT_REQUEST_ISR		_IOW(SH_MAINCAM_IOCTL_MAGIC, 10, struct sh_maincam_shdiag_production_info_cmd *)

#define SH_MAINCAM_IOCTL_SET_DEBUG_LOG				_IOW(SH_MAINCAM_IOCTL_MAGIC, 11, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_CAMINT_EXIT				_IOW(SH_MAINCAM_IOCTL_MAGIC, 12, struct sh_maincam_ctrl_cmd *)

#define CAM_INT_TYPE_VS						1
#define CAM_INT_TYPE_INT					2
#define CAM_INT_TYPE_EVENT_ISR				3
#define CAM_INT_TYPE_FRAMEEVENT_ISR			4
#define CAM_INT_TYPE_PRODUCT_ISR			5
#define CAM_INT_TYPE_TIMEOUT				6
#define CAM_INT_TYPE_EXIT					7

#define D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE	40

struct sh_maincam_event_ctrl_cmd {
	unsigned int	Id;
	unsigned int	Param1;
	unsigned int	Param2;
	unsigned int	Param3;
	unsigned int	Param4;
	unsigned int	State;
	unsigned int	QueNo;
};

struct sh_maincam_shdiag_production_info_cmd{
	unsigned long	mode;
	void			*value;
};

struct sh_maincam_ctrl_cmd {
	unsigned char									r_data[D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE];
	unsigned char									w_data[D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE];
	signed int										status;
	signed int										length;
	unsigned short									addr;
	unsigned short									irq_number;
	unsigned char									device_id[5];
	unsigned char									dummy[3];
	struct sh_maincam_event_ctrl_cmd				event_data;
	struct sh_maincam_shdiag_production_info_cmd	product_data;
};

#define SH_MAINCAM_GPIO_CAM_INT			0
#define SH_MAINCAM_GPIO_CAM_VSYNC		1
#define SH_MAINCAM_GPIO_CAM_DATA_0		4
#define SH_MAINCAM_GPIO_CAM_DATA_1		5
#define SH_MAINCAM_GPIO_CAM_DATA_2		6
#define SH_MAINCAM_GPIO_CAM_DATA_3		7
#define SH_MAINCAM_GPIO_CAM_DATA_4		8
#define SH_MAINCAM_GPIO_CAM_DATA_5		9
#define SH_MAINCAM_GPIO_CAM_DATA_6		10
#define SH_MAINCAM_GPIO_CAM_DATA_7		11
#define SH_MAINCAM_GPIO_CAM_PCLK		12
#define SH_MAINCAM_GPIO_CAM_HSYNC		13
#define SH_MAINCAM_GPIO_CAM_VSYNC_B		14
#define SH_MAINCAM_GPIO_CAM_MCLK		15
#define SH_MAINCAM_GPIO_CAM_VCAM14_EN	16

#define CAM_GPIO_LO (0)
#define CAM_GPIO_HI (1)


#endif
