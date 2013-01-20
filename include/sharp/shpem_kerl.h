/*
 * Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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

#ifndef _SHPEM_KERL_H_
#define _SHPEM_KERL_H_

#include <linux/ioctl.h>

#define SHPEM_I2C_DEVNAME	"shpem"
#define SHPEM_I2C_SLAVE		0x1E

#define SHPEM_MISC_DEVNAME		"shpem_dev"
#define SHPEM_MISC_DEVNAME_ALL	"/dev/shpem_dev"

#define SHPEM_IO			0xA2

#define SHPEM_IOCTL_COM					_IO(SHPEM_IO, 0x01)
#define SHPEM_IOCTL_SET_SLEEP_STATE		_IO(SHPEM_IO, 0x02)
#define SHPEM_IOCTL_SET_SENSOR_STATE	_IO(SHPEM_IO, 0x03)
#define SHPEM_IOCTL_HINT				_IO(SHPEM_IO, 0x04)
#define SHPEM_IOCTL_GPIO_CTL			_IO(SHPEM_IO, 0x05)
#define SHPEM_IOCTL_I2C_WRITE			_IO(SHPEM_IO, 0x06)
#define SHPEM_IOCTL_I2C_READ			_IO(SHPEM_IO, 0x07)
#define SHPEM_IOCTL_FW_WRITE_START		_IO(SHPEM_IO, 0x08)
#define SHPEM_IOCTL_FW_WRITE_END		_IO(SHPEM_IO, 0x09)
#define SHPEM_IOCTL_RESET				_IO(SHPEM_IO, 0x0A)

#define SHPEM_ISR_BREAK_PORT		0x01
#define SHPEM_ISR_RESERVE1_PORT		0x02
#define SHPEM_ISR_RESERVE2_PORT		0x04
#define SHPEM_ISR_RESERVE3_PORT		0x08
#define SHPEM_ISR_RESERVE4_PORT		0x10
#define SHPEM_ISR_RESERVE5_PORT		0x20
#define SHPEM_ISR_RESERVE6_PORT		0x40
#define SHPEM_ISR_RESERVE7_PORT		0x80

typedef struct
{
	unsigned char *send_cmd;
	unsigned char *recv_ok;
	unsigned char *recv_ng;
} shpem_command_t;

typedef struct
{
	unsigned char* buf;
	int len;
} shpem_i2c_io_t;

extern const shpem_command_t shpem_command[];

#endif /* _SHPEM_KERL_H_ */
