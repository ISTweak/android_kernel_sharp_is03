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
 
/*
 * Definitions for Y2659 chip.
 */
#ifndef Y2659_H
#define Y2659_H

#include <linux/ioctl.h>


struct sh_i2c_proximity_platform_data {
	int		gpio_irq;
	int		gpio_enable;
	int		(*gpio_setup) (void);
	void	(*gpio_shutdown)(void);
};
#define SH_PROXIMITY_I2C_DEVNAME	"SH_Y2659"
#define SH_PROXIMITY_I2C_SLAVE		0x64
#define SH_PROXIMITY_ENABLE			23
#define SH_PROXIMITY_IRQ			38


/* Y2659 register address */
#define Y2659_REG_PROX				0x00
#define Y2659_REG_GAIN				0x01
#define Y2659_REG_HYS				0x02
#define Y2659_REG_CYCLE				0x03
#define Y2659_REG_OPMOD				0x04
#define Y2659_REG_CON				0x06


/* IOCTLs for Y2659 */
#define Y2569IO						0xA2

#define ECS_IOCTL_ENABLE			_IO(Y2569IO, 0x01)
#define ECS_IOCTL_DISABLE			_IO(Y2569IO, 0x02)
#define ECS_IOCTL_SET_CYCLE			_IOW(Y2569IO, 0x03, short)
#define ECS_IOCTL_GET_VO_DATA		_IOR(Y2569IO, 0x04, char)


#endif /* Y2659_H */

