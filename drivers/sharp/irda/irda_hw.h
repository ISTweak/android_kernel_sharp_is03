/* drivers/sharp/irda/irda_hw.h (IrDA SIR/FIR driver module)
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION All rights reserved.
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

#ifndef _IR_HW_H
#define	_IR_HW_H

#define	IRDA_DEVICE_NAME	"LR388G7"

#define	IRDA_CAPABILITY_SIR_BAUDRATE	\
		"SIR   :9600bps, 19200bps, 38.4Kbps, 57.6Kbps, 115.2Kbps"
#define	IRDA_CAPABILITY_FIR_BAUDRATE	"FIR   :4Mbps"
#define	IRDA_CAPABILITY_ABOF   "aBOF  :48, 32, 24, 20, 16, 14, 12, 10, 8, 6 - 0"
#define	IRDA_CAPABILITY_MPI		"MPI   :100us - 16msec"
#define	IRDA_CAPABILITY_MTT		"MinTAT:0ms, 0.5ms - 16ms"






#define IRDA_ADDR_HCS_SIZE	0x1000

#define IRDA_ADDR_HCS1		0x60000000
#define IRDA_ADDR_HCS1_SIZE	IRDA_ADDR_HCS_SIZE
#define IRDA_ADDR_HCS2		0x94000000
#define IRDA_ADDR_HCS2_SIZE	IRDA_ADDR_HCS_SIZE









#define IRDA_INT_GPIO_NO	(137)

#define	IR_QSD_LEDA_EN		147
#define	IR_QSD_IRRX_PU		149

#define	IR_QSD_GPIO_ON		(uint16)0x0001
#define	IR_QSD_GPIO_OFF		(uint16)0x0000


#endif
