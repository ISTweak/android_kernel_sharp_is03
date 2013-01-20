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
 
#ifndef __RPC_SERVER_HANDSET_H
#define __RPC_SERVER_HANDSET_H

#define HEADPHONE_SW_IO				0xA3

/* IOCTLs for headphone sw */
#define HEADPHONE_SW_IOCTL_DETECT_START		_IO(HEADPHONE_SW_IO, 0x01)
#define HEADPHONE_SW_IOCTL_DETECT_STOP		_IO(HEADPHONE_SW_IO, 0x02)

#endif
