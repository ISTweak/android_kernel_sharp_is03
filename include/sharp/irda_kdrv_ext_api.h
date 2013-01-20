/* include/sharp/irda_kdrv_ext_api.h (IrDA SIR/FIR driver module)
 *
 * Copyright (C) 2009-2010 SHARP CORPORATION All rights reserved.
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

#ifndef _IRDA_KDRV_EXT_API_H_
#define _IRDA_KDRV_EXT_API_H_

#define IRDA_KDRV_EXT_API_SUCCESS	(0)
#define IRDA_KDRV_EXT_API_ERROR		(-1)

extern int irda_kdrv_irsd_active(void);
extern int irda_kdrv_irsd_shutdown(void);

#endif
