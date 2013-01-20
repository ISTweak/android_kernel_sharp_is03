/* arch/arm/mach-msm/include/mach/sharp_smem.h
 *
 * Copyright (C) 2009 Sharp Corporation
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
 
#include <sharp/shdisp_kerl.h>

#include "shrlog_type.h"

typedef struct 
{
    unsigned long       sh_filesystem_init;     /* file system innitialize flag */
    unsigned long       sh_hw_revision;         /* hardware revision number */
    unsigned long       sh_model_type;          /* model type information */
    unsigned long       sh_boot_mode;           /* power up mode information */
    unsigned long       sh_softupdate_mode;     /* software update mode  */
    unsigned long       sh_pwr_on_status;       /* power on status information from pmic  */
    struct shdisp_boot_context shdisp_boot_ctx;
    unsigned long       read_FlagData;				/* flag information */
    unsigned short      read_BootMode;				/* Powerup mode */
    unsigned char       read_FirstBoot;				/* FirstBoot information */
    unsigned char       rec[3];                     /* reserved */
    unsigned char       shdiag_AdjChashe[16];		/* shdiag Adj chashe information */
    unsigned long       support_FlagData;			/* support flag information */
    int                 sh_sleep_test_mode;
    unsigned char       shsecure_PassPhrase[32];
    unsigned char       bootimg_header[2048];		/* APPSBOOT header information */
    shrlog_reset_info   smem_reset_info;			/* Reset Log information structure */
    unsigned char       sh_i2c_access_dev;			/* I2C access device */
    unsigned char       sh_i2c_first_err_dev;		/* I2C first error device*/
    unsigned char       sh_i2c_last_success_dev;	/* I2C final success device*/
    unsigned short      sh_i2c_err_counter;			/* I2C error counter */
    unsigned long       fota_boot_mode;           /* FOTA mode information */
    unsigned char       pImeiData[12];            /* W-CDMA Imei data  */
    unsigned char       sh_fwupdateFlag;			/* Update Information */
    unsigned char       sh_camver[4];				/* Version information */
    unsigned char       sh_touchver[4];			/* Version information */
    unsigned char       sh_miconver[4];			/* Version information */
} sharp_smem_common_type;

/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void );

