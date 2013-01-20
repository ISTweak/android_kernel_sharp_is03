/* drivers/sharp/irda/ir_kdrv_common.h (IrDA SIR/FIR driver module)
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

#ifndef _IRDA_KDRV_COMMON_H
#define _IRDA_KDRV_COMMON_H

#include <linux/platform_device.h>
extern struct platform_device *irda_kdrv_pdev;

#define	SH_IRDA_KERNEL_DRIVER_VERSION	"1.03.11a"




#define IRLOG_PRINTK_ALERT	KERN_ALERT
#define IRLOG_PRINTK_ERR	KERN_ERR
#define IRLOG_PRINTK_WARNING	KERN_WARNING

#define IRLOG_PRINTK_INFO	KERN_INFO

#define IRLOG_IRDRV	"IRDA_DRV"
#define IRLOG_IRREG	"IR_REG"

#define IRLOG_FATAL	"F"
#define IRLOG_ERR	"E"
#define IRLOG_HIGH	"H"
#define IRLOG_MED	"M"
#define IRLOG_LOW	"L"

#define	IRLOG_IN	"[IN]"
#define	IRLOG_OUT	"[OUT]"
#define	IRLOG_INFO	"[INFO]"

#define	IRLOG_MSG_FORM(module, level)	"["module":"level"][%s][%d]"

#define IRLOG_MSG_MED_FORM(module, info)	\
	IRLOG_MSG_FORM(module,IRLOG_MED)info


#define MSG_IRDRV_FATAL(format, arg...)		\
		dev_alert(&irda_kdrv_pdev->dev,	\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_FATAL) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRDRV_ERR(format, arg...)		\
		dev_err(&irda_kdrv_pdev->dev,	\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_ERR) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRDRV_HIGH(format, arg...)		\
		dev_notice(&irda_kdrv_pdev->dev,\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_HIGH) format, \
		       __func__, __LINE__, ## arg)

#define MSG_IRDRV_MED_IN(format, arg...)
#define MSG_IRDRV_MED_OUT(format, arg...)
#define MSG_IRDRV_MED_INFO(format, arg...)
#define MSG_IRDRV_LOW(format, arg...)



#define MSG_IRREG_FATAL(format, arg...)		\
		dev_alert(&irda_kdrv_pdev->dev,	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_FATAL) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRREG_ERR(format, arg...)		\
		dev_err(&irda_kdrv_pdev->dev,	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_ERR) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRREG_HIGH(format, arg...)		\
		dev_notice(&irda_kdrv_pdev->dev,\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_HIGH) format, \
		       __func__, __LINE__, ## arg)

#define MSG_IRREG_MED_IN(format, arg...)
#define MSG_IRREG_MED_OUT(format, arg...)
#define MSG_IRREG_MED_INFO(format, arg...)
#define MSG_IRREG_LOW(format, arg...)



#define MSG_IRMOD_FATAL(format, arg...)		\
		printk(IRLOG_PRINTK_ALERT	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_FATAL) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRMOD_ERR(format, arg...)		\
		printk(IRLOG_PRINTK_ERR		\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_ERR) format, \
		       __func__, __LINE__, ## arg)
#define MSG_IRMOD_HIGH(format, arg...)		\
		printk(IRLOG_PRINTK_WARNING	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_HIGH) format, \
		       __func__, __LINE__, ## arg)

#define MSG_IRMOD_MED_IN(format, arg...)
#define MSG_IRMOD_MED_OUT(format, arg...)
#define MSG_IRMOD_MED_INFO(format, arg...)
#define MSG_IRMOD_LOW(format, arg...)


#endif
