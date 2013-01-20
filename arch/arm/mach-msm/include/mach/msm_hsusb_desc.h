/* linux/include/mach/msm_hsusb_desc.h  (usb descriptor definition)
 *
 * Copyright (c) 2010 Sharp Corporation.
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

#ifndef __ASM_ARCH_MSM_HSUSB_DESC_H
#define __ASM_ARCH_MSM_HSUSB_DESC_H


#define USB_PRODUCT_NAME			"au IS03"

#ifdef CONFIG_SH_USB_CUST
#define USB_MSC_VENDOR_ID_LEN	(8)
#define USB_MSC_VENDOR_ID		"SHARP   "
#define USB_MSC_PRODUCT_ID_LEN	(16)
#define USB_MSC_PRODUCT_ID		"au IS03 microSD "
#endif

#define USB_OBEX_STRING_DESC_WORD	"au IS03 OBEX Port"
#define USB_MDLM_STRING_DESC_WORD	"~au IS03 High Speed Serial Port"
#define USB_GS_STRING_DESC_WORD		"au IS03"
#define USB_ADB_STRING_DESC_WORD	"Android ADB Interface"
#define USB_QXDM_STRING_DESC_WORD	"Qualcomm Port"
#define USB_MSC_STRING_DESC_WORD	"au IS03 SD Storage"
#define USB_MTP_STRING_DESC_WORD	"au IS03 MTP Port"

#define USB_PID_MODE1				0x939C
#define USB_PID_MODE2				0x939B
#define USB_PID_MODE3				0x939A
#define USB_PID_MODE4				0x933A

#define USB_CDC_GUID				0xC2,0x29,0x9F,0xCC,0xD4,0x89,0x40,0x66,\
									0x89,0x2B,0x10,0xC3,0x41,0xDD,0x98,0xA9

#endif
