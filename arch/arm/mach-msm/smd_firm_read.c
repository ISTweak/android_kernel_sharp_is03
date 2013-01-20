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
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <mach/sharp_smem.h>

static ssize_t smd_firm_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	unsigned char  smem_data;

	p_sh_smem_common_type = sh_smem_get_common_address();
	if( p_sh_smem_common_type != NULL){
		smem_data  = p_sh_smem_common_type->sh_fwupdateFlag;

		if( copy_to_user( buf, (void *)&smem_data, sizeof(smem_data) ) ){
			printk( "copy_to_user failed\n" );
			return -EFAULT;
		}
	} else {
		printk("smd_shfirm_read: smem_alloc FAILE\n");
	}
	return count;
}

static struct file_operations smd_firm_fops = {
	.owner		= THIS_MODULE,
	.read		= smd_firm_read,
};

static struct miscdevice smd_firm_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smd_firm",
	.fops = &smd_firm_fops,
};

static int __init smd_firm_init( void )
{
	int ret;

	ret = misc_register(&smd_firm_dev);
	if (ret) {
		printk("fail to misc_register (smd_firm_dev)\n");
		return ret;
	}
	printk("smd_firm loaded.\n");
	return 0;
}

module_init(smd_firm_init);

MODULE_DESCRIPTION("smd_firm");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
