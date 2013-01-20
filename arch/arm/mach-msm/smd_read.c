/* arch\arm\mach-msm/smd_read.c  (smd_read.c)
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
#include <linux/fs.h>
#include <linux/poll.h>

#include <mach/sharp_smem.h>

#if 1
struct smem_comm_mode {
	unsigned short BootMode;
	unsigned long FlagData;
	unsigned char FirstBoot;
};

#endif

static int smd_mode_open(struct inode *inode, struct file *filp)
{
    printk("%s\n", __func__);
    return 0;
}


static ssize_t smd_mode_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    struct smem_comm_mode  smem_comm_data;

    printk("%s\n", __func__);
    
    p_sh_smem_common_type = sh_smem_get_common_address();
    if( p_sh_smem_common_type != NULL){
        smem_comm_data.BootMode  = p_sh_smem_common_type->read_BootMode;
        smem_comm_data.FlagData  = p_sh_smem_common_type->read_FlagData;
        smem_comm_data.FirstBoot = p_sh_smem_common_type->read_FirstBoot;

        if( copy_to_user( buf, (void *)&smem_comm_data, sizeof(smem_comm_data) ) ){
            printk( "copy_to_user failed\n" );
            return -EFAULT;
        }
    } else {
        printk("[SH_DIAG]smd_read_probe: smem_alloc FAILE\n");
    }
    return count;
}

static ssize_t smd_mode_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
    printk("%s\n", __func__);
    return count;
}

static int smd_mode_release(struct inode *inode, struct file *filp)
{
    printk("%s\n", __func__);
    return 0;
}

static struct file_operations smd_mode_fops = {
    .owner      = THIS_MODULE,
    .read       = smd_mode_read,
    .write      = smd_mode_write,
    .open       = smd_mode_open,
    .release    = smd_mode_release,
};

static struct miscdevice smd_mode_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "smd_read",
    .fops = &smd_mode_fops,
};

static int __init smd_mode_init( void )
{
    int ret;

    ret = misc_register(&smd_mode_dev);
    if (ret) {
        printk("fail to misc_register (smd_mode_dev)\n");
        return ret;
    }
    printk("smd_mode loaded.\n");
    return 0;
}

module_init(smd_mode_init);

MODULE_DESCRIPTION("smd_read");
MODULE_LICENSE("GPL v2");

