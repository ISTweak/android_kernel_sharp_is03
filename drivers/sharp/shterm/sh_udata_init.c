/* drivers/sharp/shterm/sh_udata_init.c
 *
 * Copyright (C) 2010 Sharp Corporation
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

#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <mach/sharp_smem.h>
#include <linux/wakelock.h>

#define FLAG_SIZE 4
#define GISFINT_F__MASK 0x00000300

static struct wake_lock lcdpc_wake_lock;

static int shudatainit_open( struct inode *inode, struct file *filp )
{
    wake_lock( &lcdpc_wake_lock );
    return 0;
}

static ssize_t shudatainit_read( struct file *filp, char __user *buf,
                              size_t count, loff_t *ppos )
{
    sharp_smem_common_type *ptr = NULL;
    unsigned long init_flag;
    char flag[FLAG_SIZE];
    int size;

    printk( "%s\n", __func__ );

    ptr = sh_smem_get_common_address();

    if( ptr == NULL ){
        count = 0;
        return -EFAULT;
    }

    init_flag = ptr->sh_filesystem_init;

    memset( flag, 0x00, sizeof(flag) );
    size = snprintf( flag, FLAG_SIZE, "%ld", init_flag );

    if( copy_to_user( buf, flag, size ) ){
        return -EFAULT;
    }

    return size;
}

static ssize_t shudatainit_write( struct file *filp, const char __user *buf,
                               size_t count, loff_t *ppos )
{
    return 0;
}

static int shudatainit_release( struct inode *inode, struct file *filp )
{
    return 0;
}

static struct file_operations shudatainit_fops = {
    .owner      = THIS_MODULE,
    .open       = shudatainit_open,
    .release    = shudatainit_release,
    .read       = shudatainit_read,
    .write      = shudatainit_write,
};

static struct miscdevice shudatainit_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shudatainit",
    .fops = &shudatainit_fops,
};

static int __init shudatainit_init( void )
{
    int ret;

    ret = misc_register( &shudatainit_dev );
    if( ret ){
        printk( "failure %s\n", __func__ );
        return ret;
    }
    wake_lock_init( &lcdpc_wake_lock, WAKE_LOCK_IDLE, "sh_udata_init" );
    return 0;
}

module_init(shudatainit_init);
MODULE_LICENSE("GPL");

