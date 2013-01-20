/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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
 * INCLUDE FILES
 */
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/pgtable.h>

#include <linux/sysfs.h>

/*
 * TYPE
 */
enum {
	SHSD_IOCTL_SET_POWER_ON_OFF=0,
	SHSD_IOCTL_INSERT_REMOVE_NOTIFY,
	SHSD_IOCTL_END
};

/*
 * VARIABLES
 */
static dev_t shsd_dev;
static dev_t shsd_major = 0;
static dev_t shsd_minor = 0;
static struct cdev shsd_cdev;
static struct class *shsd_class;
static unsigned long shsd_hcs1_addr = 0, shsd_hcs2_addr = 0;

/*
 * PROTOTYPES
 */
static int shsd_open(struct inode *inode, struct file *filp);
static int shsd_mmap(struct file *filp, struct vm_area_struct *vma);
static int shsd_release(struct inode *inode, struct file *filp);
static int shsd_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg);
static int shsd_ioctl_insert_remove_notify(void __user *argp);

static struct file_operations shsd_fops = {
    .owner   = THIS_MODULE,
    .open    = shsd_open,
    .mmap    = shsd_mmap,
    .ioctl   = shsd_ioctl,
    .release = shsd_release,
};

/*
 * FUNCTIONS
 */

/*
 * shsd_open
 */
static int shsd_open(struct inode *inode, struct file *filp)
{
    return 0;
}

/*
 * shsd_mmap
 */
static int shsd_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if(vma->vm_pgoff == (shsd_hcs1_addr >> PAGE_SHIFT)){
    	ret = io_remap_pfn_range(vma, vma->vm_start,
        	               shsd_hcs1_addr >> PAGE_SHIFT,
            	           vma->vm_end - vma->vm_start, vma->vm_page_prot);
    }
    else if(vma->vm_pgoff == (shsd_hcs2_addr >> PAGE_SHIFT)){
    	ret = io_remap_pfn_range(vma, vma->vm_start,
        	               shsd_hcs2_addr >> PAGE_SHIFT,
            	           vma->vm_end - vma->vm_start, vma->vm_page_prot);	
	}
	else{
		printk(KERN_ALERT"SHSD KERNEL: %s ERROR\n", __func__);
		return -1;
	}

    return 0;
}

/*
 * shsd_ioctl
 */
static int shsd_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
	case SHSD_IOCTL_INSERT_REMOVE_NOTIFY:
		ret = shsd_ioctl_insert_remove_notify(argp);
		break;
		
	default:
        ret = -EFAULT;
        break;
	}

    return ret;
}

/*==============================================================================
    Fanctions
==============================================================================*/
extern void stheno_add_card(void);
extern void stheno_remove_card(void);

static int shsd_ioctl_insert_remove_notify(void __user *argp)
{
    int ret, sw;

    ret = copy_from_user(&sw, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if (sw == 0) {
		/* remove */
		stheno_remove_card();
		
    } else {
		/* insert */
		stheno_add_card();
    }

    return 0;
}

/*
 * shsd_release
 */
static int shsd_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/*
 * shsd_init
 */
static int __init shsd_init(void)
{
    int ret;

	shsd_hcs1_addr = 0x60000000;
	shsd_hcs2_addr = 0x94000000;
    ret = alloc_chrdev_region(&shsd_dev, 0, 1, "shsd");

    if (!ret) {
        shsd_major = MAJOR(shsd_dev);
        shsd_minor = MINOR(shsd_dev);
    } else {
        goto shsd_err_1;
    }

    cdev_init(&shsd_cdev, &shsd_fops);

    shsd_cdev.owner = THIS_MODULE;
    shsd_cdev.ops = &shsd_fops;

    ret = cdev_add(&shsd_cdev, shsd_dev, 1);

    if (ret) {
        goto shsd_err_2;
    }

    shsd_class = class_create(THIS_MODULE, "shsd");

    if (IS_ERR(shsd_class)) {
        goto shsd_err_2;
    }

    device_create(shsd_class, NULL,
                  shsd_dev, &shsd_cdev, "shsd");

    return 0;

shsd_err_2:
    cdev_del(&shsd_cdev);

shsd_err_1:
    return -1;
}
module_init(shsd_init);

/*
 * shsd_exit
 */
static void shsd_exit(void)
{
    device_destroy(shsd_class, shsd_dev);
    class_destroy(shsd_class);
    cdev_del(&shsd_cdev);

    return;
}
module_exit(shsd_exit);

MODULE_DESCRIPTION("SHARP SD DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
