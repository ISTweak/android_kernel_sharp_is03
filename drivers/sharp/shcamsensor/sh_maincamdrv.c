/* drivers/sharp/shcamsensor/sh_maincamdrv.c  (Camera Driver)
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>

#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/io.h>

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <media/msm_camera.h>
#include <mach/camera.h>

#include "sh_maincamdrv.h"
#include <linux/mfd/tps65023.h>

#include <linux/mm.h>
#include <linux/io.h>
#include <asm/pgtable.h>

#define SH_MAINCAM_MINORNUM_BASE	0
#define SH_MAINCAM_DEVICE_COUNT		(1)
#define SH_MAINCAM_DEVICE_NAME		"shmaincam"
#define SH_MAINCAM_NODE_NAME		"shmaincam"
#define SH_MAINCAM_CLASS_NAME		"cls_shmaincam"
#define SH_MAINCAM_IRQ_NAME			"irq_shmaincam"

#define SH_MAINCAM_HCS1_ADDR		0x60000000
#define SH_MAINCAM_HCS1_SIZE		0x1000
#define SH_MAINCAM_HCS2_ADDR		0x94000000
#define SH_MAINCAM_HCS2_SIZE		0x1000

static struct class  *sh_maincam_class;
static dev_t          sh_maincam_devno;
static struct device *sh_maincam_class_dev;

static void __iomem *sh_maincam_hcs1 = 0x00000000;
static void __iomem *sh_maincam_hcs2 = 0x00000000;

static int shmcamdrv_irq_port1 = 0;
static int shmcamdrv_irq_port2 = 0;

static int sh_maincamera_node = 0;

struct sh_maincam_sync_head_t {
	struct file			*file;
	struct cdev			cdev;
	spinlock_t			irq_event_q_lock;
	struct list_head	irq_event_q;
	uint32				irq_event_count;
	wait_queue_head_t	irq_event_wait;
	struct wake_lock	wake_lock;
	struct mutex		mut_lock;
};
static struct sh_maincam_sync_head_t *sh_maincam_sync_head = 0;

struct sh_maincam_thread_ctl_t {
	struct list_head		thread_q;
	wait_queue_head_t		thread_wait;
};
static struct sh_maincam_thread_ctl_t *sh_maincam_thread_ctl = 0;

struct sh_maincam_irq_msg_t {
	struct list_head			list;
	uint16						irq_number;
	struct sh_maincam_ctrl_cmd	ctrl_data;
};

static irqreturn_t sh_maincam_drv_gpio_vsync_isr( int irq_num, void *data );
static irqreturn_t sh_maincam_drv_gpio_lint3_isr( int irq_num, void *data );
static int sh_maincam_drv_event_isr( void __user *argp );
static int sh_maincam_drv_frame_event_isr( void __user *argp );
static int sh_maincam_drv_production_isr( void __user *argp );
static int sh_maincam_drv_get_isr_num( void __user *argp );
static int sh_maincam_drv_timeout_irq_num( void __user *argp );
static int sh_maincam_drv_enable_irq( void __user *argp );
static int sh_maincam_drv_disable_irq( void __user *argp );
static int sh_maincam_drv_camif_pad_reg_reset( void __user *argp );
static int sh_maincam_drv_request_irq( void __user *argp );
static int sh_maincam_drv_free_irq( void __user *argp );
static int sh_maincam_drv_camint_exit_isr( void __user *argp );
static int sh_maincam_drv_sensor_init( void );
static int sh_maincam_drv_sensor_off( void );
static int sh_maincam_sys_open(struct inode *inode, struct file *filep);
static int sh_maincam_sys_release(struct inode *inode, struct file *filep);
static long sh_maincam_sys_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static unsigned int sh_maincam_sys_poll(struct file *filep, struct poll_table_struct *pll_table);
static int sh_maincam_sys_mmap(struct file *filep, struct vm_area_struct *vma);
static int sh_maincam_drv_sync_init( struct sh_maincam_sync_head_t *sync );
static int sh_maincam_drv_sync_thread_ctl_init( struct sh_maincam_thread_ctl_t *sync );
static int sh_maincam_drv_sync_destroy( struct sh_maincam_sync_head_t *sync );
static void sh_maincam_drv_irq_msg_free( struct sh_maincam_sync_head_t *sync );
static void sh_maincam_drv_thread_ctl_free( struct sh_maincam_thread_ctl_t *sync );
static int __init sh_maincam_drv_start( void );
static void __exit sh_maincam_drv_remove( void );
static int sh_maincam_drv_access_gpio( void __user *argp );
static int sh_maincam_thread_control( void );
static int sh_maincam_isr_kind( void __user *argp );
static int sh_maincam_drv_set_debug_log( void __user *argp );


static irqreturn_t sh_maincam_drv_gpio_vsync_isr( int irq_num, void *data )
{
	irqreturn_t rc = IRQ_HANDLED;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;

	do{

		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_VS;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up( &sh_maincam_sync_head->irq_event_wait );

	}while(0);

	return rc;
}

static irqreturn_t sh_maincam_drv_gpio_lint3_isr( int irq_num, void *data )
{
	irqreturn_t rc = IRQ_HANDLED;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;

	do{

		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_INT;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up( &sh_maincam_sync_head->irq_event_wait );

	}while(0);

	return rc;
}

static int sh_maincam_drv_event_isr( void __user *argp )
{
	int rc = 0;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;
	struct sh_maincam_event_ctrl_cmd param;

	do{
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_event_ctrl_cmd))) {
			rc = -EFAULT;
			break;
		}


		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_EVENT_ISR;

		qmsg->ctrl_data.event_data = (struct sh_maincam_event_ctrl_cmd)param;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up_interruptible( &sh_maincam_sync_head->irq_event_wait );

		rc = sh_maincam_thread_control();

	}while(0);

	return rc;
}

static int sh_maincam_drv_frame_event_isr( void __user *argp )
{
	int rc = 0;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;
	struct sh_maincam_event_ctrl_cmd param;

	do{
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_event_ctrl_cmd))) {
			rc = -EFAULT;
			break;
		}


		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_FRAMEEVENT_ISR;

		qmsg->ctrl_data.event_data = (struct sh_maincam_event_ctrl_cmd)param;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up_interruptible( &sh_maincam_sync_head->irq_event_wait );

		rc = sh_maincam_thread_control();

	}while(0);

	return rc;
}

static int sh_maincam_drv_production_isr( void __user *argp )
{
	int rc = 0;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;
	struct sh_maincam_shdiag_production_info_cmd param;

	do{
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_shdiag_production_info_cmd))) {
			rc = -EFAULT;
			break;
		}


		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_PRODUCT_ISR;

		qmsg->ctrl_data.product_data = (struct sh_maincam_shdiag_production_info_cmd)param;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up_interruptible( &sh_maincam_sync_head->irq_event_wait );

		rc = sh_maincam_thread_control();

	}while(0);

	return rc;
}

static int sh_maincam_drv_get_isr_num( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;
		param.irq_number  = 0;

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if ( !list_empty( &sh_maincam_sync_head->irq_event_q ) ){
			qmsg = list_first_entry(&sh_maincam_sync_head->irq_event_q, struct sh_maincam_irq_msg_t, list);
			list_del(&qmsg->list);
			if( (qmsg->irq_number == CAM_INT_TYPE_EVENT_ISR) ||
				(qmsg->irq_number == CAM_INT_TYPE_FRAMEEVENT_ISR) ){
				param.event_data = (struct sh_maincam_event_ctrl_cmd)qmsg->ctrl_data.event_data;
			}
			else if( qmsg->irq_number == CAM_INT_TYPE_PRODUCT_ISR ){
				param.product_data = (struct sh_maincam_shdiag_production_info_cmd)qmsg->ctrl_data.product_data;
			}
			else{
				;
			}
			param.irq_number = qmsg->irq_number;
			sh_maincam_sync_head->irq_event_count--;
			kfree(qmsg);
		}
		else{
			param.status = -1;
		}

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_maincam_drv_timeout_irq_num( void __user *argp )
{
	int ret = -ETIMEDOUT;
	struct sh_maincam_ctrl_cmd param;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;
		param.irq_number  = (uint16)CAM_INT_TYPE_TIMEOUT;

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_maincam_drv_enable_irq( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;
	unsigned long flags = 0;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

		enable_irq(shmcamdrv_irq_port1);
		enable_irq(shmcamdrv_irq_port2);

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_maincam_drv_disable_irq( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;
	unsigned long flags = 0;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

		disable_irq(shmcamdrv_irq_port1);
		disable_irq(shmcamdrv_irq_port2);

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_maincam_drv_camif_pad_reg_reset( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		msm_camio_camif_pad_reg_reset();
		udelay(1500);

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_maincam_drv_request_irq( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;
	unsigned long flags = 0;

	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

		shmcamdrv_irq_port1 = MSM_GPIO_TO_INT( SH_MAINCAM_GPIO_CAM_INT );
		shmcamdrv_irq_port2 = MSM_GPIO_TO_INT( SH_MAINCAM_GPIO_CAM_VSYNC );
		ret = request_irq( shmcamdrv_irq_port1, sh_maincam_drv_gpio_lint3_isr, IRQF_TRIGGER_RISING,  "sh_maincam_drv_lint3", 0 );
		ret = request_irq( shmcamdrv_irq_port2, sh_maincam_drv_gpio_vsync_isr, IRQF_TRIGGER_FALLING, "sh_maincam_drv_vsync", 0 );

		disable_irq(shmcamdrv_irq_port1);
		disable_irq(shmcamdrv_irq_port2);
		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}

static int sh_maincam_drv_free_irq( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;
	unsigned long flags = 0;

	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

		free_irq( shmcamdrv_irq_port1, 0 );
		free_irq( shmcamdrv_irq_port2, 0 );

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}

static int sh_maincam_drv_camint_exit_isr( void __user *argp )
{
	int rc = 0;
	unsigned long flags = 0;
	struct sh_maincam_irq_msg_t *qmsg = NULL;

	do{

		qmsg = kzalloc( sizeof(struct sh_maincam_irq_msg_t ),GFP_ATOMIC );
		if ( !qmsg ) {
			break;
		}

		spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags);

		qmsg->irq_number = (uint16)CAM_INT_TYPE_EXIT;

		list_add_tail( &qmsg->list, &sh_maincam_sync_head->irq_event_q );

		sh_maincam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

		wake_up_interruptible( &sh_maincam_sync_head->irq_event_wait );

	}while(0);

	return rc;
}

static int sh_maincam_drv_access_gpio( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;

	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		if( param.addr == SH_MAINCAM_GPIO_CAM_MCLK ){
			if( param.w_data[0] == CAM_GPIO_HI ){
				msm_camio_clk_rate_set(65*100*1000);
				mdelay(2);
				gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_MCLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA), GPIO_ENABLE );
			}
			else{
				gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_MCLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_DISABLE );
			}
		}
		else{
			gpio_direction_output(param.addr, param.w_data[0]);
		}
	}while(0);

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}

static int sh_maincam_drv_sensor_init( void )
{
	int ret = 0;

	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_INT,			0, GPIO_INPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VSYNC,		0, GPIO_INPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_0,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_1,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_2,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_3,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_4,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_5,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_6,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_7,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_PCLK,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_HSYNC,		1, GPIO_INPUT,	GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VSYNC_B,		1, GPIO_INPUT,	GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_MCLK,		0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VCAM14_EN,	0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_ENABLE );

	mdelay(1);

	return ret;

}

static int sh_maincam_drv_sensor_off( void )
{
	int ret = 0;

	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_INT,			0, GPIO_INPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VSYNC,		0, GPIO_INPUT,	GPIO_PULL_UP,	GPIO_2MA), GPIO_DISABLE );

	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_0,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_1,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_2,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_3,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_4,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_5,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_6,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_DATA_7,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_PCLK,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_16MA),GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_HSYNC,		1, GPIO_INPUT,	GPIO_PULL_UP, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VSYNC_B,		1, GPIO_INPUT,	GPIO_PULL_UP, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_MCLK,		0, GPIO_OUTPUT, GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_MAINCAM_GPIO_CAM_VCAM14_EN,	0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );

	return ret;
}

static int sh_maincam_sys_open(struct inode *inode, struct file *filep)
{
	int ret = 0;

	wake_lock( &sh_maincam_sync_head->wake_lock );
	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		sh_maincam_hcs1 =  ioremap_nocache(SH_MAINCAM_HCS1_ADDR, SH_MAINCAM_HCS1_SIZE);
		sh_maincam_hcs2 =  ioremap_nocache(SH_MAINCAM_HCS2_ADDR, SH_MAINCAM_HCS2_SIZE);

		if (likely(sh_maincam_hcs1 != NULL)) {
		} else {
			ret = -ENOMEM;
			break;
		}
		if (likely(sh_maincam_hcs2 != NULL)) {
		} else {
			iounmap( sh_maincam_hcs1 );
			ret = -ENOMEM;
			break;
		}

		ret = sh_maincam_drv_sensor_init();

	}while(0);

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}


static int sh_maincam_sys_release(struct inode *inode, struct file *filep)
{
	int ret = 0;

	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		ret = sh_maincam_drv_sensor_off();
		if ( ret < 0 ) {
			break;
		}

		sh_maincam_drv_irq_msg_free( sh_maincam_sync_head );
		sh_maincam_drv_thread_ctl_free( sh_maincam_thread_ctl );
		wake_unlock( &sh_maincam_sync_head->wake_lock );

	}while(0);

	iounmap( sh_maincam_hcs1 );
	iounmap( sh_maincam_hcs2 );

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}

static int sh_maincam_isr_kind( void __user *argp )
{
	int ret = 0;

	if( list_empty( &sh_maincam_sync_head->irq_event_q ) ){
		wake_up_interruptible( &sh_maincam_thread_ctl->thread_wait );
	}

	ret = wait_event_interruptible_timeout(
				sh_maincam_sync_head->irq_event_wait,
				!list_empty( &sh_maincam_sync_head->irq_event_q ) ,
				msecs_to_jiffies(10000));
	if(ret == 0 ){
		ret = sh_maincam_drv_timeout_irq_num( argp );
	}else{
		ret = sh_maincam_drv_get_isr_num( argp );
	}

	return ret;
}

static int sh_maincam_thread_control( void )
{
	int ret = 0;

	ret = wait_event_interruptible_timeout(
				sh_maincam_thread_ctl->thread_wait,
				list_empty( &sh_maincam_sync_head->irq_event_q ) ,
				msecs_to_jiffies(10000));

	if(ret == 0 ){
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int sh_maincam_drv_set_debug_log( void __user *argp )
{
	int ret = 0;
	struct sh_maincam_ctrl_cmd param;

	mutex_lock( &sh_maincam_sync_head->mut_lock );

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_maincam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	mutex_unlock( &sh_maincam_sync_head->mut_lock );

	return ret;
}

static long sh_maincam_sys_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	do {
		switch (cmd) {
			case SH_MAINCAM_IOCTL_READ_IRQ_KIND:
				ret = sh_maincam_isr_kind( argp );
				break;
			case SH_MAINCAM_IOCTL_ENABLE_IRQ:
				ret = sh_maincam_drv_enable_irq( argp );
				break;
			case SH_MAINCAM_IOCTL_DISABLE_IRQ:
				ret = sh_maincam_drv_disable_irq( argp );
				break;
			case SH_MAINCAM_IOCTL_CAMIF_PAD_RESET:
				ret = sh_maincam_drv_camif_pad_reg_reset( argp );
				break;
			case SH_MAINCAM_IOCTL_REQUEST_IRQ:
				ret = sh_maincam_drv_request_irq( argp );
				break;
			case SH_MAINCAM_IOCTL_FREE_IRQ:
				ret = sh_maincam_drv_free_irq( argp );
				break;
			case SH_MAINCAM_IOCTL_GPIO_CTRL:
				ret = sh_maincam_drv_access_gpio( argp );
				break;
			case SH_MAINCAM_IOCTL_EVENT_REQUEST_ISR:
				ret =  sh_maincam_drv_event_isr( argp );
				break;
			case SH_MAINCAM_IOCTL_FRAMEEVENT_REQUEST_ISR:
				ret  = sh_maincam_drv_frame_event_isr( argp );
				break;
			case SH_MAINCAM_IOCTL_PRODUCT_REQUEST_ISR:
				ret  = sh_maincam_drv_production_isr( argp );
				break;
			case SH_MAINCAM_IOCTL_SET_DEBUG_LOG:
				ret  = sh_maincam_drv_set_debug_log( argp );
				break;
			case SH_MAINCAM_IOCTL_CAMINT_EXIT:
				ret  = sh_maincam_drv_camint_exit_isr( argp );
				break;
			default:
				ret = -EFAULT;
				break;
		}
	}while(0);

	return ret;
}

static unsigned int sh_maincam_sys_poll(struct file *filep, struct poll_table_struct *pll_table)
{
	uint32 rc = POLLERR;
	unsigned long flags = 0;

	poll_wait(filep, &sh_maincam_sync_head->irq_event_wait, pll_table);

	spin_lock_irqsave( &sh_maincam_sync_head->irq_event_q_lock, flags );

	if ( !list_empty( &sh_maincam_sync_head->irq_event_q ) ){
		rc = POLLIN | POLLRDNORM;
	}
	else{
	}

	spin_unlock_irqrestore( &sh_maincam_sync_head->irq_event_q_lock, flags );

	return rc;
}

static int sh_maincam_sys_mmap(struct file *filep, struct vm_area_struct *vma)
{
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if(io_remap_pfn_range(vma, vma->vm_start,
						 (unsigned long)SH_MAINCAM_HCS1_ADDR >> PAGE_SHIFT,
						 SH_MAINCAM_HCS1_SIZE,
						 vma->vm_page_prot))
	{
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations sh_maincam_fops = {
	.owner          = THIS_MODULE,
	.open           = sh_maincam_sys_open,
	.unlocked_ioctl = sh_maincam_sys_ioctl,
	.release        = sh_maincam_sys_release,
	.read           = NULL,
	.write          = NULL,
	.poll           = sh_maincam_sys_poll,
	.mmap           = sh_maincam_sys_mmap,
};


static int sh_maincam_drv_sync_init( struct sh_maincam_sync_head_t *sync )
{
	int rc = 0;

	spin_lock_init( &sync->irq_event_q_lock );
	INIT_LIST_HEAD( &sync->irq_event_q );
	init_waitqueue_head( &sync->irq_event_wait );
	wake_lock_init( &sync->wake_lock, WAKE_LOCK_SUSPEND, SH_MAINCAM_DEVICE_NAME );
	sync->irq_event_count = 0;
	mutex_init( &sync->mut_lock );

	return rc;
}

static int sh_maincam_drv_sync_thread_ctl_init( struct sh_maincam_thread_ctl_t *sync )
{
	int rc = 0;

	INIT_LIST_HEAD( &sync->thread_q );
	init_waitqueue_head( &sync->thread_wait );

	return rc;
}


static int sh_maincam_drv_sync_destroy( struct sh_maincam_sync_head_t *sync )
{
	wake_lock_destroy( &sync->wake_lock );

	return 0;
}

static void sh_maincam_drv_irq_msg_free( struct sh_maincam_sync_head_t *sync )
{
	struct sh_maincam_irq_msg_t *qmsg = NULL;

	for (;;){
		if (!list_empty(&sync->irq_event_q)) {
			qmsg = list_first_entry(&sync->irq_event_q, struct sh_maincam_irq_msg_t, list);
			list_del(&qmsg->list);
			sync->irq_event_count--;
			kfree(qmsg);
		}
		else{
			break;
		}
	}

	return;
}

static void sh_maincam_drv_thread_ctl_free( struct sh_maincam_thread_ctl_t *sync )
{
	struct sh_maincam_thread_ctl_t *qmsg = NULL;

	for (;;){
		if (!list_empty(&sync->thread_q)) {
			qmsg = list_first_entry(&sync->thread_q, struct sh_maincam_thread_ctl_t, thread_q);
			list_del(&qmsg->thread_q);
			kfree(qmsg);
		}
		else{
			break;
		}
	}

	return;
}

static int __init sh_maincam_drv_start( void )
{
	int rc = -ENODEV;
	int errfunc = 0;

	do {
		if (sh_maincamera_node >= SH_MAINCAM_DEVICE_COUNT) {
			rc = -ENODEV;
			errfunc = 0;
			break;
		}

		if (!sh_maincam_class){
			sh_maincam_class = class_create(THIS_MODULE, SH_MAINCAM_CLASS_NAME);
			if( IS_ERR(sh_maincam_class)) {
				errfunc = 0;
				rc = PTR_ERR(sh_maincam_class);
				break;
			}

			rc = alloc_chrdev_region(&sh_maincam_devno,
									SH_MAINCAM_MINORNUM_BASE,
									SH_MAINCAM_DEVICE_COUNT,
									SH_MAINCAM_DEVICE_NAME);
			if ( rc < 0 ) {
				errfunc = 1;
				break;
			}
		}

		sh_maincam_sync_head = kzalloc( sizeof(struct sh_maincam_sync_head_t ) * SH_MAINCAM_DEVICE_COUNT,
									GFP_ATOMIC );
		if ( !sh_maincam_sync_head ) {
			rc = -ENOMEM;
			errfunc = 2;
			break;
		}

		sh_maincam_thread_ctl = kzalloc( sizeof(struct sh_maincam_thread_ctl_t ) * SH_MAINCAM_DEVICE_COUNT,
									GFP_ATOMIC );
		if ( !sh_maincam_thread_ctl ) {
			rc = -ENOMEM;
			errfunc = 3;
			break;
		}

		memset(sh_maincam_sync_head, 0x00, sizeof(struct sh_maincam_sync_head_t ) * SH_MAINCAM_DEVICE_COUNT);
		rc = sh_maincam_drv_sync_init( sh_maincam_sync_head );
		if (rc < 0) {
			errfunc = 4;
			break;
		}

		memset(sh_maincam_thread_ctl, 0x00, sizeof(struct sh_maincam_thread_ctl_t ) * SH_MAINCAM_DEVICE_COUNT);
		rc = sh_maincam_drv_sync_thread_ctl_init( sh_maincam_thread_ctl );
		if (rc < 0) {
			errfunc = 5;
			break;
		}

		sh_maincam_class_dev = device_create( sh_maincam_class, NULL,
												MKDEV(MAJOR(sh_maincam_devno), SH_MAINCAM_MINORNUM_BASE ),
												NULL, SH_MAINCAM_NODE_NAME );
		if ( IS_ERR( sh_maincam_class_dev ) ){
			rc = PTR_ERR( sh_maincam_class_dev );
			errfunc = 6;
			break;
		}

		cdev_init(&sh_maincam_sync_head->cdev, &sh_maincam_fops);
		sh_maincam_sync_head->cdev.owner = THIS_MODULE;

		rc = cdev_add( &sh_maincam_sync_head->cdev, MKDEV(MAJOR(sh_maincam_devno),
								SH_MAINCAM_MINORNUM_BASE ), SH_MAINCAM_DEVICE_COUNT );
		if ( rc < 0 ) {
			errfunc = 7;
			break;
		}

		sh_maincamera_node++;
		rc = 0;
		errfunc = 0;

	}while(0);

	switch( errfunc ){
		case 8:
			cdev_del(&sh_maincam_sync_head->cdev);
		case 7:
			device_destroy( sh_maincam_class, sh_maincam_devno );
		case 6:
		case 5:
			sh_maincam_drv_sync_destroy( sh_maincam_sync_head );
		case 4:
			kfree( sh_maincam_thread_ctl );
			sh_maincam_thread_ctl = NULL;
		case 3:
			kfree( sh_maincam_sync_head );
			sh_maincam_sync_head = NULL;
		case 2:
			unregister_chrdev_region( sh_maincam_devno, SH_MAINCAM_DEVICE_COUNT );
		case 1:
			class_destroy( sh_maincam_class );
			sh_maincam_class = NULL;
		case 0:
		default:
			break;
	}

	return rc;
}



static void __exit sh_maincam_drv_remove( void )
{
	do {
		if ( sh_maincamera_node == 0 ) {
			break;
		}

		cdev_del(&sh_maincam_sync_head->cdev);

		device_destroy( sh_maincam_class, sh_maincam_devno );

		sh_maincam_drv_sync_destroy( sh_maincam_sync_head );

		kfree( sh_maincam_sync_head );
		sh_maincam_sync_head = NULL;

		kfree( sh_maincam_thread_ctl );
		sh_maincam_thread_ctl = NULL;

		unregister_chrdev_region( sh_maincam_devno, SH_MAINCAM_DEVICE_COUNT );

		class_destroy( sh_maincam_class );
		sh_maincam_class = NULL;

		sh_maincamera_node = 0;
	}while(0);

	return;
}


module_init(sh_maincam_drv_start);
module_exit(sh_maincam_drv_remove);

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");
