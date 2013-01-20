/* drivers/sharp/irda/irda_kdrv.c (IrDA SIR/FIR driver module)
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <mach/pmic.h>
#include <mach/gpio.h>
#include <mach/vreg.h>

#include <linux/platform_device.h>

#include "sharp/irda_common.h"

#include "ir_LR388G7.h"
#include "ir_kdrv_common.h"
#include "irda_hw.h"
#include "sharp/irda_kdrv_api.h"
#include "sharp/irda_memory.h"
#include "sharp/irda_kdrv_ext_api.h"

#include <linux/wakelock.h>


#define	MSG_IRDRV_MED_INFO_ISR(format, arg...)

#define	MSG_IRDRV_MED_INFO_ISR_RECV_CTRL(format, arg...)

#define	MSG_IRDRV_MED_INFO_IOCTL(format, arg...)	\
					MSG_IRDRV_MED_INFO(format, ## arg )

#define IRDA_KERNEL_API_SUCCESS	(0)
#define IRDA_KERNEL_API_ERROR	(-1)

#define IRDA_ISR_SUCCESS	(0)
#define IRDA_ISR_ERROR		(-1)

static int IrDA_kdrv_major = 0;
static struct cdev IrDA_kdrv_cdev;

#define IRDA_KDRV_DEVS		(1)
#define IRDA_KDRV_MINOR		(0)
#define IRDA_KDRV_BASE_MINOR	(0)
#define IRDA_KDRV_MINOR_COUNT	(1)

#define IRDA_KDRV_CLASS_NAME	"cls_shirda"
static struct class *IrDA_kdrv_class = NULL;
static dev_t IrDA_kdrv_dev;

typedef enum {
	IRDA_KDRV_STATUS_CLOSE,
	IRDA_KDRV_STATUS_OPEN,
	IRDA_KDRV_STATUS_SIR_RECV,
	IRDA_KDRV_STATUS_SIR_SEND,
	IRDA_KDRV_STATUS_FIR_RECV,
	IRDA_KDRV_STATUS_FIR_SEND,
	IRDA_KDRV_STATUS_CARRIER,
	IRDA_KDRV_STATUS_ERROR,

	IRDA_KDRV_STATUS_MAX
} irda_kdrv_state_enum;

typedef enum {
	IRDA_KDRV_EVENT_OPEN,
	IRDA_KDRV_EVENT_CLOSE,
	IRDA_KDRV_EVENT_MMAP,
	IRDA_KDRV_EVENT_INIT,
	IRDA_KDRV_EVENT_TERM,
	IRDA_KDRV_EVENT_GET_QOS,
	IRDA_KDRV_EVENT_SET_QOS,
	IRDA_KDRV_EVENT_CHK_RX,
	IRDA_KDRV_EVENT_CARRIER,
	IRDA_KDRV_EVENT_POLL,
	IRDA_KDRV_EVENT_WRITE,
	IRDA_KDRV_EVENT_READ,
	IRDA_KDRV_EVENT_READEND,

	IRDA_KDRV_EVENT_ISR_IRDA,
	IRDA_KDRV_EVENT_ISR_CARRIER,
	IRDA_KDRV_EVENT_MAX
} irda_kdrv_event_enum;

typedef enum {
	IRDA_ACT_OK,
	IRDA_ACT_NG,
	IRDA_ACT_MAX
} irda_kdrv_action_enum;

static irda_kdrv_action_enum
		irda_kdrv_action[IRDA_KDRV_STATUS_MAX][IRDA_KDRV_EVENT_MAX] = {
	{
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,

		IRDA_ACT_NG,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_NG,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_OK,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_OK,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_OK,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_OK,
		IRDA_ACT_NG
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_NG,
		IRDA_ACT_OK
	},
	{
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_NG,
		IRDA_ACT_OK,
		IRDA_ACT_OK,

		IRDA_ACT_NG,
		IRDA_ACT_NG
	}
};

typedef enum {
	IRDA_KDRV_LED_LOW_POWER,
	IRDA_KDRV_LED_HIGH_POWER
} irda_kdrv_led_power_enum;

typedef enum {
	IRDA_KDRV_LED_ST_NON,
	IRDA_KDRV_LED_ST_SIR,
	IRDA_KDRV_LED_ST_FIR
} irda_kdrv_led_state_enum;

typedef enum {
	IRDA_KDRV_MODE_STOP,
	IRDA_KDRV_MODE_RECV,
	IRDA_KDRV_MODE_SEND
} irda_kdrv_mode_enum;

typedef enum {
	IRDA_KDRV_SEND_NORMAL,
	IRDA_KDRV_SEND_CONTINUE,
	IRDA_KDRV_SEND_CONTINUE_LAST
} irda_kdrv_send_mode_enum;

typedef struct {
	irda_kdrv_state_enum state;
	irda_qos_info qos_info;
	irda_kdrv_led_state_enum led_st;
	irda_kdrv_send_mode_enum send_mode;
	int fifo_write_cnt;
	bool first_send;
	bool poll;
} irda_kdrv_control_struct;

static irda_kdrv_control_struct IrDA_kdrv;

#define IRDA_FIFO_WRITE_MAX	(8)

typedef enum {
	IRDA_FOPS_READ_CONTINUE,
	IRDA_FOPS_READ_END,
	IRDA_FOPS_READ_ERR_END
} irda_read_fops_control_enum;

#define IRDA_IRQ_NUM		MSM_GPIO_TO_INT(IRDA_INT_GPIO_NO)
#define IRDA_IRQ_NAME		"irq_shirda"

typedef enum {
	IRDA_KDRV_WU_EV_SEND_WAIT,
	IRDA_KDRV_WU_EV_ISR_CARRIER_EXPIRE,
	IRDA_KDRV_WU_EV_ISR_RECV,
	IRDA_KDRV_WU_EV_ISR_SEND,
	IRDA_KDRV_WU_EV_ISR_SEND_ERR,
	IRDA_KDRV_WU_EV_POWEROFF,
	IRDA_KDRV_WU_EV_ENUM_MAX
} irda_kdrv_wakeup_event_enum;

#define IRDA_KDRV_WU_EV_QUE_MAX		(9+1)

typedef struct {
	unsigned short			rp;
	unsigned short			wp;
	irda_kdrv_wakeup_event_enum	event[IRDA_KDRV_WU_EV_QUE_MAX];
}irda_kdrv_wakeup_que_struct;

static irda_kdrv_wakeup_que_struct IrDA_kdrv_wu_que;

static wait_queue_head_t IrDA_kdrv_read_wakeup;
static wait_queue_head_t IrDA_kdrv_poll_wakeup;

#define IRDA_SEND_TIME_EXP		(4)

#define IRDA_CARRIER_SENSE_TIME		(10*100)
#define IRDA_CARRIER_SENSE_CHK_CNT	(560*100/IRDA_CARRIER_SENSE_TIME)
static int IrDA_kdrv_carrier_cnt = 0;

static spinlock_t IrDA_kdrv_spin_lock;
static struct semaphore IrDA_kdrv_sem;

static void *IrDA_kdrv_mmap_p = NULL;
static void *IrDA_kmalloc_ptr = NULL;

static int irda_kdrv_memory_alloc(void);
static void irda_kdrv_memory_free(void);
#define IRDA_MEM_OK		(0)
#define IRDA_MEM_ERR_KMALLOC	(-1)
#define IRDA_MEM_ERR_IOREMAP	(-2)

#define IRDA_SIR_CRC_LEN	(2)
#define IRDA_FIR_CRC_LEN	(4)
#define IRDA_MPI_TIMEUNIT	(10)
#define IRDA_MTT_TIMEUNIT	(10)
#define IRDA_FREE_FIFO		(2080)

static void irda_kdrv_init_control(void);
static int irda_kdrv_irdacc_term_func(void);

struct platform_device *irda_kdrv_pdev = NULL;

static struct wake_lock shirda_wake_lock;
#define SHIRDA_WAKELOCK		"shirda_wakelock"


#define IRDA_HCS_SIZE	\
	((((IRDA_ADDR_HCS_SIZE - 1) / PAGE_SIZE) + 1) * PAGE_SIZE)

#define IRDA_MMAP_SIZE	\
	((((IRDA_MMAP_LENGTH - 1) / PAGE_SIZE) + 1) * PAGE_SIZE)






int irda_kdrv_irsd_active(void)
{
	int ret = IRDA_KDRV_EXT_API_SUCCESS;

	if (ir_reg_irsd_active() != IR_REG_RESULT_SUCCESS) {
		MSG_IRDRV_FATAL("irsd_active error\n");
		ret = IRDA_KDRV_EXT_API_ERROR;
	}
	return (ret);
}

int irda_kdrv_irsd_shutdown(void)
{
	int ret = IRDA_KDRV_EXT_API_SUCCESS;

	if (ir_reg_irsd_shutdown() != IR_REG_RESULT_SUCCESS) {
		MSG_IRDRV_FATAL("irsd_shutdown error\n");
		ret = IRDA_KDRV_EXT_API_ERROR;
	}
	return (ret);
}




void irda_kdrv_wakeup_read(irda_kdrv_wakeup_event_enum event)
{
	unsigned short que_wp;

	que_wp = IrDA_kdrv_wu_que.wp;
	que_wp++;
	if (que_wp >= IRDA_KDRV_WU_EV_QUE_MAX) {
		que_wp = 0;
	}
	if (que_wp == IrDA_kdrv_wu_que.rp) {
		MSG_IRDRV_FATAL("wakeup que memory full\n");
	} else {
		IrDA_kdrv_wu_que.event[IrDA_kdrv_wu_que.wp] = event;
		IrDA_kdrv_wu_que.wp = que_wp;

		MSG_IRDRV_MED_INFO("wakeup que wp=%#x rp=%#x event=%#x\n",
				IrDA_kdrv_wu_que.wp,IrDA_kdrv_wu_que.rp,
								(int)event);
	}

	wake_up(&IrDA_kdrv_read_wakeup);

}

void irda_kdrv_wakeup_read_irqsave(irda_kdrv_wakeup_event_enum event)
{
	unsigned short que_wp;
	unsigned long spin_lock_flags;

	spin_lock_irqsave(&IrDA_kdrv_spin_lock, spin_lock_flags);

	que_wp = IrDA_kdrv_wu_que.wp;
	que_wp++;
	if (que_wp >= IRDA_KDRV_WU_EV_QUE_MAX) {
		que_wp = 0;
	}
	if (que_wp == IrDA_kdrv_wu_que.rp) {
		MSG_IRDRV_FATAL("wakeup que memory full\n");
	} else {
		IrDA_kdrv_wu_que.event[IrDA_kdrv_wu_que.wp] = event;
		IrDA_kdrv_wu_que.wp = que_wp;

		MSG_IRDRV_MED_INFO("wakeup que wp=%#x rp=%#x event=%#x\n",
				IrDA_kdrv_wu_que.wp,IrDA_kdrv_wu_que.rp,
								(int)event);
	}

	spin_unlock_irqrestore(&IrDA_kdrv_spin_lock, spin_lock_flags);

	wake_up(&IrDA_kdrv_read_wakeup);

}

static TYPE_BAUD irda_kdrv_get_baud(void)
{
	TYPE_BAUD reg_baud;

	switch (IrDA_kdrv.qos_info.baud_rate) {
	case IRDA_BAUD_9600:
		reg_baud = IR_BAUD_9600;
		break;
	case IRDA_BAUD_19200:
		reg_baud = IR_BAUD_19200;
		break;
	case IRDA_BAUD_38400:
		reg_baud = IR_BAUD_38400;
		break;
	case IRDA_BAUD_57600:
		reg_baud = IR_BAUD_57600;
		break;
	case IRDA_BAUD_115200:
		reg_baud = IR_BAUD_115200;
		break;
	case IRDA_BAUD_4000000:
		reg_baud = IR_BAUD_4000000;
		break;
	default:
		reg_baud = IR_BAUD_9600;
		break;
	}

	return (reg_baud);
}

static void irda_kdrv_isr_set_mode(irda_kdrv_mode_enum irda_mode)
{
	TYPE_IR_SEND_KIND send_mode;

	switch (irda_mode) {
	case IRDA_KDRV_MODE_RECV:
		ir_reg_irda_set_ca(IrDA_kdrv.qos_info.connection_address);
		ir_reg_irda_set_mpi(IrDA_kdrv.qos_info.mpi * IRDA_MPI_TIMEUNIT);
		ir_reg_irda_set_mtt(IrDA_kdrv.qos_info.mtt * IRDA_MTT_TIMEUNIT);

		if (IrDA_kdrv.qos_info.baud_rate <= IRDA_BAUD_115200) {
			ir_reg_irda_set_bof(IrDA_kdrv.qos_info.add_bof);
			ir_reg_irda_set_baud(irda_kdrv_get_baud());
			ir_reg_irda_sir_set_rx_adr_match();
		} else {
			ir_reg_irda_fir_set_rx_adr_match();
		}
		break;
	case IRDA_KDRV_MODE_SEND:
		if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_NORMAL) {
			send_mode = IR_SEND_SINGLE;
		} else {
			send_mode = IR_SEND_CONTINUE;
		}
		if (IrDA_kdrv.qos_info.baud_rate <= IRDA_BAUD_115200) {
			ir_reg_irda_sir_set_tx(send_mode);
		} else {
			ir_reg_irda_fir_set_tx(send_mode);
		}
		break;
	case IRDA_KDRV_MODE_STOP:
		ir_reg_irda_txrx_dis();
		break;
	default:
		MSG_IRDRV_FATAL("mode change error:mode=%#x\n", irda_mode);
		break;
	}

}

static void irda_kdrv_isr_recv_err(void)
{
	MSG_IRDRV_ERR("isr receive err\n");

	irda_kdrv_isr_set_mode(IRDA_KDRV_MODE_STOP);

	ir_reg_irda_err_reset();

	irda_kdrv_isr_set_mode(IRDA_KDRV_MODE_RECV);
}

static int irda_kdrv_isr_recv_read(irda_kdrv_state_enum mode)
{
	int ret = IRDA_ISR_SUCCESS;
	irda_drv_data_info *irda_mem;
	unsigned short mem_wp;
	uint8 *write_drv_p;
	uint16 irda_crc_len;

	MSG_IRDRV_MED_INFO_ISR("isr fifo read:mode=%#x\n", mode);

	if (mode == IRDA_KDRV_STATUS_SIR_RECV) {
		irda_crc_len = IRDA_SIR_CRC_LEN;
	} else {
		irda_crc_len = IRDA_FIR_CRC_LEN;
	}

	irda_mem = (irda_drv_data_info *)IrDA_kdrv_mmap_p;

	mem_wp = irda_mem->receive.control.wp;
	mem_wp++;
	if (mem_wp >= IRDA_MEM_FRAME_MAX) {
		mem_wp = 0;
	}
	if (mem_wp == irda_mem->receive.control.rp) {
		ret = IRDA_ISR_ERROR;
		MSG_IRDRV_ERR("receive memory full error\n");
		goto error;
	} else {
		MSG_IRDRV_MED_INFO_ISR(
			"receive memory: read_p=%d write_p=%d\n",
						irda_mem->receive.control.rp,
						irda_mem->receive.control.wp);

		mem_wp = irda_mem->receive.control.wp;
		write_drv_p = &irda_mem->receive.databuff[mem_wp].data[0];
		irda_mem->receive.databuff[mem_wp].len = 0;
	}

	if (ir_reg_irda_get_rxdata(
		(uint16*)&irda_mem->receive.databuff[mem_wp].data[0],
		(int32*)&irda_mem->receive.databuff[mem_wp].len) ==
							IR_REG_RESULT_SUCCESS) {

		if (irda_mem->receive.databuff[mem_wp].len < irda_crc_len) {
			MSG_IRDRV_ERR("receive len=%d,crc_len=%d\n",
				(int)irda_mem->receive.databuff[mem_wp].len,
							irda_crc_len);
			irda_mem->receive.databuff[mem_wp].len =
							(uint16)IR_LENGTH_ZERO;
			ret = IRDA_ISR_ERROR;
		} else {
			MSG_IRDRV_MED_INFO_ISR_RECV_CTRL(
				"receive data(ctrl)=%#x,len=%d\n",
				irda_mem->receive.databuff[mem_wp].data[1],
				(int)irda_mem->receive.databuff[mem_wp].len);

			irda_mem->receive.databuff[mem_wp].len -=
								irda_crc_len;


			irda_mem->receive.control.wp++;
			if (irda_mem->receive.control.wp >=
							IRDA_MEM_FRAME_MAX) {
				irda_mem->receive.control.wp = 0;
			}
		}
	} else {
		MSG_IRDRV_FATAL("receive read err\n");
		ret = IRDA_ISR_ERROR;
	}

error:

	return (ret);
}

static int irda_kdrv_isr_sir_recv(TYPE_INT_FACTOR int_fact)
{
	int	isr_chk = IRDA_ISR_SUCCESS;

	MSG_IRDRV_MED_INFO_ISR("sir(isr) receive\n");

	if (int_fact == IR_INT_SIR_RX) {
		if (irda_kdrv_isr_recv_read(IRDA_KDRV_STATUS_SIR_RECV) ==
							IRDA_ISR_SUCCESS) {
			irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_RECV);
		} else {
			isr_chk = IRDA_ISR_ERROR;
		}
	} else {
		isr_chk = IRDA_ISR_ERROR;
	}

	if (isr_chk != IRDA_ISR_SUCCESS) {
		irda_kdrv_isr_recv_err();
	}

	return(isr_chk);
}

static int irda_kdrv_isr_sir_send(TYPE_INT_FACTOR int_fact)
{
	int	isr_chk = IRDA_ISR_SUCCESS;

	MSG_IRDRV_MED_INFO_ISR("sir(isr) send\n");

	if (int_fact == IR_INT_SIR_TX) {
		irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_SEND);
	} else {
		irda_kdrv_isr_set_mode(IRDA_KDRV_MODE_STOP);

		irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_SEND_ERR);
		isr_chk = IRDA_ISR_ERROR;
	}

	return(isr_chk);
}

static int irda_kdrv_isr_fir_recv(TYPE_INT_FACTOR int_fact)
{
	int	isr_chk = IRDA_ISR_SUCCESS;

	MSG_IRDRV_MED_INFO_ISR("fir(isr) receive\n");

	if (int_fact == IR_INT_FIR_RX) {
		if (irda_kdrv_isr_recv_read(IRDA_KDRV_STATUS_FIR_RECV) ==
							 IRDA_ISR_SUCCESS) {
			ir_reg_irda_fir_rx_restart();

			irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_RECV);
		} else {
			isr_chk = IRDA_ISR_ERROR;
		}
	} else {
		isr_chk = IRDA_ISR_ERROR;
	}

	if (isr_chk != IRDA_ISR_SUCCESS) {
		irda_kdrv_isr_recv_err();
	}

	return(isr_chk);
}

static int irda_kdrv_isr_fir_send(TYPE_INT_FACTOR int_fact)
{
	int	isr_chk = IRDA_ISR_SUCCESS;

	MSG_IRDRV_MED_INFO_ISR("fir(isr) send\n");

	if (int_fact == IR_INT_FIR_TX) {
		irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_SEND);
	} else {
		irda_kdrv_isr_set_mode(IRDA_KDRV_MODE_STOP);

		irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_SEND_ERR);
		isr_chk = IRDA_ISR_ERROR;
	}

	return(isr_chk);
}

static void irda_kdrv_isr_carrier_chk(void)
{
	ir_reg_irda_carrier_timer_int_clear();

	if (ir_reg_irda_carrier_chk() == IR_REG_RESULT_FAIL) {
		IrDA_kdrv_carrier_cnt++;

		MSG_IRDRV_MED_INFO_ISR("carrier not found(cnt=%d)\n",
						IrDA_kdrv_carrier_cnt);
	} else {
		IrDA_kdrv_carrier_cnt = 0;

		MSG_IRDRV_MED_INFO_ISR("carrier found(cnt=%d)\n",
						IrDA_kdrv_carrier_cnt);
	}

	if (IrDA_kdrv_carrier_cnt < IRDA_CARRIER_SENSE_CHK_CNT) {
		ir_reg_irda_carrier_timer_start();
	} else {
		MSG_IRDRV_MED_INFO_ISR("no carrier!(cnt=%d)\n",
						IrDA_kdrv_carrier_cnt);

		ir_reg_irda_carrier_timer_term();
		irda_kdrv_wakeup_read(IRDA_KDRV_WU_EV_ISR_CARRIER_EXPIRE);
	}
}

static int irda_kdrv_isr_carrier(TYPE_INT_FACTOR int_fact)
{
	int	isr_chk = IRDA_ISR_SUCCESS;

	if (int_fact == IR_INT_TIMER_EXPIRE) {
		irda_kdrv_isr_carrier_chk();
	} else {
		ir_reg_irda_txrx_dis();

		ir_reg_irda_err_reset();

		ir_reg_irda_set_carrier_mode();

		IrDA_kdrv_carrier_cnt = 0;

		ir_reg_irda_carrier_timer_init(IRDA_CARRIER_SENSE_TIME);
		ir_reg_irda_carrier_timer_start();

		isr_chk = IRDA_ISR_ERROR;
	}

	return(isr_chk);
}

static irqreturn_t irda_kdrv_isr_irda(int irq, void *dev_id)
{
	irqreturn_t ret = IRQ_HANDLED;
	TYPE_INT_FACTOR	int_fact = IR_INT_NONE;
	uint16 w_sr3 = 0x00, w_sr0 = 0x00, w_sr1 = 0x00, w_sr18 = 0x00;
	int	isr_chk = IRDA_ISR_SUCCESS;

	int_fact = ir_reg_irda_get_int_factor(&w_sr3, &w_sr0, &w_sr1,&w_sr18);

	switch (IrDA_kdrv.state) {
	case IRDA_KDRV_STATUS_SIR_RECV:
		isr_chk = irda_kdrv_isr_sir_recv(int_fact);
		break;

	case IRDA_KDRV_STATUS_SIR_SEND:
		isr_chk = irda_kdrv_isr_sir_send(int_fact);
		break;

	case IRDA_KDRV_STATUS_FIR_RECV:
		isr_chk = irda_kdrv_isr_fir_recv(int_fact);
		break;

	case IRDA_KDRV_STATUS_FIR_SEND:
		isr_chk = irda_kdrv_isr_fir_send(int_fact);
		break;

	case IRDA_KDRV_STATUS_CARRIER:
		isr_chk = irda_kdrv_isr_carrier(int_fact);
		break;

	case IRDA_KDRV_STATUS_ERROR:
	default:
		isr_chk = IRDA_ISR_ERROR;
		break;
	}

	if (isr_chk != IRDA_ISR_SUCCESS) {
		if (int_fact == IR_INT_FATAL) {
			MSG_IRDRV_ERR("fatal:sr3=%#x sr0=%#x sr1=%#x\n",
							w_sr3, w_sr0,w_sr1);
			MSG_IRDRV_ERR("fatal:sr18=%#x\n",w_sr18);
		} else {
			MSG_IRDRV_ERR("error:state=%#x sr3=%#x sr0=%#x\n",
						IrDA_kdrv.state, w_sr3, w_sr0);
			MSG_IRDRV_ERR("error:sr1=%#x sr18=%#x\n",
							w_sr1, w_sr18);
		}
	}

	return (ret);
}

static void irda_kdrv_set_led_mode(irda_boud_enum baud)
{
	if (baud <= IRDA_BAUD_115200) {
		if (IrDA_kdrv.led_st != IRDA_KDRV_LED_ST_SIR) {
			ir_reg_irdadrv_sir_set_led();
			IrDA_kdrv.led_st = IRDA_KDRV_LED_ST_SIR;

			MSG_IRDRV_MED_INFO("chg led sir mode:%#x\n",
							IrDA_kdrv.led_st);
		}
	} else {
		if (IrDA_kdrv.led_st != IRDA_KDRV_LED_ST_FIR) {
			ir_reg_irdadrv_fir_set_led();
			IrDA_kdrv.led_st = IRDA_KDRV_LED_ST_FIR;

			MSG_IRDRV_MED_INFO("chg led fir mode:%#x\n",
							IrDA_kdrv.led_st);
		}
	}
}

static void irda_kdrv_set_mode(irda_kdrv_mode_enum irda_mode)
{
	TYPE_IR_SEND_KIND send_mode;

	switch (irda_mode) {
	case IRDA_KDRV_MODE_RECV:
		irda_kdrv_set_led_mode(IrDA_kdrv.qos_info.baud_rate);

		ir_reg_irda_set_ca(IrDA_kdrv.qos_info.connection_address);
		ir_reg_irda_set_mpi(IrDA_kdrv.qos_info.mpi * IRDA_MPI_TIMEUNIT);
		ir_reg_irda_set_mtt(IrDA_kdrv.qos_info.mtt * IRDA_MTT_TIMEUNIT);

		if (IrDA_kdrv.qos_info.baud_rate <= IRDA_BAUD_115200) {
			ir_reg_irda_set_bof(IrDA_kdrv.qos_info.add_bof);
			ir_reg_irda_set_baud(irda_kdrv_get_baud());
			ir_reg_irda_sir_set_rx_adr_match();
		} else {
			ir_reg_irda_fir_set_rx_adr_match();
		}
		break;
	case IRDA_KDRV_MODE_SEND:
		if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_NORMAL) {
			send_mode = IR_SEND_SINGLE;
		} else {
			send_mode = IR_SEND_CONTINUE;
		}
		if (IrDA_kdrv.qos_info.baud_rate <= IRDA_BAUD_115200) {
			ir_reg_irda_sir_set_tx(send_mode);
		} else {
			ir_reg_irda_fir_set_tx(send_mode);
		}
		break;
	case IRDA_KDRV_MODE_STOP:
		ir_reg_irda_txrx_dis();
		break;
	default:
		MSG_IRDRV_FATAL("mode change error:mode=%#x\n", irda_mode);
		break;
	}

}

static void irda_kdrv_set_receive_start(void)
{
	irda_kdrv_set_mode(IRDA_KDRV_MODE_STOP);
	irda_kdrv_set_mode(IRDA_KDRV_MODE_RECV);
}

static int irda_kdrv_irdacc_init_func(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	MSG_IRDRV_MED_INFO("Power On\n");

	ret = request_irq(IRDA_IRQ_NUM,
			  irda_kdrv_isr_irda,
			  IRQF_DISABLED |
			  IRQF_TRIGGER_LOW,
			  IRDA_IRQ_NAME,
			  NULL);

	if (ret != IRDA_KERNEL_API_SUCCESS) {
		MSG_IRDRV_FATAL("request_irq error:ret=%#x\n", ret);
		ret = -ENXIO;

		goto error;
	}

	if (ir_reg_irda_hwinit() == IR_REG_RESULT_SUCCESS) {
		ir_reg_irda_set_timer_control();

		irda_kdrv_set_receive_start();
	} else {
		MSG_IRDRV_FATAL("hw init error\n");
		ret = -EIO;
	}

error:

	if (ret != IRDA_KERNEL_API_SUCCESS) {
		(void)irda_kdrv_irdacc_term_func();
	}

	return (ret);
}

static int irda_kdrv_irdacc_term_func(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	MSG_IRDRV_MED_INFO("power off\n");

	synchronize_irq(IRDA_IRQ_NUM);
	free_irq(IRDA_IRQ_NUM, NULL);

	if (ir_reg_irda_hwterm() != IR_REG_RESULT_SUCCESS) {
		MSG_IRDRV_FATAL("ir_reg_irda_hwterm error\n");
		ret = -EIO;
	}

	irda_kdrv_wakeup_read_irqsave(IRDA_KDRV_WU_EV_POWEROFF);

	return (ret);
}

static void irda_kdrv_recv_data_ignore(void)
{
	irda_drv_data_info *irda_mem;

	MSG_IRDRV_ERR("receive data ignore:now_state=%#x\n", IrDA_kdrv.state);

	irda_mem = (irda_drv_data_info *)IrDA_kdrv_mmap_p;
	irda_mem->receive.control.wp = irda_mem->receive.control.rp;

}

static void irda_kdrv_send_err(void)
{
	MSG_IRDRV_ERR("send err\n");

	irda_kdrv_set_mode(IRDA_KDRV_MODE_STOP);

	ir_reg_irda_err_reset();

	if (IrDA_kdrv.poll == true) {
		MSG_IRDRV_ERR("poll wake up\n");

		IrDA_kdrv.poll = false;
		wake_up(&IrDA_kdrv_poll_wakeup);
	}
}

static int irda_kdrv_send_start(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;
	unsigned long spin_lock_flags;
	irda_drv_data_info *irda_mem;
	unsigned short mem_rp;
	unsigned short free_size;
	bool skip_flag = false;
	TYPE_IR_SEND_KIND send_mode;
	TYPE_IR_REG_RESULT reg_ret;

	free_size = ir_reg_irda_chk_fifo_free_size();
	if (free_size < IRDA_FREE_FIFO) {
		MSG_IRDRV_MED_INFO("fifo not empty(%d byte)\n", free_size);
		skip_flag = true;
	} else {
		if (IrDA_kdrv.fifo_write_cnt >= IRDA_FIFO_WRITE_MAX) {
			MSG_IRDRV_MED_INFO("fifo write max\n");
			skip_flag = true;
		}
	}

	irda_mem = (irda_drv_data_info *)IrDA_kdrv_mmap_p;
	mem_rp = irda_mem->send.control.rp;

	if (irda_mem->send.control.rp == irda_mem->send.control.wp) {
		MSG_IRDRV_MED_INFO("send memory empty\n");
		skip_flag = true;
	}

	if (skip_flag == false) {
		if (irda_mem->send.databuff[mem_rp].mode ==
							IRDA_SEND_MODE_NORMAL) {
			if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_CONTINUE) {
				IrDA_kdrv.send_mode =
						IRDA_KDRV_SEND_CONTINUE_LAST;

				MSG_IRDRV_MED_INFO(
					"send mode chg(continue last)=%#x\n",
							IrDA_kdrv.send_mode);
			}
		}

		if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_NORMAL) {
			send_mode = IR_SEND_SINGLE;
		} else {
			send_mode = IR_SEND_CONTINUE;
		}

		MSG_IRDRV_MED_INFO("send data(ctrl)=%#x,len=%d,mode=%d\n",
					irda_mem->send.databuff[mem_rp].data[1],
				(int)irda_mem->send.databuff[mem_rp].len,
								(int)send_mode);


		spin_lock_irqsave(&IrDA_kdrv_spin_lock, spin_lock_flags);

		reg_ret = ir_reg_irda_set_txdata(
			(uint16*)&irda_mem->send.databuff[mem_rp].data[0],
					irda_mem->send.databuff[mem_rp].len,
								send_mode);

		if (reg_ret == IR_REG_RESULT_SUCCESS) {
			IrDA_kdrv.fifo_write_cnt++;

			if (IrDA_kdrv.first_send == true) {
				if (IrDA_kdrv.qos_info.baud_rate <=
							 IRDA_BAUD_115200) {
					ir_reg_irda_sir_tx_ena();
				} else {
					ir_reg_irda_fir_tx_ena();
				}
				IrDA_kdrv.first_send = false;
			}

			MSG_IRDRV_MED_INFO("send enable:fifo cnt=%d\n",
						IrDA_kdrv.fifo_write_cnt);

		} else {
			MSG_IRDRV_ERR
				("set tx reg error:data_p=%#x len=%#x\n",
				(int)&irda_mem->send.databuff[mem_rp].data[0],
				(int)irda_mem->send.databuff[mem_rp].len);

			ret = -EINVAL;
		}

		spin_unlock_irqrestore(&IrDA_kdrv_spin_lock, spin_lock_flags);

		irda_mem->send.control.rp++;
		if (irda_mem->send.control.rp >= IRDA_MEM_FRAME_MAX) {
			irda_mem->send.control.rp = 0;
		}

		if (IrDA_kdrv.poll == true) {
			MSG_IRDRV_MED_INFO("poll wake up\n");

			IrDA_kdrv.poll = false;
			wake_up(&IrDA_kdrv_poll_wakeup);
		}

		MSG_IRDRV_MED_INFO("mem_wp=%d, mem_rp=%d, poll=%#x\n",
						irda_mem->send.control.wp,
						irda_mem->send.control.rp,
							IrDA_kdrv.poll);
	}

	return (ret);
}

static irda_read_fops_control_enum irda_kdrv_send_comp(void)
{
	irda_read_fops_control_enum ret = IRDA_FOPS_READ_CONTINUE;
	irda_kdrv_action_enum act_val;

	MSG_IRDRV_MED_IN("send comp(fifo cnt=%d)\n", IrDA_kdrv.fifo_write_cnt);

	act_val = irda_kdrv_action[IrDA_kdrv.state][IRDA_KDRV_EVENT_ISR_IRDA];
	if (act_val == IRDA_ACT_OK) {
		IrDA_kdrv.fifo_write_cnt--;
		if (IrDA_kdrv.fifo_write_cnt < 0) {
			IrDA_kdrv.fifo_write_cnt = 0;

			MSG_IRDRV_ERR("send comp error(fifo empty)\n");
		}

		if ((IrDA_kdrv.send_mode == IRDA_KDRV_SEND_NORMAL) ||
			(IrDA_kdrv.send_mode == IRDA_KDRV_SEND_CONTINUE_LAST &&
					IrDA_kdrv.fifo_write_cnt == 0)) {

			if (IrDA_kdrv.qos_info.baud_rate == IRDA_BAUD_4000000) {
				IrDA_kdrv.state = IRDA_KDRV_STATUS_FIR_RECV;
			} else {
				IrDA_kdrv.state = IRDA_KDRV_STATUS_SIR_RECV;
			}

			MSG_IRDRV_MED_INFO("chg state=%#x\n", IrDA_kdrv.state);

			irda_kdrv_set_receive_start();

			ret = IRDA_FOPS_READ_END;
		} else {
			if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_CONTINUE) {
				if (irda_kdrv_send_start() !=
						IRDA_KERNEL_API_SUCCESS) {
					IrDA_kdrv.state =
							IRDA_KDRV_STATUS_ERROR;

					MSG_IRDRV_ERR("chg state=%#x\n",
							IrDA_kdrv.state);

					irda_kdrv_send_err();

					ret = IRDA_FOPS_READ_ERR_END;

				}
			}
		}
	} else {
		ret = IRDA_FOPS_READ_CONTINUE;

		MSG_IRDRV_MED_INFO("send comp warning(cross?)\n");
	}

	MSG_IRDRV_MED_OUT("send comp:ret=%#x(fifo cnt=%d)\n",
					ret, IrDA_kdrv.fifo_write_cnt);

	return (ret);
}

static int irda_kdrv_set_qos(irda_qos_info set_qos_info)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	MSG_IRDRV_MED_INFO("baud_rate=%#x,ca=%#x,add_bof=%d\n",
							set_qos_info.baud_rate,
						set_qos_info.connection_address,
							set_qos_info.add_bof);

	MSG_IRDRV_MED_INFO("mpi=%d,mtt=%d\n", set_qos_info.mpi,
							set_qos_info.mtt);

	if (set_qos_info.baud_rate < IRDA_BAUD_9600 ||
				set_qos_info.baud_rate > IRDA_BAUD_4000000) {
		MSG_IRDRV_ERR("baud_rate err\n");
		ret = -EINVAL;
		goto error;
	}

	switch (set_qos_info.add_bof) {
	case IRDA_DRV_SET_ADD_BOF_48:
	case IRDA_DRV_SET_ADD_BOF_32:
	case IRDA_DRV_SET_ADD_BOF_24:
	case IRDA_DRV_SET_ADD_BOF_20:
	case IRDA_DRV_SET_ADD_BOF_16:
	case IRDA_DRV_SET_ADD_BOF_14:
	case IRDA_DRV_SET_ADD_BOF_12:
	case IRDA_DRV_SET_ADD_BOF_10:
	case IRDA_DRV_SET_ADD_BOF_8:
	case IRDA_DRV_SET_ADD_BOF_6:
	case IRDA_DRV_SET_ADD_BOF_5:
	case IRDA_DRV_SET_ADD_BOF_4:
	case IRDA_DRV_SET_ADD_BOF_3:
	case IRDA_DRV_SET_ADD_BOF_2:
	case IRDA_DRV_SET_ADD_BOF_1:
	case IRDA_DRV_SET_ADD_BOF_0:
		ret = IRDA_KERNEL_API_SUCCESS;
		break;
	default:
		MSG_IRDRV_ERR("add_bof err\n");
		ret = -EINVAL;
		goto error;
		break;
	}

	if ((set_qos_info.mpi < IRDA_DRV_SET_MPI_MIN ||
				set_qos_info.mpi > IRDA_DRV_SET_MPI_MAX )) {
		MSG_IRDRV_ERR("mpi err\n");
		ret = -EINVAL;
		goto error;
	}

	if ((set_qos_info.mtt < IRDA_DRV_SET_MTT_MIN ||
				set_qos_info.mtt > IRDA_DRV_SET_MTT_MAX) &&
				set_qos_info.mtt != IRDA_DRV_SET_MTT_ZERO) {
		MSG_IRDRV_ERR("mtt err\n");
		ret = -EINVAL;
		goto error;
	}

	if (IrDA_kdrv.state != IRDA_KDRV_STATUS_SIR_RECV &&
				IrDA_kdrv.state != IRDA_KDRV_STATUS_FIR_RECV) {

		MSG_IRDRV_MED_INFO("set qos val keep\n");

		IrDA_kdrv.qos_info.baud_rate = set_qos_info.baud_rate;
		IrDA_kdrv.qos_info.connection_address =
						set_qos_info.connection_address;
		IrDA_kdrv.qos_info.add_bof = set_qos_info.add_bof;
		IrDA_kdrv.qos_info.mpi = set_qos_info.mpi;
		IrDA_kdrv.qos_info.mtt = set_qos_info.mtt;
	} else {
		if (IrDA_kdrv.qos_info.baud_rate != set_qos_info.baud_rate) {
			MSG_IRDRV_MED_INFO("chg baud_rate=%#x -> %#x\n",
						IrDA_kdrv.qos_info.baud_rate,
						set_qos_info.baud_rate);

			if (set_qos_info.baud_rate < IRDA_BAUD_4000000){
				if (IrDA_kdrv.state !=
						IRDA_KDRV_STATUS_SIR_RECV) {
					IrDA_kdrv.state =
						IRDA_KDRV_STATUS_SIR_RECV;
					MSG_IRDRV_MED_INFO(
							"chg state=%#x\n",
							IrDA_kdrv.state);
				}
			} else {
				if (IrDA_kdrv.state !=
						IRDA_KDRV_STATUS_FIR_RECV) {
					IrDA_kdrv.state =
						IRDA_KDRV_STATUS_FIR_RECV;
					MSG_IRDRV_MED_INFO(
							"chg state=%#x\n",
							IrDA_kdrv.state);
				}
			}

			IrDA_kdrv.qos_info.baud_rate = set_qos_info.baud_rate;
			irda_kdrv_set_mode(IRDA_KDRV_MODE_STOP);
			irda_kdrv_set_mode(IRDA_KDRV_MODE_RECV);
		}

		if (IrDA_kdrv.qos_info.connection_address !=
					set_qos_info.connection_address) {
			MSG_IRDRV_MED_INFO("chg ca=%#x -> %#x\n",
					IrDA_kdrv.qos_info.connection_address,
					set_qos_info.connection_address);

			IrDA_kdrv.qos_info.connection_address =
						set_qos_info.connection_address;
			ir_reg_irda_set_ca(
					IrDA_kdrv.qos_info.connection_address);
		}

		if (IrDA_kdrv.qos_info.add_bof != set_qos_info.add_bof) {
			MSG_IRDRV_MED_INFO("chg bof=%d -> %d\n",
						IrDA_kdrv.qos_info.add_bof,
						set_qos_info.add_bof);

			IrDA_kdrv.qos_info.add_bof = set_qos_info.add_bof;
			ir_reg_irda_set_bof((uint32)IrDA_kdrv.qos_info.add_bof);
		}

		if (IrDA_kdrv.qos_info.mpi != set_qos_info.mpi) {
			MSG_IRDRV_MED_INFO("chg mpi=%d -> %d\n",
							IrDA_kdrv.qos_info.mpi,
							set_qos_info.mpi);

			IrDA_kdrv.qos_info.mpi = set_qos_info.mpi;
			ir_reg_irda_set_mpi(
			(uint32)IrDA_kdrv.qos_info.mpi * IRDA_MPI_TIMEUNIT);
		}

		if (IrDA_kdrv.qos_info.mtt != set_qos_info.mtt) {
			MSG_IRDRV_MED_INFO("chg mtt=%d -> %d\n",
							IrDA_kdrv.qos_info.mtt,
							set_qos_info.mtt);

			IrDA_kdrv.qos_info.mtt = set_qos_info.mtt;
			ir_reg_irda_set_mtt(
				IrDA_kdrv.qos_info.mtt * IRDA_MTT_TIMEUNIT);
		}
	}

error:

	return (ret);

}

static int irda_kdrv_carrier_sense_start(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	MSG_IRDRV_MED_INFO("CARRIER TIME=%d\n", IRDA_CARRIER_SENSE_TIME);

	irda_kdrv_set_mode(IRDA_KDRV_MODE_STOP);

	ir_reg_irdadrv_sir_set_led();
	ir_reg_irda_set_carrier_mode();

	IrDA_kdrv_carrier_cnt = 0;

	ir_reg_irda_carrier_timer_init(IRDA_CARRIER_SENSE_TIME);
	ir_reg_irda_carrier_timer_start();

	return (ret);
}










static int irda_kdrv_ioctl_gpio_control(unsigned int cmd)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	MSG_IRDRV_MED_INFO("gpio control cmd=0x%x\n", cmd);

	switch (cmd) {
	case IRDA_DRV_IOCTL_GPIO_LEDA_ON:
		ir_reg_led_leda_on();
		break;

	case IRDA_DRV_IOCTL_GPIO_LEDA_OFF:
		ir_reg_led_leda_off();
		break;

	default:
		ret = -EINVAL;
		break;

	}

	return (ret);
}



static int irda_kdrv_memory_alloc(void)
{
	int ret = IRDA_MEM_OK;

	IrDA_kmalloc_ptr =  kmalloc(IRDA_MMAP_LENGTH + PAGE_SIZE, GFP_KERNEL);
	if (IrDA_kmalloc_ptr == NULL){
		ret = IRDA_MEM_ERR_KMALLOC;

		IrDA_kdrv_mmap_p = NULL;

		MSG_IRDRV_ERR("kmalloc error\n");
	} else {
		IrDA_kdrv_mmap_p =
				(void *)ALIGN((unsigned long)IrDA_kmalloc_ptr,
								PAGE_SIZE);
	}

	if (ret == IRDA_MEM_OK) {
		if (ir_reg_ioremap_nocache() != IR_REG_RESULT_SUCCESS) {
			ret = IRDA_MEM_ERR_IOREMAP;
			MSG_IRDRV_ERR("ioremap error\n");
		}
	}

	if (ret != IRDA_MEM_OK) {
		irda_kdrv_memory_free();
	}

	return (ret);
}

static void irda_kdrv_memory_free(void)
{
	if (IrDA_kmalloc_ptr != NULL) {
		kfree(IrDA_kmalloc_ptr);
		IrDA_kmalloc_ptr = NULL;
	}
	IrDA_kdrv_mmap_p = NULL;

	ir_reg_iounmap();
}

int irda_kdrv_fops_open(struct inode *inode, struct file *filp)
{
	int ret = IRDA_KERNEL_API_SUCCESS;


	irda_kdrv_state_enum now_state;
	irda_kdrv_action_enum act_val;

	down(&IrDA_kdrv_sem);
	now_state = IrDA_kdrv.state;
	act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_OPEN];
	if (act_val == IRDA_ACT_OK) {
		irda_kdrv_init_control();

		IrDA_kdrv.state = IRDA_KDRV_STATUS_OPEN;
	}
	MSG_IRDRV_MED_INFO("irda open:now_state=%#x, next_state=%#x\n",
						now_state, IrDA_kdrv.state);

	if (act_val != IRDA_ACT_OK) {
		MSG_IRDRV_ERR("open req error:now_state=%#x\n", now_state);

		ret = -EPERM;
	}





	if (ret == IRDA_KERNEL_API_SUCCESS) {
		wake_lock_init(&shirda_wake_lock, WAKE_LOCK_IDLE,
							SHIRDA_WAKELOCK);
		wake_lock(&shirda_wake_lock);
	}

	up(&IrDA_kdrv_sem);
	return (ret);
}

int irda_kdrv_fops_close(struct inode *inodep, struct file *filp)
{
	int ret = IRDA_KERNEL_API_SUCCESS;
	irda_kdrv_state_enum now_state;

	down(&IrDA_kdrv_sem);

	now_state = IrDA_kdrv.state;

	IrDA_kdrv.state = IRDA_KDRV_STATUS_CLOSE;

	MSG_IRDRV_MED_INFO("irda close:now_state=%#x next_state=%#x\n",
						now_state, IrDA_kdrv.state);

	if (now_state != IRDA_KDRV_STATUS_OPEN) {
		MSG_IRDRV_ERR("close req warning:now_state=%#x\n",
							now_state);

		(void)irda_kdrv_irdacc_term_func();
	} else {
		irda_kdrv_wakeup_read_irqsave(IRDA_KDRV_WU_EV_POWEROFF);
	}



	wake_unlock(&shirda_wake_lock);
	wake_lock_destroy(&shirda_wake_lock);

	up(&IrDA_kdrv_sem);
	return (ret);
}

int irda_kdrv_fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = IRDA_KERNEL_API_SUCCESS;
	unsigned long pfn;

	MSG_IRDRV_MED_INFO("irda mmap\n");

	pfn = vma->vm_pgoff;
	if ((pfn == (IRDA_ADDR_HCS1 >> PAGE_SHIFT)) ||
			(pfn == (IRDA_ADDR_HCS2 >> PAGE_SHIFT))){
		if ((vma->vm_end - vma->vm_start) == IRDA_HCS_SIZE) {
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			ret = io_remap_pfn_range(vma, vma->vm_start, pfn,
					IRDA_HCS_SIZE, vma->vm_page_prot);
			if (ret != IRDA_KERNEL_API_SUCCESS){
				MSG_IRDRV_FATAL("io_remap_pfn_range\n");
				ret = -ENXIO;
			}
		} else {
			MSG_IRDRV_FATAL("illegal size\n");
			ret = -ENXIO;
		}
	} else if (pfn == 0) {
		if ((vma->vm_end - vma->vm_start) == IRDA_MMAP_SIZE){
			if (IrDA_kdrv_mmap_p != NULL) {
				pfn = __pa(IrDA_kdrv_mmap_p) >> PAGE_SHIFT;
				ret = remap_pfn_range(vma, vma->vm_start, pfn,
							IRDA_MMAP_SIZE,
							vma->vm_page_prot);
				if (ret != IRDA_KERNEL_API_SUCCESS) {
					MSG_IRDRV_FATAL(
						"remap_pfn_range error\n");
					ret = -ENXIO;
				}
			} else {
				MSG_IRDRV_FATAL("memory error\n");
				ret = -ENOMEM;
			}
		} else {
			MSG_IRDRV_FATAL("illegal size\n");
			ret = -ENXIO;
		}
	} else {
		MSG_IRDRV_FATAL("illegal address\n");
		ret = -ENXIO;
	}

	return (ret);
}

int irda_kdrv_fops_ioctl(
		struct inode *inode, struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	int ret = IRDA_KERNEL_API_SUCCESS;
	irda_kdrv_state_enum now_state;
	irda_kdrv_action_enum act_val;
	unsigned long copy_ret;
	int check_rx;
	irda_qos_info set_qos_info;

	MSG_IRDRV_MED_INFO("irda ioctl cmd=%#x\n", cmd);

	down(&IrDA_kdrv_sem);

	switch (cmd) {
	case IRDA_DRV_IOCTL_IRDACC_INIT:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_INIT];
		if (act_val == IRDA_ACT_OK) {
			if (IrDA_kdrv.qos_info.baud_rate == IRDA_BAUD_4000000) {
				IrDA_kdrv.state = IRDA_KDRV_STATUS_FIR_RECV;
			} else {
				IrDA_kdrv.state = IRDA_KDRV_STATUS_SIR_RECV;
			}
			MSG_IRDRV_MED_INFO(
				"power on:now_state=%#x next_state=%#x\n",
						now_state, IrDA_kdrv.state);
		} else {
			MSG_IRDRV_ERR(
				"power on req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}

		if (act_val == IRDA_ACT_OK) {
			ret = irda_kdrv_irdacc_init_func();
			if (ret == IRDA_KERNEL_API_SUCCESS) {
				copy_ret = copy_to_user((int __user *)arg,
							&IrDA_kdrv.qos_info,
						sizeof(IrDA_kdrv.qos_info));
				if (copy_ret != IRDA_KERNEL_API_SUCCESS) {
					MSG_IRDRV_FATAL(
					"copy_to_user QOS error=%#x\n",
								(int)copy_ret);

					(void)irda_kdrv_irdacc_term_func();

					ret = -ENXIO;
				}
			}
		}

		if (ret != IRDA_KERNEL_API_SUCCESS) {
			IrDA_kdrv.state = now_state;
		}
		break;

	case IRDA_DRV_IOCTL_IRDACC_TERM:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_TERM];
		if (act_val == IRDA_ACT_OK) {
			IrDA_kdrv.state = IRDA_KDRV_STATUS_OPEN;
			MSG_IRDRV_MED_INFO("power off:now_state=%#x\n",
								now_state);
		} else {
			MSG_IRDRV_ERR(
				"power off req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}

		irda_kdrv_set_mode(IRDA_KDRV_MODE_STOP);
		(void)irda_kdrv_irdacc_term_func();
		break;

	case IRDA_DRV_IOCTL_SET_QOS:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_SET_QOS];
		if (act_val == IRDA_ACT_OK) {
			MSG_IRDRV_MED_INFO("set qos:now_state=%#x\n",
								now_state);
		} else {
			MSG_IRDRV_ERR(
				"set qos req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}

		if (act_val == IRDA_ACT_OK) {
			copy_ret = copy_from_user(&set_qos_info,
						(int __user *)arg,
						sizeof(irda_qos_info));
			if (copy_ret == IRDA_KERNEL_API_SUCCESS) {
				ret = irda_kdrv_set_qos(set_qos_info);

				if (ret != IRDA_KERNEL_API_SUCCESS) {
					IrDA_kdrv.state = now_state;
				}
			} else {
				ret = -ENXIO;
			}
		}
		break;

	case IRDA_DRV_IOCTL_GET_QOS:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_GET_QOS];
		if (act_val == IRDA_ACT_OK) {
			MSG_IRDRV_MED_INFO(
				"get qos:now_state=%#x\n", now_state);
		} else {
			MSG_IRDRV_ERR(
				"get qos req error:now_state=%#x\n", now_state);
			ret = -EPERM;
		}

		if (act_val == IRDA_ACT_OK) {
			copy_ret = copy_to_user(
					(int __user *)arg,
					&IrDA_kdrv.qos_info,
					sizeof(IrDA_kdrv.qos_info));
			if (copy_ret != IRDA_KERNEL_API_SUCCESS) {
				MSG_IRDRV_FATAL(
					"copy_to_user QOS error=%#x\n", (int)copy_ret);

				ret = -ENXIO;
			}

			MSG_IRDRV_MED_INFO_IOCTL(
					"baud_rate          = %#x\n",
					IrDA_kdrv.qos_info.baud_rate);
			MSG_IRDRV_MED_INFO_IOCTL(
					"connection_address = %#x\n",
				IrDA_kdrv.qos_info.connection_address);
			MSG_IRDRV_MED_INFO_IOCTL(
					"add_bof            = %d\n",
					IrDA_kdrv.qos_info.add_bof);
			MSG_IRDRV_MED_INFO_IOCTL(
					"mpi                = %d\n",
					IrDA_kdrv.qos_info.mpi);
			MSG_IRDRV_MED_INFO(
					"mtt                = %d\n",
					IrDA_kdrv.qos_info.mtt);
		}
		break;

	case IRDA_DRV_IOCTL_CARRIER_SENSE:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_CARRIER];
		if (act_val == IRDA_ACT_OK) {

			ir_reg_irda_int_clear();

			IrDA_kdrv.state = IRDA_KDRV_STATUS_CARRIER;

			MSG_IRDRV_MED_INFO(
				"carrier sense:now_state=%#x\n", now_state);
		} else {
			MSG_IRDRV_ERR(
				"carrier sense req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}

		if (act_val == IRDA_ACT_OK) {
			ret = irda_kdrv_carrier_sense_start();
		}

		if (ret != IRDA_KERNEL_API_SUCCESS) {
			IrDA_kdrv.state = now_state;
		}
		break;

	case IRDA_DRV_IOCTL_CHECK_RX:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_CHK_RX];
		if (act_val == IRDA_ACT_OK) {
			MSG_IRDRV_MED_INFO("check rx:now_state=%#x\n",
								now_state);
		} else {
			MSG_IRDRV_ERR(
				"check rx req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}

		if (act_val == IRDA_ACT_OK) {
			check_rx = (int)ir_reg_irda_chk_fifo_used_size();

			MSG_IRDRV_MED_INFO("check rx:size=%d\n", check_rx);

			copy_ret = copy_to_user(
					(int __user *)arg,
					&check_rx,
					sizeof(check_rx));
			if (copy_ret != IRDA_KERNEL_API_SUCCESS) {
				MSG_IRDRV_FATAL(
				"copy_to_user chk_rx error=%#x\n",
								(int)copy_ret);

				ret = -ENXIO;
			}

		}
		break;




	case IRDA_DRV_IOCTL_GPIO_LEDA_ON:
	case IRDA_DRV_IOCTL_GPIO_LEDA_OFF:
		ret = irda_kdrv_ioctl_gpio_control(cmd);
		break;



	case IRDA_DRV_IOCTL_READ_END:
		now_state = IrDA_kdrv.state;
		act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_READEND];
		if (act_val == IRDA_ACT_OK) {
			irda_kdrv_wakeup_read_irqsave(IRDA_KDRV_WU_EV_POWEROFF);
		} else {
			MSG_IRDRV_ERR(
				"read end req error:now_state=%#x\n",
								now_state);
			ret = -EPERM;
		}
		break;

	default:
		MSG_IRDRV_ERR(
			"IOCTL cmd error:cmd=%#x\n", cmd);

		ret = -EINVAL;
		break;
	}

	up(&IrDA_kdrv_sem);

	return (ret);
}

unsigned int irda_kdrv_fops_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = POLLIN | POLLRDNORM;
	irda_kdrv_state_enum now_state;
	irda_kdrv_action_enum act_val;
	irda_drv_data_info *irda_mem;
	unsigned short mem_wp;

	MSG_IRDRV_MED_IN("poll(mask=%#x)\n", mask);

	down(&IrDA_kdrv_sem);

	now_state = IrDA_kdrv.state;
	act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_POLL] ;
	if (act_val == IRDA_ACT_OK) {
		if (now_state == IRDA_KDRV_STATUS_SIR_RECV ||
				now_state == IRDA_KDRV_STATUS_FIR_RECV ||
					now_state == IRDA_KDRV_STATUS_CARRIER) {
			ir_reg_irda_int_clear();
		}

		if (IrDA_kdrv.qos_info.baud_rate == IRDA_BAUD_4000000) {
			IrDA_kdrv.state = IRDA_KDRV_STATUS_FIR_SEND;
		} else {
			IrDA_kdrv.state = IRDA_KDRV_STATUS_SIR_SEND;
		}
		MSG_IRDRV_MED_INFO("poll:now_state=%#x next_state=%#x\n",
						now_state, IrDA_kdrv.state);
	} else {
		mask = POLLERR;
		MSG_IRDRV_ERR("poll req error:now_state=%#x\n", now_state);
	}

	if (mask == POLLERR) {
		goto error;
	}

	irda_mem = (irda_drv_data_info *)IrDA_kdrv_mmap_p;

	if (now_state == IRDA_KDRV_STATUS_SIR_RECV ||
				now_state == IRDA_KDRV_STATUS_FIR_RECV ||
					now_state == IRDA_KDRV_STATUS_CARRIER) {
		IrDA_kdrv.first_send = true;

		IrDA_kdrv.fifo_write_cnt = 0;

		irda_mem->send.control.rp = irda_mem->send.control.wp;

		irda_mem->receive.control.wp = irda_mem->receive.control.rp;

		irda_kdrv_wakeup_read_irqsave(IRDA_KDRV_WU_EV_SEND_WAIT);

	}

	poll_wait(filp, &IrDA_kdrv_poll_wakeup, wait);

	mem_wp = irda_mem->send.control.wp;
	mem_wp++;
	if (mem_wp >= IRDA_MEM_FRAME_MAX) {
		mem_wp = 0;
	}
	if (mem_wp != irda_mem->send.control.rp) {
		mask = POLLOUT | POLLWRNORM;

		IrDA_kdrv.poll = false;
	} else {
		IrDA_kdrv.poll = true;
	}

	MSG_IRDRV_MED_INFO("mem_wp=%d, mem_rp=%d, poll=%#x\n",
						irda_mem->send.control.wp,
						irda_mem->send.control.rp,
								IrDA_kdrv.poll);

error:
	up(&IrDA_kdrv_sem);

	MSG_IRDRV_MED_OUT("poll(mask=%#x)\n", mask);

	return (mask);
}

ssize_t irda_kdrv_fops_write(
		struct file *filp, const char __user *buf, size_t count,
								loff_t *f_pos)
{
	ssize_t ret = IRDA_KERNEL_API_SUCCESS;
	irda_drv_data_info *irda_mem;
	unsigned short mem_rp;
	irda_kdrv_state_enum now_state;
	irda_kdrv_action_enum act_val;

	MSG_IRDRV_MED_INFO("irda write\n");

	down(&IrDA_kdrv_sem);

	now_state = IrDA_kdrv.state;
	act_val = irda_kdrv_action[now_state][IRDA_KDRV_EVENT_WRITE] ;
	if (act_val != IRDA_ACT_OK) {
		MSG_IRDRV_ERR("write req error:now_state=%#x\n", now_state);

		ret = -EPERM;
		goto error;
	}

	if (IrDA_kdrv.first_send == true) {
		irda_mem = (irda_drv_data_info *)IrDA_kdrv_mmap_p;
		mem_rp = irda_mem->send.control.rp;

		if (irda_mem->send.databuff[irda_mem->send.control.rp].mode ==
							IRDA_SEND_MODE_NORMAL) {
			IrDA_kdrv.send_mode = IRDA_KDRV_SEND_NORMAL;
		} else {
			IrDA_kdrv.send_mode = IRDA_KDRV_SEND_CONTINUE;
		}

		MSG_IRDRV_MED_INFO("send mode=%#x\n", IrDA_kdrv.send_mode);

		irda_kdrv_set_mode(IRDA_KDRV_MODE_SEND);

	} else {
		if (IrDA_kdrv.send_mode == IRDA_KDRV_SEND_NORMAL) {
			IrDA_kdrv.state = IRDA_KDRV_STATUS_ERROR;
			MSG_IRDRV_ERR("chg state=%#x\n", IrDA_kdrv.state);

			irda_kdrv_send_err();

			ret = -EPERM;
			goto error;
		}
	}

	ret = irda_kdrv_send_start();
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		IrDA_kdrv.state = IRDA_KDRV_STATUS_ERROR;
		MSG_IRDRV_ERR("chg state=%#x\n", IrDA_kdrv.state);

		irda_kdrv_send_err();
	}

error:
	up(&IrDA_kdrv_sem);

	return (ret);
}

ssize_t irda_kdrv_fops_read(struct file *filp, char __user *buf, size_t count,
								loff_t *f_pos)
{
	ssize_t ret = -EIO;
	unsigned long spin_lock_flags;
	irda_kdrv_wakeup_event_enum event;
	bool read_action = true;
	irda_kdrv_action_enum act_val;
	irda_read_fops_control_enum read_control_val;
	long timeout_val;

	MSG_IRDRV_MED_IN("irda read\n");

	while (read_action) {
		timeout_val = 1;
		if (IrDA_kdrv.state == IRDA_KDRV_STATUS_FIR_SEND ||
				IrDA_kdrv.state == IRDA_KDRV_STATUS_SIR_SEND) {
			timeout_val = wait_event_timeout(
							IrDA_kdrv_read_wakeup,
				(IrDA_kdrv_wu_que.rp != IrDA_kdrv_wu_que.wp),
						(IRDA_SEND_TIME_EXP * HZ));
		} else {
			wait_event(IrDA_kdrv_read_wakeup,
				(IrDA_kdrv_wu_que.rp != IrDA_kdrv_wu_que.wp));

		}
		read_action = false;

		spin_lock_irqsave(&IrDA_kdrv_spin_lock, spin_lock_flags);

		if (timeout_val == 0) {
			event = IRDA_KDRV_WU_EV_ISR_SEND_ERR;

			MSG_IRDRV_ERR("wait_event_timeout=%d,event=%d\n",
						(int)timeout_val, event);
		} else {
			event = IrDA_kdrv_wu_que.event[IrDA_kdrv_wu_que.rp];
			if ((IrDA_kdrv_wu_que.rp+1) >= IRDA_KDRV_WU_EV_QUE_MAX){
				IrDA_kdrv_wu_que.rp = 0;
			} else {
				IrDA_kdrv_wu_que.rp++;
			}
		}

		MSG_IRDRV_MED_INFO("read wakeup event=%#x,rp=%d,wp=%d\n",
						event, IrDA_kdrv_wu_que.rp,
							IrDA_kdrv_wu_que.wp);

		spin_unlock_irqrestore(&IrDA_kdrv_spin_lock, spin_lock_flags);

		switch (event) {
		case IRDA_KDRV_WU_EV_ISR_RECV:

			down(&IrDA_kdrv_sem);
			act_val =
		irda_kdrv_action[IrDA_kdrv.state][IRDA_KDRV_EVENT_ISR_IRDA] ;

			if (act_val == IRDA_ACT_OK) {
				ret = IRDA_DRV_RETUN_RECV_COMP;
			} else {
				irda_kdrv_recv_data_ignore();

				read_action = true;
			}
			up(&IrDA_kdrv_sem);
			break;




		case IRDA_KDRV_WU_EV_ISR_CARRIER_EXPIRE:

			down(&IrDA_kdrv_sem);
			act_val =
		irda_kdrv_action[IrDA_kdrv.state][IRDA_KDRV_EVENT_ISR_CARRIER];

			if (act_val == IRDA_ACT_OK) {
				if (IrDA_kdrv.qos_info.baud_rate ==
							IRDA_BAUD_4000000) {
					IrDA_kdrv.state =
						IRDA_KDRV_STATUS_FIR_RECV;
				} else {
					IrDA_kdrv.state =
						IRDA_KDRV_STATUS_SIR_RECV;
				}

				MSG_IRDRV_MED_INFO(
					"carrier expire(chg state=%#x)\n",
					IrDA_kdrv.state);

				irda_kdrv_set_receive_start();

				ret = IRDA_DRV_RETUN_NO_CARRIER;
			} else {
				read_action = true;
			}
			up(&IrDA_kdrv_sem);
			break;

		case IRDA_KDRV_WU_EV_SEND_WAIT:
			MSG_IRDRV_MED_INFO("chg:wait_event_timeout()\n");

			read_action = true;
			break;

		case IRDA_KDRV_WU_EV_ISR_SEND:

			down(&IrDA_kdrv_sem);

			read_control_val = irda_kdrv_send_comp() ;

			switch (read_control_val) {
			case IRDA_FOPS_READ_CONTINUE:
				read_action = true;
				break;

			case IRDA_FOPS_READ_END:
				ret = IRDA_DRV_RETUN_SEND_COMP;
				break;

			case IRDA_FOPS_READ_ERR_END:
				ret = IRDA_DRV_RETUN_SEND_ERR;
				break;
			default:
				MSG_IRDRV_FATAL(
				"irda_kdrv_send_comp return error=%#x\n",
							read_control_val);
				break;
			}

			up(&IrDA_kdrv_sem);
			break;

		case IRDA_KDRV_WU_EV_ISR_SEND_ERR:

			down(&IrDA_kdrv_sem);

			act_val =
		irda_kdrv_action[IrDA_kdrv.state][IRDA_KDRV_EVENT_ISR_IRDA] ;
			if (act_val == IRDA_ACT_OK) {
				IrDA_kdrv.state = IRDA_KDRV_STATUS_ERROR;

				irda_kdrv_send_err();

				ret = IRDA_DRV_RETUN_SEND_ERR;
			} else {
				read_action = true;
			}

			up(&IrDA_kdrv_sem);
			break;

		case IRDA_KDRV_WU_EV_POWEROFF:
			ret = IRDA_DRV_RETUN_IRDA_STOP;
			break;

		default:
			MSG_IRDRV_FATAL(
				"send wake up error=%#x\n", event);
			break;
		}
	}

	MSG_IRDRV_MED_OUT("irda read:ret=%#x\n", ret);

	return (ret);
}

static struct file_operations IrDA_kdrv_fops = {
	.owner	 = THIS_MODULE,
	.open	 = irda_kdrv_fops_open,
	.release = irda_kdrv_fops_close,
	.mmap	 = irda_kdrv_fops_mmap,
	.ioctl	 = irda_kdrv_fops_ioctl,
	.write	 = irda_kdrv_fops_write,
	.poll	 = irda_kdrv_fops_poll,
	.read	 = irda_kdrv_fops_read
};

static void irda_kdrv_init_control(void)
{
	IrDA_kdrv.state = IRDA_KDRV_STATUS_CLOSE;

	IrDA_kdrv.qos_info.baud_rate = IRDA_BAUD_9600;
	IrDA_kdrv.qos_info.connection_address = IRDA_KDRV_DEF_CA;
	IrDA_kdrv.qos_info.add_bof = IRDA_KDRV_DEF_BOF;
	IrDA_kdrv.qos_info.mpi = IRDA_KDRV_DEF_MPI;
	IrDA_kdrv.qos_info.mtt = IRDA_KDRV_DEF_MTT;

	IrDA_kdrv.led_st = IRDA_KDRV_LED_ST_NON;

	IrDA_kdrv.send_mode = IRDA_KDRV_SEND_NORMAL;

	IrDA_kdrv.fifo_write_cnt = 0;

	IrDA_kdrv.first_send = false;

	IrDA_kdrv.poll = false;

	IrDA_kdrv_wu_que.wp = 0;
	IrDA_kdrv_wu_que.rp = 0;
	memset(&IrDA_kdrv_wu_que.event, 0x00, sizeof(IrDA_kdrv_wu_que.event));
}

static int irda_kdrv_init_gpio(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;

	ret = gpio_tlmm_config(
		GPIO_CFG(IRDA_INT_GPIO_NO, 0, GPIO_INPUT, GPIO_NO_PULL,
							GPIO_2MA),
							GPIO_ENABLE);
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		MSG_IRDRV_FATAL("gpio_tlmm_config No.%d error = %d\n",
						IRDA_INT_GPIO_NO, ret);
		ret = -ENXIO;



	} else {
		ret = gpio_tlmm_config(
			GPIO_CFG(IR_QSD_LEDA_EN, 0, GPIO_OUTPUT,
							GPIO_PULL_DOWN,
							GPIO_2MA),
							GPIO_ENABLE);
		if (ret != IRDA_KERNEL_API_SUCCESS) {
			MSG_IRDRV_FATAL(
				"gpio_tlmm_config No.%d error = %d\n",
							IR_QSD_LEDA_EN, ret);
			ret = -ENXIO;
		}


	}

	return (ret);
}

static void irda_kdrv_init_kernel_func(void)
{
	spin_lock_init(&IrDA_kdrv_spin_lock);

	init_MUTEX(&IrDA_kdrv_sem);

	init_waitqueue_head(&IrDA_kdrv_read_wakeup);
	init_waitqueue_head(&IrDA_kdrv_poll_wakeup);
}

static int irda_kdrv_init_func(void)
{
	int ret = IRDA_KERNEL_API_SUCCESS;


	int memalloc_ret = IRDA_MEM_OK;

	memalloc_ret = irda_kdrv_memory_alloc();
	if (memalloc_ret != IRDA_MEM_OK) {
		MSG_IRDRV_ERR("memory alloc = %d\n", memalloc_ret);
		if (memalloc_ret == IRDA_MEM_ERR_KMALLOC) {
			ret = -ENOMEM;
		} else {
			ret = -ENXIO;
		}
		goto error;
	}




		ret = irda_kdrv_init_gpio();

		if (ret == IRDA_KERNEL_API_SUCCESS) {
			irda_kdrv_init_kernel_func();

			irda_kdrv_init_control();
		}




error:
	return (ret);
}

static ssize_t irda_kdrv_ver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	result = scnprintf(buf, PAGE_SIZE, "%s\n",
						SH_IRDA_KERNEL_DRIVER_VERSION);

	return result;
}

static struct device_attribute irda_kdrv_ver_attr = {
	.attr = {
		.name = "version",
		.mode = S_IRUGO,
		.owner = THIS_MODULE },
	.show = irda_kdrv_ver_show,
	.store = NULL,
};

static ssize_t irda_kdrv_device_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	result = scnprintf(buf, PAGE_SIZE, "%s\n", IRDA_DEVICE_NAME);

	return result;
}

static struct device_attribute irda_kdrv_device_name_attr = {
	.attr = {
		.name = "name",
		.mode = S_IRUGO,
		.owner = THIS_MODULE },
	.show = irda_kdrv_device_name_show,
	.store = NULL,
};

static ssize_t irda_kdrv_capability_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t result;

	result = scnprintf(buf, PAGE_SIZE, "%s\n%s\n%s\n%s\n%s\n",
						IRDA_CAPABILITY_SIR_BAUDRATE,
						IRDA_CAPABILITY_FIR_BAUDRATE,
							IRDA_CAPABILITY_ABOF,
							IRDA_CAPABILITY_MPI,
							IRDA_CAPABILITY_MTT);

	return result;
}

static struct device_attribute irda_kdrv_capability_attr = {
	.attr = {
		.name = "capability",
		.mode = S_IRUGO,
		.owner = THIS_MODULE },
	.show = irda_kdrv_capability_show,
	.store = NULL,
};

static int __devinit irda_kdrv_platform_driver_init(
						struct platform_device *pdev)
{
	int ret = IRDA_KERNEL_API_SUCCESS;
	struct device *dev_create;
	dev_t dev;
	int alloc_ret = 0;
	int cdev_ret = 0;

	IrDA_kdrv_major = 0;
	dev = MKDEV(IrDA_kdrv_major, 0);
	alloc_ret = alloc_chrdev_region(
				&dev,
				IRDA_KDRV_BASE_MINOR,
				IRDA_KDRV_DEVS,
				IRDA_DEVFILE_NAME);
	if (alloc_ret != IRDA_KERNEL_API_SUCCESS) {
		ret = -EIO;
		MSG_IRDRV_FATAL("alloc_chrdev_region error\n");
		goto error_chrdev_region;
	}

	IrDA_kdrv_major = MAJOR(dev);

	cdev_init(&IrDA_kdrv_cdev, &IrDA_kdrv_fops);
	IrDA_kdrv_cdev.owner = THIS_MODULE;
	IrDA_kdrv_cdev.ops = &IrDA_kdrv_fops;

	cdev_ret = cdev_add(&IrDA_kdrv_cdev,
					MKDEV(IrDA_kdrv_major, IRDA_KDRV_MINOR),
							IRDA_KDRV_MINOR_COUNT);
	if (cdev_ret != IRDA_KERNEL_API_SUCCESS) {
		ret = -EIO;
		MSG_IRDRV_FATAL("cdev_add error\n");
		goto error_cdev_add;
	}

	IrDA_kdrv_class = class_create(THIS_MODULE, IRDA_KDRV_CLASS_NAME);
	if (IS_ERR(IrDA_kdrv_class)) {
		ret = -ENOMEM;
		MSG_IRDRV_FATAL("class_create error\n");
		goto error_class_create;
	}

	IrDA_kdrv_dev = MKDEV(IrDA_kdrv_major, IRDA_KDRV_MINOR);
	dev_create = device_create(IrDA_kdrv_class,
			NULL,
			IrDA_kdrv_dev,
			NULL,
			IRDA_DEVFILE_NAME);

	if (IS_ERR(dev_create)) {
		ret = -ENODEV;
		MSG_IRDRV_FATAL("device_create error\n");
		goto error_device_create;
	}

	ret = irda_kdrv_init_func();
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		goto error_irdrv_init_func;
	}


	ret = device_create_file(&pdev->dev, &irda_kdrv_ver_attr);
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		MSG_IRDRV_FATAL("version attr create error = %d\n", ret);
	}

	ret = device_create_file(&pdev->dev, &irda_kdrv_device_name_attr);
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		MSG_IRDRV_FATAL("device name attr create error = %d\n", ret);
	}

	ret = device_create_file(&pdev->dev, &irda_kdrv_capability_attr);
	if (ret != IRDA_KERNEL_API_SUCCESS) {
		MSG_IRDRV_FATAL("capability attr create error = %d\n", ret);
	}

	return (ret);

error_irdrv_init_func:
	device_destroy(IrDA_kdrv_class, IrDA_kdrv_dev);

error_device_create:
	class_destroy(IrDA_kdrv_class);

error_class_create:
	cdev_del(&IrDA_kdrv_cdev);

error_cdev_add:
	unregister_chrdev_region(dev, IRDA_KDRV_DEVS);

error_chrdev_region:

	return (ret);

}

static int __devexit irda_kdrv_platform_driver_remove(
						struct platform_device *pdev)
{

	dev_t dev;
	dev = MKDEV(IrDA_kdrv_major, IRDA_KDRV_BASE_MINOR);

	device_remove_file(&pdev->dev, &irda_kdrv_ver_attr);

	device_remove_file(&pdev->dev, &irda_kdrv_device_name_attr);

	device_remove_file(&pdev->dev, &irda_kdrv_capability_attr);

	device_destroy(IrDA_kdrv_class, IrDA_kdrv_dev);

	class_destroy(IrDA_kdrv_class);

	cdev_del(&IrDA_kdrv_cdev);

	unregister_chrdev_region(dev, IRDA_KDRV_DEVS);

	irda_kdrv_memory_free();

	return (IRDA_KERNEL_API_SUCCESS);
}

struct platform_driver irda_kdrv_pdrv = {
	.remove		= __devexit_p(irda_kdrv_platform_driver_remove),
	.driver = {
		.name  = IRDA_DEVFILE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init irda_kdrv_module_init(void)
{
	int retval = IRDA_KERNEL_API_SUCCESS;

	irda_kdrv_pdev = platform_device_alloc(
					(const char *)IRDA_DEVFILE_NAME,
							(unsigned int) -1);
	if (irda_kdrv_pdev == NULL) {
		MSG_IRMOD_FATAL("platform_device_alloc error\n");
		retval = -ENODEV;
	}

	if (retval == IRDA_KERNEL_API_SUCCESS) {
		retval = platform_device_add(irda_kdrv_pdev);
		if (retval != IRDA_KERNEL_API_SUCCESS) {
			platform_device_put(irda_kdrv_pdev);

			MSG_IRMOD_FATAL(
				"platform_device_add error = %d\n", retval);
		}
	}

	if (retval == IRDA_KERNEL_API_SUCCESS) {
		retval = platform_driver_probe(&irda_kdrv_pdrv,
						irda_kdrv_platform_driver_init);
		if (retval != IRDA_KERNEL_API_SUCCESS) {
			platform_device_unregister(irda_kdrv_pdev);

			MSG_IRMOD_FATAL(
			"platform_driver_probe error = %d\n", retval);
		}
	}

	return (retval);
}

static void __exit irda_kdrv_module_exit(void)
{
	platform_device_unregister(irda_kdrv_pdev);
	platform_driver_unregister(&irda_kdrv_pdrv);
}

module_init(irda_kdrv_module_init);
module_exit(irda_kdrv_module_exit);

MODULE_DESCRIPTION("IrDA SIR/FIR driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION(SH_IRDA_KERNEL_DRIVER_VERSION);
