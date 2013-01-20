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

#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <mach/board.h>

#include <sharp/shpem_kerl.h>

typedef signed char shpem_result_t;

enum
{
	SHPEM_SUCCESS            = 0,
	SHPEM_NG                 = -1,
	SHPEM_COM_ERROR          = -2,
	SHPEM_PARAM_ERROR        = -3,
	SHPEM_EXCLUSIVE_ERROR    = -4,
	SHPEM_INIT_ERROR         = -5,
	SHPEM_BREAK_MICON_ERROR  = -6,
	SHPEM_BREAK_SENSOR_ERROR = -7,
	SHPEM_SEQUENCE_ERROR     = -8,
	SHPEM_SLEEP_ERROR        = -9,
	SHPEM_VERIFY_ERROR       = -10,
	SHPEM_IO_ERROR           = -11,
};

enum
{
	SHPEM_CMD_PVER,
	SHPEM_CMD_WPSN,
	SHPEM_CMD_RPSN,
	SHPEM_CMD_CLSP,
	SHPEM_CMD_RDSP,
	SHPEM_CMD_SJGM,
	SHPEM_CMD_RDMS,
	SHPEM_CMD_SMIR,
	SHPEM_CMD_RMIR,
	SHPEM_CMD_TMES,
	SHPEM_CMD_SLEP,
	SHPEM_CMD_TTES,
	SHPEM_CMD_SCMS,
	SHPEM_CMD_DSST,
	SHPEM_CMD_DSTR,
	SHPEM_CMD_DSEN,
	SHPEM_CMD_DSAR,
	SHPEM_CMD_DSAD,
	SHPEM_CMD_ADSR,
	SHPEM_CMD_BTST,
	SHPEM_CMD_SOOF,
	SHPEM_CMD_KIND,
	SHPEM_CMD_DSSD,
	SHPEM_CMD_DSLG,
	SHPEM_CMD_DSHZ,
	SHPEM_CMD_SOAR,
	SHPEM_CMD_SOAC,
	SHPEM_CMD_SHDN,
	SHPEM_CMD_INIT,
	SHPEM_CMD_SENS,
	SHPEM_CMD_SENE,
	SHPEM_CMD_DSCS,
	SHPEM_CMD_DSCE,
	SHPEM_CMD_RESET,
	SHPEM_CMD_NUM,
};

enum
{
	SHPEM_ISR_ID_BREAK_PEDO,
	SHPEM_ISR_ID_BREAK_ACCEL,
	SHPEM_ISR_ID_BREAK_DISP,
	SHPEM_ISR_ID_NUM,
};

enum
{
	SHPEM_MICON_SLEEP_PEDO,
	SHPEM_MICON_SLEEP_ACCEL,
	SHPEM_MICON_SLEEP_DIAG,
	SHPEM_MICON_SLEEP_NUM,
};

enum
{
	SHPEM_MICON_SLEEP,
	SHPEM_MICON_WAKEUP,
};

enum
{
	SHPEM_MICON_SENSOR_ACCEL,
	SHPEM_MICON_SENSOR_DIAG,
	SHPEM_MICON_SENSOR_NUM,
};

enum
{
	SHPEM_MICON_SENSOR_ON,
	SHPEM_MICON_SENSOR_OFF,
};

#define SHPEM_FW_WRITE_LOOP 6

typedef struct shpem_com_param_tag
{
	unsigned char cmd_id;
	unsigned char* send_buf;
	unsigned char* recv_buf;
} shpem_com_param_t;

/* Compile Switch */
#define SHPEM_SHTERM_ENABLE
#define SHPEM_SH_I2C_ENABLE

#ifdef SHPEM_SHTERM_ENABLE
#include <sharp/shterm_k.h>
#endif	/* SHPEM_SHTERM_ENABLE */

#ifdef SHPEM_SH_I2C_ENABLE
#include <sharp/sh_i2c.h>
#endif	/* SHPEM_SH_I2C_ENABLE */

#define SHPEM_LOG_TAG "SHPEMkerl"
#define  SHPEM_DEBUG_LOG_ENABLE_1
//#define  SHPEM_DEBUG_LOG_ENABLE_2
//#define  SHPEM_DEBUG_LOG_ENABLE_SENDBUF
//#define  SHPEM_DEBUG_LOG_ENABLE_READBUF

/* about Debug Log */
#ifdef SHPEM_DEBUG_LOG_ENABLE_1
#define SHPEM_DEBUG_LOG_1(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHPEM_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHPEM_DEBUG_LOG_1(fmt, args...)
#endif

#ifdef SHPEM_DEBUG_LOG_ENABLE_2
#define SHPEM_DEBUG_LOG_2(fmt, args...)	printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHPEM_LOG_TAG, __func__, __LINE__, ## args)
#else
#define SHPEM_DEBUG_LOG_2(fmt, args...)
#endif

/* Macro */
#define MAKE_UNSIGNED_SHORT(a, b)	((unsigned short)( ((((unsigned short)a) <<   8)& 0xFF00) | \
													   (( (unsigned short)b)        & 0x00FF) ) )

/* about GPIOs */
#define SHPEM_GPIO_NUM_FW_CTL	116
#define SHPEM_GPIO_NUM_COMREQ	127
#define SHPEM_GPIO_NUM_COMRDY	128
#define SHPEM_GPIO_NUM_INT		37
#define SHPEM_GPIO_NUM_RESET	119

#define GPIO_HIGH				1
#define GPIO_LOW				0

#define SHPEM_GPIO_REQ_FW_CTL	SHPEM_GPIO_NUM_FW_CTL, "gpio_shpem_fw_ctl"
#define SHPEM_GPIO_REQ_COMREQ	SHPEM_GPIO_NUM_COMREQ, "gpio_shpem_comreq"
#define SHPEM_GPIO_REQ_COMRDY	SHPEM_GPIO_NUM_COMRDY, "gpio_shpem_comrdy"
#define SHPEM_GPIO_REQ_INT		SHPEM_GPIO_NUM_INT,    "gpio_shpem_int"
#define SHPEM_GPIO_REQ_RESET	SHPEM_GPIO_NUM_RESET,  "gpio_shpem_reset"

#define SHPEM_GPIO_CNF_FW_CTL	GPIO_CFG(SHPEM_GPIO_NUM_FW_CTL, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE
#define SHPEM_GPIO_CNF_COMREQ	GPIO_CFG(SHPEM_GPIO_NUM_COMREQ, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE
#define SHPEM_GPIO_CNF_COMRDY	GPIO_CFG(SHPEM_GPIO_NUM_COMRDY, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE
#define SHPEM_GPIO_CNF_INT		GPIO_CFG(SHPEM_GPIO_NUM_INT   , 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE
#define SHPEM_GPIO_CNF_RESET	GPIO_CFG(SHPEM_GPIO_NUM_RESET , 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE

#define SHPEM_FW_CTL_HIGH	gpio_direction_output(SHPEM_GPIO_NUM_FW_CTL, GPIO_HIGH)
#define SHPEM_FW_CTL_LOW	gpio_direction_output(SHPEM_GPIO_NUM_FW_CTL, GPIO_LOW)
#define SHPEM_COMREQ_HIGH	gpio_direction_output(SHPEM_GPIO_NUM_COMREQ, GPIO_HIGH)
#define SHPEM_COMREQ_LOW	gpio_direction_output(SHPEM_GPIO_NUM_COMREQ, GPIO_LOW)
#define SHPEM_RESET_HIGH	gpio_direction_output(SHPEM_GPIO_NUM_RESET, GPIO_HIGH)
#define SHPEM_RESET_LOW		gpio_direction_output(SHPEM_GPIO_NUM_RESET, GPIO_LOW)

#define SHPEM_IRQ_COMRDY		MSM_GPIO_TO_INT(SHPEM_GPIO_NUM_COMRDY)
#define SHPEM_IRQ_INT			MSM_GPIO_TO_INT(SHPEM_GPIO_NUM_INT)

#define SHPEM_IRQ_COMRDY_HIGH		request_irq(SHPEM_IRQ_COMRDY, &shpem_com_irqhandler, \
												 IRQF_TRIGGER_HIGH | IRQF_DISABLED,      \
											     SHPEM_I2C_DEVNAME, NULL)
#define SHPEM_IRQ_COMRDY_LOW		request_irq(SHPEM_IRQ_COMRDY, &shpem_com_irqhandler, \
												 IRQF_TRIGGER_LOW | IRQF_DISABLED,       \
											     SHPEM_I2C_DEVNAME, NULL)
#define SHPEM_IRQ_COMRDY_FREE		free_irq(SHPEM_IRQ_COMRDY, NULL)
#define SHPEM_IRQ_INT_HIGH			request_irq(SHPEM_IRQ_INT, &shpem_int_irqhandler,    \
												 IRQF_TRIGGER_HIGH | IRQF_DISABLED,      \
											     SHPEM_I2C_DEVNAME, NULL)
#define SHPEM_IRQ_INT_FREE			free_irq(SHPEM_IRQ_INT, NULL)

#define SHPEM_IRQ_COMRDY_ENABLE		enable_irq(SHPEM_IRQ_COMRDY)
#define SHPEM_IRQ_COMRDY_DISABLE	disable_irq_nosync(SHPEM_IRQ_COMRDY)
#define SHPEM_IRQ_INT_ENABLE		enable_irq(SHPEM_IRQ_INT)
#define SHPEM_IRQ_INT_DISABLE		disable_irq_nosync(SHPEM_IRQ_INT)

/* about COM */
#define SHPEM_COM_RETRY				3
#define SHPEM_COM_TIMEOUT_OPEN		msecs_to_jiffies(20)
#define SHPEM_COM_TIMEOUT_WRITE_END	msecs_to_jiffies(20)
#define SHPEM_COM_TIMEOUT_READ		msecs_to_jiffies(100)
#define SHPEM_COM_TIMEOUT_READ_SCMS	msecs_to_jiffies(750)
#define SHPEM_COM_TIMEOUT_CLOSE		msecs_to_jiffies(100)

/* about Work Queue */
#define SHPEM_WORKQUEUE_NAME	"shpem"

/* about Break */
#define SHPEM_RESET_COUNT	2
enum
{
	SHPEM_NORMAL,
	SHPEM_BREAK_SENSOR,
	SHPEM_BREAK_MICON,
	SHPEM_FIRM_WRITE,
};

/* about I2C buffer */
#define SHPEM_MAX_SEND_BUF	135
#define SHPEM_MAX_RECV_BUF	129

/* about Command */
const shpem_command_t shpem_command[] = 
{
	{"PVER",										"PVERxxxxr",								"ERORxxxxx"},
	{"WPSNxxxxs",									"WPSN",										"EROR"},
	{"RPSN",										"RPSNxxxxr",								"ERORxxxxx"},
	{"CLSP",										"CLSP",										"EROR"},
	{"RDSP",										"RDSPxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxr",	"ERORxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"},
	{"SJGMxs",										"SJGM",										"EROR"},
	{"RDMS",										"RDMSxxxxxxr",								"ERORxxxxxxx"},
	{"SMIRxxxxxxxxs",								"SMIR",										"EROR"},
	{"RMIR",										"RMIRxxxxxr",								"ERORxxxxxx"},
	{"TMES",										"TMES",										"EROR"},
	{"SLEP",										"SLEP",										"EROR"},
	{"TTES",										"TTES",										"EROR"},
	{"SCMSxxxxxxxxxxxxxxs",							"SCMSxr",									"ERORxx"},
	{"DSST",										"DSST",										"EROR"},
	{"DSTR",										"DSTR",										"EROR"},
	{"DSEN",										"DSEN",										"EROR"},
	{"DSARxxxs",									"DSAR",										"EROR"},
	{"DSADxs",										"DSAD",										"EROR"},
	{"ADSR",										"ADSRxxxxxxxxr",							"ERORxxxxxxxxx"},
	{"BTSTxs",										"BTST",										"EROR"},
	{"SOOFxs",										"SOOF",										"EROR"},
	{"KIND",										"KINDxr",									"ERORxx"},
	{"DSSDxs",										"DSSD",										"EROR"},
	{"DSLGxs",										"DSLG",										"EROR"},
	{"DSHZxs",										"DSHZ",										"EROR"},
	{"SOARxs",										"SOARxxxxr",								"ERORxxxxx"},
	{"SOACxs",										"SOAC",										"EROR"},
	{"SHDN",										"SHDN",										"EROR"},
	{"INIT",										"INIT",										"EROR"},
	{"SENS",										"SENS",										"EROR"},
	{"SENE",										"SENE",										"EROR"},
	{"DSCSxxxs",									"DSCS",										"EROR"},
	{"DSCE",										"DSCE",										"EROR"},
};

static shpem_result_t shpem_com_kerl(shpem_com_param_t* param);

static struct semaphore shpem_sem;
static struct semaphore shpem_ioctl_sem;
static struct semaphore shpem_sleep_sem;
static unsigned char shpem_micon_sleep = 0x00;
static unsigned char shpem_micon_sleep_f = SHPEM_MICON_SLEEP;
static struct semaphore shpem_sensor_sem;
static unsigned char shpem_micon_sensor = 0x00;
static unsigned char shpem_micon_sensor_f = SHPEM_MICON_SENSOR_OFF;
static struct workqueue_struct *shpem_wq;
static struct work_struct shpem_work;
static struct i2c_client *shpem_i2c_client;
static DECLARE_WAIT_QUEUE_HEAD(shpem_com_wq);
static DECLARE_WAIT_QUEUE_HEAD(shpem_int_wq);
static atomic_t shpem_com_flag;
static atomic_t shpem_int_flag;
static unsigned char shpem_int_type = 0x00;
static unsigned char shpem_reset_f = 0;
static unsigned char shpem_reset_retry_count = 0;
static unsigned char shpem_state = SHPEM_NORMAL;
static unsigned char shpem_init_f = 1;
static unsigned char shpem_disp_break_f = 0;
static unsigned char shpem_shutdown_f = 0;


static int shpem_i2c_write(unsigned char *buf, int len)
{
	int ret;
	struct i2c_msg msg[] =
	{
		{
		 .addr = shpem_i2c_client->addr,
		 .flags = 0,
		 .len = len,
		 .buf = buf,
		 },
	};
	
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_DEBUG_LOG_2("addr = 0x%02X", shpem_i2c_client->addr);
	
	ret = i2c_transfer(shpem_i2c_client->adapter, msg, 1);
	if(ret >= 0)
	{
		return SHPEM_SUCCESS;
	}
	else
	{
		SHPEM_DEBUG_LOG_1("i2c_transfer(1) Error [%d]", ret);
	}
	
	return SHPEM_COM_ERROR;
}

static int shpem_i2c_read(unsigned char *buf, int len)
{
	int ret;
	struct i2c_msg msg[] =
	{
		{
		 .addr = shpem_i2c_client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};
	
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_DEBUG_LOG_2("addr = 0x%02X", shpem_i2c_client->addr);
	
	ret = i2c_transfer(shpem_i2c_client->adapter, msg, 1);
	if(ret >= 0)
	{
		return SHPEM_SUCCESS;
	}
	else
	{
		SHPEM_DEBUG_LOG_1("i2c_transfer(2) Error [%d]", ret);
	}
	
	return SHPEM_COM_ERROR;
}

static void shpem_micon_reset(void)
{
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_RESET_LOW;
	
	msleep(1);
	
	SHPEM_RESET_HIGH;
	
	SHPEM_FW_CTL_LOW;
	
	msleep(5);
	
	SHPEM_RESET_LOW;
	
	msleep(500);
	
	shpem_micon_sleep_f = SHPEM_MICON_SLEEP;
#ifdef SHPEM_SHTERM_ENABLE
	if(shpem_init_f == 0)
	{
		SHPEM_DEBUG_LOG_2("shterm_k_set_info(0)");
		if(shterm_k_set_info(SHTERM_INFO_ACCELE, 0) != SHTERM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shterm_k_set_info(0) Error");
		}
	}
#endif	/* SHPEM_SHTERM_ENABLE */
	shpem_micon_sensor_f = SHPEM_MICON_SENSOR_OFF;
}

static shpem_result_t shpem_init_send(void)
{
	shpem_result_t ret;
	shpem_com_param_t param;
	unsigned char write_buf[4] = {'I', 'N', 'I', 'T'};
	unsigned char read_buf[4] = {0};
	
	SHPEM_DEBUG_LOG_2();
	
	param.cmd_id = SHPEM_CMD_INIT;
	param.send_buf = write_buf;
	param.recv_buf = read_buf;
	
	ret = shpem_com_kerl(&param);
	if(ret != SHPEM_SUCCESS)
	{
		shpem_disp_break_f = 1;
		SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d Disp Break Flag ON", ret);
	}
	else
	{
		shpem_disp_break_f = 0;
	}
	
	return ret;
}

static shpem_result_t shpem_shdn_send(void)
{
	shpem_result_t ret;
	shpem_com_param_t param;
	unsigned char write_buf[4] = {'S', 'H', 'D', 'N'};
	unsigned char read_buf[4] = {0};
	
	SHPEM_DEBUG_LOG_2();
	
	param.cmd_id = SHPEM_CMD_SHDN;
	param.send_buf = write_buf;
	param.recv_buf = read_buf;
	
	ret = shpem_com_kerl(&param);
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
	}
	
	return ret;
}

static irqreturn_t shpem_com_irqhandler(int irq, void *devid)
{
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_IRQ_COMRDY_DISABLE;
	
	atomic_set( &shpem_com_flag, 1 );
	
	wake_up(&shpem_com_wq);
	
	return IRQ_HANDLED;
}

static irqreturn_t shpem_int_irqhandler(int irq, void *devid)
{
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_IRQ_INT_DISABLE;
	
	queue_work(shpem_wq, &shpem_work);
	
	return IRQ_HANDLED;
}

static void shpem_micon_restor(void)
{
	SHPEM_DEBUG_LOG_2();
	
	if(shpem_reset_retry_count <= SHPEM_RESET_COUNT-2)
	{
		shpem_reset_f = 1;
		
		SHPEM_IRQ_INT_DISABLE;
		
		queue_work(shpem_wq, &shpem_work);
	}
}

static void shpem_wq_handler(struct work_struct *poWork)
{
	shpem_result_t ret;
	shpem_com_param_t param;
	unsigned char write_buf[4] = {'R', 'M', 'I', 'R'};
	unsigned char read_buf[10] = {0};
	
	SHPEM_DEBUG_LOG_2();
	
	param.send_buf = write_buf;
	param.recv_buf = read_buf;
	
	if(shpem_reset_f == 1)
	{
		SHPEM_DEBUG_LOG_1("reset_f ON");
		
		shpem_micon_reset();
		param.cmd_id = SHPEM_CMD_RESET;
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			if(ret == SHPEM_COM_ERROR)
			{
				shpem_reset_retry_count++;
				SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d Reset Retry %d", ret, shpem_reset_retry_count);
				if(shpem_reset_retry_count >= SHPEM_RESET_COUNT)
				{
					SHPEM_DEBUG_LOG_1("Micon Break!!");
					shpem_state = SHPEM_BREAK_MICON;
					shpem_reset_retry_count = 0;
				}
			}
			else
			{
				SHPEM_DEBUG_LOG_1("Break Reset Cancel ret = %d", ret);
				shpem_reset_retry_count = 0;
				shpem_reset_f = 0;
			}
		}
		else
		{
			if( (param.recv_buf[4] & SHPEM_ISR_BREAK_PORT) != 0)
			{
				shpem_reset_retry_count++;
				SHPEM_DEBUG_LOG_1("SHPEM_ISR_BREAK_PORT Reset Retry %d", shpem_reset_retry_count);
				if( (shpem_state == SHPEM_BREAK_SENSOR) || (shpem_reset_retry_count >= SHPEM_RESET_COUNT) )
				{
					SHPEM_DEBUG_LOG_1("Sensor Break!!");
					shpem_state = SHPEM_BREAK_SENSOR;
					shpem_reset_retry_count = 0;
					shpem_init_f = 1;
					shpem_reset_f = 0;
					ret = shpem_init_send();
					if(ret != SHPEM_SUCCESS)
					{
						SHPEM_DEBUG_LOG_1("shpem_init_send Error %d", ret);
					}
					shpem_init_f = 0;
				}
			}
			else
			{
				SHPEM_DEBUG_LOG_2("Micon & Sensor Normal");
				shpem_state = SHPEM_NORMAL;
				shpem_reset_retry_count = 0;
				shpem_init_f = 1;
				shpem_reset_f = 0;
				ret = shpem_init_send();
				if(ret != SHPEM_SUCCESS)
				{
					SHPEM_DEBUG_LOG_1("shpem_init_send Error %d", ret);
				}
				shpem_init_f = 0;
				SHPEM_IRQ_INT_FREE;
				if(SHPEM_IRQ_INT_HIGH < 0)
				{
					SHPEM_DEBUG_LOG_1("request_irq Error");
				}
			}
		}
	}
	else
	{
		param.cmd_id = SHPEM_CMD_RMIR;
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
		}
		else
		{
			if( (param.recv_buf[4] & SHPEM_ISR_BREAK_PORT) != 0)
			{
			}
			else
			{
				SHPEM_IRQ_INT_ENABLE;
			}
		}
	}
}

static unsigned char shpem_cal_checksum(unsigned char *cmd, unsigned short len)
{
	unsigned short count;
	unsigned char sum = 0;

	for(count = 0; count < len; count++)
	{
		sum += (unsigned char)cmd[count];
	}
	return sum;
}

static shpem_result_t shpem_send_check(shpem_com_param_t* param)
{
	int count, len, sum;
	unsigned short size;
	
	if( (param->send_buf == NULL) || (param->cmd_id >= SHPEM_CMD_NUM) )
	{
		return SHPEM_PARAM_ERROR;
	}
	
	if(param->cmd_id == SHPEM_CMD_RESET)
	{
		return SHPEM_SUCCESS;
	}
	
	if(param->cmd_id == SHPEM_CMD_DSTR)
	{
		for(count=0;count<4;count++)
		{
			if(param->send_buf[count] != shpem_command[param->cmd_id].send_cmd[count])
			{
				SHPEM_DEBUG_LOG_1("Command Error");
				return SHPEM_PARAM_ERROR;
			}
		}
		size = MAKE_UNSIGNED_SHORT(param->send_buf[8], param->send_buf[9]) + 10;
		if(param->send_buf[size] != shpem_cal_checksum(param->send_buf, size))
		{
			SHPEM_DEBUG_LOG_1("checksum Error send = 0x%02X != 0x%02X", param->send_buf[size], shpem_cal_checksum(param->send_buf, size));
			return SHPEM_PARAM_ERROR;
		}
	}
	else
	{
		len = strlen((char*)shpem_command[param->cmd_id].send_cmd);
		sum = shpem_cal_checksum(param->send_buf, len-1);
		
		for(count=0;count<len;count++)
		{
			switch(shpem_command[param->cmd_id].send_cmd[count])
			{
			case 'x':
				break;
			case 's':
				if(param->send_buf[count] != sum)
				{
					SHPEM_DEBUG_LOG_1("checksum Error send = 0x%02X != 0x%02X", param->send_buf[count], sum);
					return SHPEM_PARAM_ERROR;
				}
				break;
			default:
				if(param->send_buf[count] != shpem_command[param->cmd_id].send_cmd[count])
				{
					SHPEM_DEBUG_LOG_1("Command Error");
					return SHPEM_PARAM_ERROR;
				}
				break;
			}
		}
	}
	return SHPEM_SUCCESS;
}

static shpem_result_t shpem_recv_check(shpem_com_param_t* param)
{
	shpem_result_t ret;
	unsigned char* ref_buf;
	int recv_len, count;
	
	if(param == NULL)
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	if( (param->cmd_id >= SHPEM_CMD_NUM) || (param->send_buf == NULL) || (param->recv_buf == NULL) )
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	if(param->recv_buf[0] == shpem_command[param->cmd_id].recv_ok[0])
	{
		ref_buf = (unsigned char*)(shpem_command[param->cmd_id].recv_ok);
		ret = SHPEM_SUCCESS;
	}
	else if(param->recv_buf[0] == shpem_command[param->cmd_id].recv_ng[0])
	{
		ref_buf = (unsigned char*)(shpem_command[param->cmd_id].recv_ng);
		ret = SHPEM_NG;
	}
	else
	{
		SHPEM_DEBUG_LOG_1("SHPEM_VERIFY_ERROR");
		return SHPEM_VERIFY_ERROR;
	}
	
	recv_len = strlen((char*)ref_buf);
	
	for(count = 0; count < recv_len; count++)
	{
		switch(ref_buf[count])
		{
			case 'x':
				break;
			case 'r':
				if(param->recv_buf[count] != (shpem_cal_checksum(param->recv_buf, recv_len-1)))
				{
					SHPEM_DEBUG_LOG_1("SHPEM_VERIFY_ERROR");
					return SHPEM_VERIFY_ERROR;
				}
				break;
			default:
				if(param->recv_buf[count] != ref_buf[count])
				{
					SHPEM_DEBUG_LOG_1("SHPEM_VERIFY_ERROR");
					return SHPEM_VERIFY_ERROR;
				}
				break;
		}
	}
	
	return ret;
}

static int shpem_sleep_cmd_check(unsigned char cmd_id)
{
	switch(cmd_id)
	{
		case SHPEM_CMD_RDMS:
		case SHPEM_CMD_SCMS:
		case SHPEM_CMD_SENS:
		case SHPEM_CMD_SENE:
			return -1;
		default:
			return 0;
	}
}

static int shpem_sensor_cmd_check(unsigned char cmd_id)
{
	switch(cmd_id)
	{
		case SHPEM_CMD_PVER:
		case SHPEM_CMD_RMIR:
		case SHPEM_CMD_DSST:
		case SHPEM_CMD_DSTR:
		case SHPEM_CMD_DSEN:
		case SHPEM_CMD_KIND:
		case SHPEM_CMD_DSHZ:
		case SHPEM_CMD_SHDN:
		case SHPEM_CMD_INIT:
		case SHPEM_CMD_RESET:
			return 0;
		default:
			return -1;
	}
}

static int shpem_disp_cmd_check(unsigned char cmd_id)
{
	switch(cmd_id)
	{
		case SHPEM_CMD_DSST:
		case SHPEM_CMD_DSTR:
		case SHPEM_CMD_DSEN:
		case SHPEM_CMD_DSAR:
		case SHPEM_CMD_DSHZ:
		case SHPEM_CMD_DSCS:
		case SHPEM_CMD_DSCE:
			return 0;
		default:
			return -1;
	}
}

static int shpem_break_cmd_check(unsigned char cmd_id)
{
	switch(cmd_id)
	{
		case SHPEM_CMD_DSST:
		case SHPEM_CMD_DSEN:
		case SHPEM_CMD_DSAR:
		case SHPEM_CMD_DSHZ:
		case SHPEM_CMD_DSCS:
		case SHPEM_CMD_RDMS:
		case SHPEM_CMD_SENS:
		case SHPEM_CMD_SENE:
			return 0;
		default:
			return -1;
	}
}

static shpem_result_t shpem_com_open(shpem_com_param_t* param)
{
	long timeout;
	
	SHPEM_DEBUG_LOG_2();
	
	atomic_set(&shpem_com_flag, 0);
#ifdef SHPEM_SH_I2C_ENABLE
	sh_i2c_mutex_lock(shpem_i2c_client);
#endif	/* SHPEM_SH_I2C_ENABLE */
	
	SHPEM_COMREQ_HIGH;
	
	if(SHPEM_IRQ_COMRDY_HIGH < 0)
	{
#ifdef SHPEM_SH_I2C_ENABLE
		sh_i2c_mutex_free(shpem_i2c_client);
#endif	/* SHPEM_SH_I2C_ENABLE */
		SHPEM_DEBUG_LOG_1("request_irq Error");
		return SHPEM_COM_ERROR;
	}
	
	timeout = wait_event_timeout(shpem_com_wq, (atomic_read(&shpem_com_flag) != 0), SHPEM_COM_TIMEOUT_OPEN);
	SHPEM_IRQ_COMRDY_FREE;
#ifdef SHPEM_SH_I2C_ENABLE
	sh_i2c_mutex_free(shpem_i2c_client);
#endif	/* SHPEM_SH_I2C_ENABLE */
	if(timeout <= 0)
	{
		if(atomic_read(&shpem_com_flag) != 0)
		{
			SHPEM_DEBUG_LOG_2("COMRDY High");
		}
		else
		{
			SHPEM_DEBUG_LOG_1("open timeout %ld", timeout);
			return SHPEM_COM_ERROR;
		}
	}
	
	return SHPEM_SUCCESS;
}

static shpem_result_t shpem_com_write(shpem_com_param_t* param)
{
	int ret;
	
	SHPEM_DEBUG_LOG_2();
	
	if(param->cmd_id == SHPEM_CMD_DSTR)
	{
		ret = shpem_i2c_write(param->send_buf, MAKE_UNSIGNED_SHORT(param->send_buf[8], param->send_buf[9]) + 11);
	}
	else
	{
		ret = shpem_i2c_write(param->send_buf, strlen((char*)shpem_command[param->cmd_id].send_cmd));
	}
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_i2c_write Error %d", ret);
		
		return SHPEM_COM_ERROR;
	}
	
	SHPEM_DEBUG_LOG_2("shpem_i2c_write Success");
	
#ifdef SHPEM_DEBUG_LOG_ENABLE_SENDBUF
	{
		unsigned short count;
		if(param->cmd_id == SHPEM_CMD_DSTR)
		{
			for(count=0;count<MAKE_UNSIGNED_SHORT(param->send_buf[8], param->send_buf[9]) + 11;count++)
			{
				SHPEM_DEBUG_LOG_1("send_buf[%4d] = 0x%02X", count, param->send_buf[count]);
			}
		}
		else
		{
			for(count=0;count<strlen((char*)shpem_command[param->cmd_id].send_cmd);count++)
			{
				SHPEM_DEBUG_LOG_1("send_buf[%4d] = 0x%02X", count, param->send_buf[count]);
			}
		}
	}
#endif
	
	return SHPEM_SUCCESS;
}

static shpem_result_t shpem_com_write_end(shpem_com_param_t* param)
{
	long timeout;
	
	SHPEM_DEBUG_LOG_2();
	
	atomic_set(&shpem_com_flag, 0);
	
	if(SHPEM_IRQ_COMRDY_LOW < 0)
	{
		SHPEM_DEBUG_LOG_1("request_irq Error");
		return SHPEM_COM_ERROR;
	}
	
	timeout = wait_event_timeout(shpem_com_wq, (atomic_read(&shpem_com_flag) != 0), SHPEM_COM_TIMEOUT_WRITE_END);
	SHPEM_IRQ_COMRDY_FREE;
	if(timeout <= 0)
	{
		if(atomic_read(&shpem_com_flag) != 0)
		{
			SHPEM_DEBUG_LOG_2("COMRDY Low");
		}
		else
		{
			SHPEM_DEBUG_LOG_1("write_end timeout %ld", timeout);
			return SHPEM_COM_ERROR;
		}
	}
	
	SHPEM_COMREQ_LOW;
	
	return SHPEM_SUCCESS;
}

static shpem_result_t shpem_com_read(shpem_com_param_t* param)
{
	long timeout;
	int ret;
	shpem_result_t result;
	
	SHPEM_DEBUG_LOG_2();
	
	atomic_set(&shpem_com_flag, 0);
	
	if(SHPEM_IRQ_COMRDY_HIGH < 0)
	{
		SHPEM_DEBUG_LOG_1("request_irq Error");
		return SHPEM_COM_ERROR;
	}
	
	if(param->cmd_id == SHPEM_CMD_SCMS)
	{
		timeout = wait_event_timeout(shpem_com_wq, (atomic_read(&shpem_com_flag) != 0), SHPEM_COM_TIMEOUT_READ_SCMS);
	}
	else
	{
		timeout = wait_event_timeout(shpem_com_wq, (atomic_read(&shpem_com_flag) != 0), SHPEM_COM_TIMEOUT_READ);
	}
	SHPEM_IRQ_COMRDY_FREE;
	if(timeout <= 0)
	{
		if(atomic_read(&shpem_com_flag) != 0)
		{
			SHPEM_DEBUG_LOG_2("COMRDY High");
		}
		else
		{
			SHPEM_DEBUG_LOG_1("read timeout %ld", timeout);
			return SHPEM_COM_ERROR;
		}
	}
	
	SHPEM_COMREQ_HIGH;
	
	udelay(100);
	
	ret = shpem_i2c_read(param->recv_buf, strlen((char*)shpem_command[param->cmd_id].recv_ok));
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_i2c_read Error %d", ret);
		
		return SHPEM_COM_ERROR;
	}
	
	SHPEM_DEBUG_LOG_2("shpem_i2c_read Success");
	
#ifdef SHPEM_DEBUG_LOG_ENABLE_READBUF
	{
		unsigned char count;
		
		for(count=0;count<strlen((char*)shpem_command[param->cmd_id].recv_ok);count++)
		{
			SHPEM_DEBUG_LOG_1("read_buf[%4d] = 0x%02X", count, param->recv_buf[count]);
		}
	}
#endif
	
	result = shpem_recv_check(param);
	
	return result;
}

static shpem_result_t shpem_com_close(shpem_com_param_t* param)
{
	long timeout;
	
	SHPEM_DEBUG_LOG_2();
	
	atomic_set(&shpem_com_flag, 0);
	
	if(SHPEM_IRQ_COMRDY_LOW < 0)
	{
		SHPEM_DEBUG_LOG_1("request_irq Error");
		return SHPEM_COM_ERROR;
	}
	
	timeout = wait_event_timeout(shpem_com_wq, (atomic_read(&shpem_com_flag) != 0), SHPEM_COM_TIMEOUT_CLOSE);
	SHPEM_IRQ_COMRDY_FREE;
	if(timeout <= 0)
	{
		if(atomic_read(&shpem_com_flag) != 0)
		{
			SHPEM_DEBUG_LOG_2("COMRDY Low");
		}
		else
		{
			SHPEM_DEBUG_LOG_1("close timeout %ld", timeout);
			return SHPEM_COM_ERROR;
		}
	}
	
	SHPEM_COMREQ_LOW;
	
	udelay(1000);
	
	return SHPEM_SUCCESS;
}

static shpem_result_t shpem_com_cancel(shpem_com_param_t* param)
{
	SHPEM_DEBUG_LOG_2();
	
	SHPEM_COMREQ_HIGH;
	
	msleep(10);
	
	SHPEM_COMREQ_LOW;
	
	msleep(100);
	
	return SHPEM_SUCCESS;
}

static void shpem_hint_cb_call_kerl(unsigned char* buf)
{
	SHPEM_DEBUG_LOG_2();
	
	if(buf[4] != 0)
	{
		SHPEM_DEBUG_LOG_2("int_type 0x%02X", buf[4]);
		if( (buf[4] & SHPEM_ISR_BREAK_PORT) != 0)
		{
			shpem_micon_restor();
		}
		
		shpem_int_type = buf[4];
		
		atomic_set( &shpem_int_flag, 1 );
		
		wake_up(&shpem_int_wq);
	}
}

static shpem_result_t shpem_com_kerl(shpem_com_param_t* param)
{
	shpem_result_t ret, result = SHPEM_SUCCESS;
	int retry = 0;
	unsigned char dummy[5];
	
	SHPEM_DEBUG_LOG_2();
	
	if(param == NULL)
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	if( (param->cmd_id >= SHPEM_CMD_NUM) || (param->send_buf == NULL) || (param->recv_buf == NULL) )
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	if(shpem_send_check(param) != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	down(&shpem_sem);
	
	SHPEM_DEBUG_LOG_2("cmd_id = %d", param->cmd_id);
	
	if(result == SHPEM_SUCCESS)
	{
		if(shpem_state != SHPEM_NORMAL)
		{
			if( (shpem_state == SHPEM_BREAK_SENSOR) && (shpem_sensor_cmd_check(param->cmd_id) != 0) )
			{
				SHPEM_DEBUG_LOG_1("SHPEM_BREAK_SENSOR_ERROR Sensor Break");
				result = SHPEM_BREAK_SENSOR_ERROR;
			}
			else if(shpem_state == SHPEM_BREAK_MICON)
			{
				SHPEM_DEBUG_LOG_1("SHPEM_BREAK_MICON_ERROR Micon Break");
				result = SHPEM_BREAK_MICON_ERROR;
			}
			else if(shpem_state == SHPEM_FIRM_WRITE)
			{
				SHPEM_DEBUG_LOG_1("SHPEM_EXCLUSIVE_ERROR Firm Update");
				result = SHPEM_EXCLUSIVE_ERROR;
			}
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if(shpem_shutdown_f == 1)
		{
			SHPEM_DEBUG_LOG_2("SHPEM_INIT_ERROR");
			result = SHPEM_INIT_ERROR;
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if( (shpem_init_f == 1) && (param->cmd_id != SHPEM_CMD_INIT) )
		{
			SHPEM_DEBUG_LOG_1("SHPEM_INIT_ERROR state INIT");
			result = SHPEM_INIT_ERROR;
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if( (shpem_sleep_cmd_check(param->cmd_id) != 0) && (shpem_micon_sleep_f == SHPEM_MICON_SLEEP) )
		{
			SHPEM_DEBUG_LOG_1("SHPEM_SLEEP_ERROR");
			result = SHPEM_SLEEP_ERROR;
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if(shpem_reset_f == 1)
		{
			if(param->cmd_id == SHPEM_CMD_RESET)
			{
				param->cmd_id = SHPEM_CMD_RMIR;
			}
			else
			{
				result = SHPEM_INIT_ERROR;
				SHPEM_DEBUG_LOG_1("SHPEM_INIT_ERROR");
			}
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if( (shpem_disp_break_f == 1) && (shpem_disp_cmd_check(param->cmd_id) == 0) )
		{
			//if(shpem_init_send() != SHPEM_SUCCESS)
			{
				SHPEM_DEBUG_LOG_1("SHPEM_BREAK_ERROR Display Break");
				//result = SHPEM_BREAK_ERROR;
			}
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		for(retry=0;retry<SHPEM_COM_RETRY;retry++)
		{
			ret = shpem_com_open(param);
			if(ret != SHPEM_SUCCESS)
			{
				SHPEM_DEBUG_LOG_1("shpem_com_open Error %d", ret);
				shpem_com_cancel(param);
			}
			if(ret == SHPEM_SUCCESS)
			{
				ret = shpem_com_write(param);
				if(ret != SHPEM_SUCCESS)
				{
					SHPEM_DEBUG_LOG_1("shpem_com_write Error %d", ret);
					shpem_com_cancel(param);
				}
			}
			if(ret == SHPEM_SUCCESS)
			{
				ret = shpem_com_write_end(param);
				if(ret != SHPEM_SUCCESS)
				{
					SHPEM_DEBUG_LOG_1("shpem_com_write_end Error %d", ret);
					shpem_com_cancel(param);
				}
			}
			if(ret == SHPEM_SUCCESS)
			{
				ret = shpem_com_read(param);
				if( (ret == SHPEM_SUCCESS) || (ret == SHPEM_NG) )
				{
					result = ret;
					ret = SHPEM_SUCCESS;
				}
				if(ret != SHPEM_SUCCESS)
				{
					SHPEM_DEBUG_LOG_1("shpem_com_read Error %d", ret);
					shpem_com_cancel(param);
				}
			}
			if(ret == SHPEM_SUCCESS)
			{
				ret = shpem_com_close(param);
				if(ret != SHPEM_SUCCESS)
				{
					SHPEM_DEBUG_LOG_1("shpem_com_close Error %d", ret);
					shpem_com_cancel(param);
				}
				else
				{
					if( (result == SHPEM_NG) && (shpem_break_cmd_check(param->cmd_id) == 0) )
					{
						SHPEM_DEBUG_LOG_1("Response NG retry");
					}
					else
					{
						SHPEM_DEBUG_LOG_2("Success");
						if(param->cmd_id == SHPEM_CMD_RMIR)
						{
							shpem_hint_cb_call_kerl(param->recv_buf);
						}
						else if(param->cmd_id == SHPEM_CMD_SHDN)
						{
							shpem_shutdown_f = 1;
						}
						break;
					}
				}
			}
			SHPEM_DEBUG_LOG_1("shpem_com_kerl retry %d", retry);
		}
		if(retry >= 3)
		{
			dummy[4] = SHPEM_ISR_BREAK_PORT;
			SHPEM_DEBUG_LOG_1("shpem_com_kerl retry Over");
			
			shpem_hint_cb_call_kerl(dummy);
			
			if(result != SHPEM_NG)
			{
				result = SHPEM_COM_ERROR;
			}
		}
	}
	
	up(&shpem_sem);
	
	return result;
}

static shpem_result_t shpem_ioctl_com(shpem_com_param_t* argp)
{
	shpem_result_t result = SHPEM_SUCCESS;
	shpem_com_param_t param;
	static unsigned char send_buf[1051] = {0};
	static unsigned char recv_buf[40] = {0};
	
	down(&shpem_ioctl_sem);
	
	if(copy_from_user(&(param.cmd_id), &(argp->cmd_id), sizeof(param.cmd_id)))
	{
		SHPEM_DEBUG_LOG_1("copy_from_user Error");
		result = SHPEM_IO_ERROR;
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if(param.cmd_id >= SHPEM_CMD_NUM)
		{
			SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR Error");
			result = SHPEM_PARAM_ERROR;
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if( (argp->send_buf == NULL) || (argp->recv_buf == NULL) )
		{
			SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR Error");
			result = SHPEM_PARAM_ERROR;
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		if(param.cmd_id == SHPEM_CMD_DSTR)
		{
			if(copy_from_user(send_buf, argp->send_buf, MAKE_UNSIGNED_SHORT(argp->send_buf[8], argp->send_buf[9]) + 11))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				result = SHPEM_IO_ERROR;
			}
		}
		else
		{
			if(copy_from_user(send_buf, argp->send_buf, strlen((char*)shpem_command[argp->cmd_id].send_cmd)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				result = SHPEM_IO_ERROR;
			}
		}
	}
	
	if(result == SHPEM_SUCCESS)
	{
		param.send_buf = send_buf;
		param.recv_buf = recv_buf;
		
		result = shpem_com_kerl(&param);
		if(result == SHPEM_SUCCESS)
		{
			if(copy_to_user(((shpem_com_param_t*)argp)->recv_buf, param.recv_buf, strlen((char*)shpem_command[param.cmd_id].recv_ok)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				result = SHPEM_IO_ERROR;
			}
		}
	}
	
	up(&shpem_ioctl_sem);
	
	return result;
}

static void shpem_firm_write_start(void)
{
	SHPEM_DEBUG_LOG_2();
	
	down(&shpem_sem);
	
	shpem_reset_f = 0;
	
	shpem_state = SHPEM_FIRM_WRITE;
	
	SHPEM_IRQ_INT_DISABLE;
	
	up(&shpem_sem);
}

static void shpem_firm_write_end(void)
{
	shpem_result_t ret;
	
	SHPEM_DEBUG_LOG_2();
	
	down(&shpem_sem);
	
	shpem_init_f = 1;
	
	shpem_state = SHPEM_NORMAL;
	
	SHPEM_IRQ_INT_FREE;
	if(SHPEM_IRQ_INT_HIGH < 0)
	{
		SHPEM_DEBUG_LOG_1("request_irq Error");
	}
	
	up(&shpem_sem);
	
	ret = shpem_init_send();
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_init_send Error %d", ret);
	}
	shpem_init_f = 0;
}

static shpem_result_t shpem_set_sleep_state_kerl( unsigned char dev_id, unsigned char sleep )
{
	shpem_result_t ret;
	shpem_com_param_t param;
	unsigned char write_buf[4] = {0};
	unsigned char read_buf[4] = {0};
	
	SHPEM_DEBUG_LOG_2();
	
	if( (dev_id >= SHPEM_MICON_SLEEP_NUM) || (sleep > SHPEM_MICON_WAKEUP) )
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	down(&shpem_sleep_sem);
	
	if(sleep == SHPEM_MICON_WAKEUP)
	{
		shpem_micon_sleep |= (1 << dev_id);
	}
	else
	{
		shpem_micon_sleep &= ~(1 << dev_id);
	}
	SHPEM_DEBUG_LOG_2("shpem_micon_sleep 0x%02X", shpem_micon_sleep);
	
	if( (shpem_micon_sleep != 0x00) && (shpem_micon_sleep_f == SHPEM_MICON_SLEEP) )
	{
		SHPEM_DEBUG_LOG_2("Send TMES");
		write_buf[0] = 'T';
		write_buf[1] = 'M';
		write_buf[2] = 'E';
		write_buf[3] = 'S';
		param.cmd_id = SHPEM_CMD_TMES;
		param.send_buf = write_buf;
		param.recv_buf = read_buf;
		
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
		}
		else
		{
			shpem_micon_sleep_f = SHPEM_MICON_WAKEUP;
#ifdef SHPEM_SHTERM_ENABLE
			if(shterm_k_set_info(SHTERM_INFO_ACCELE, 1) != SHTERM_SUCCESS)
			{
				SHPEM_DEBUG_LOG_1("shterm_k_set_info(1) Error");
			}
#endif	/* SHPEM_SHTERM_ENABLE */
		}
	}
	else if( (shpem_micon_sleep == 0x00) && (shpem_micon_sleep_f == SHPEM_MICON_WAKEUP) )
	{
		SHPEM_DEBUG_LOG_2("Send SLEP");
		write_buf[0] = 'S';
		write_buf[1] = 'L';
		write_buf[2] = 'E';
		write_buf[3] = 'P';
		param.cmd_id = SHPEM_CMD_SLEP;
		param.send_buf = write_buf;
		param.recv_buf = read_buf;
		
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
		}
		else
		{
			shpem_micon_sleep_f = SHPEM_MICON_SLEEP;
#ifdef SHPEM_SHTERM_ENABLE
			if(shterm_k_set_info(SHTERM_INFO_ACCELE, 0) != SHTERM_SUCCESS)
			{
				SHPEM_DEBUG_LOG_1("shterm_k_set_info(0) Error");
			}
#endif	/* SHPEM_SHTERM_ENABLE */
		}
	}
	else
	{
		SHPEM_DEBUG_LOG_2("Sleep State Not Change");
		
		ret = SHPEM_SUCCESS;
	}
	
	up(&shpem_sleep_sem);
	
	return ret;
}

static shpem_result_t shpem_set_sensor_state_kerl( unsigned char dev_id, unsigned char sensor)
{
	shpem_result_t ret;
	shpem_com_param_t param;
	unsigned char write_buf[4] = {0};
	unsigned char read_buf[4] = {0};
	
	SHPEM_DEBUG_LOG_2();
	
	if( (dev_id >= SHPEM_MICON_SENSOR_NUM) || (sensor > SHPEM_MICON_SENSOR_OFF) )
	{
		SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
		return SHPEM_PARAM_ERROR;
	}
	
	down(&shpem_sensor_sem);
	
	if(sensor == SHPEM_MICON_SENSOR_ON)
	{
		shpem_micon_sensor |= (1 << dev_id);
	}
	else
	{
		shpem_micon_sensor &= ~(1 << dev_id);
	}
	SHPEM_DEBUG_LOG_2("shpem_micon_sensor 0x%02X", shpem_micon_sensor);
	
	if( (shpem_micon_sensor != 0x00) && (shpem_micon_sensor_f == SHPEM_MICON_SENSOR_OFF) )
	{
		SHPEM_DEBUG_LOG_2("Send SENS");
		write_buf[0] = 'S';
		write_buf[1] = 'E';
		write_buf[2] = 'N';
		write_buf[3] = 'S';
		param.cmd_id = SHPEM_CMD_SENS;
		param.send_buf = write_buf;
		param.recv_buf = read_buf;
		
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
		}
		else
		{
			shpem_micon_sensor_f = SHPEM_MICON_SENSOR_ON;
		}
	}
	else if( (shpem_micon_sensor == 0x00) && (shpem_micon_sensor_f == SHPEM_MICON_SENSOR_ON) )
	{
		SHPEM_DEBUG_LOG_2("Send SENE");
		write_buf[0] = 'S';
		write_buf[1] = 'E';
		write_buf[2] = 'N';
		write_buf[3] = 'E';
		param.cmd_id = SHPEM_CMD_SENE;
		param.send_buf = write_buf;
		param.recv_buf = read_buf;
		
		ret = shpem_com_kerl(&param);
		if(ret != SHPEM_SUCCESS)
		{
			SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
		}
		else
		{
			shpem_micon_sensor_f = SHPEM_MICON_SENSOR_OFF;
		}
	}
	else
	{
		SHPEM_DEBUG_LOG_2("Sensor State Not Change");
		
		ret = SHPEM_SUCCESS;
	}
	
	up(&shpem_sensor_sem);
	
	return ret;
}

static shpem_result_t shpem_gpio_ctl(unsigned char* gpio)
{
	switch(gpio[0])
	{
		case SHPEM_GPIO_NUM_FW_CTL:
		case SHPEM_GPIO_NUM_COMREQ:
		case SHPEM_GPIO_NUM_RESET:
			break;
		default:
			return SHPEM_PARAM_ERROR;
	}
	switch(gpio[1])
	{
		case GPIO_LOW:
		case GPIO_HIGH:
			break;
		default:
			return SHPEM_PARAM_ERROR;
	}
	
	gpio_direction_output(gpio[0], gpio[1]);
	
	return SHPEM_SUCCESS;
}

void shpem_diag_test_start( void )
{
	SHPEM_DEBUG_LOG_2();
}

void shpem_diag_test_end( void )
{
	SHPEM_DEBUG_LOG_2();
}

static int shpem_config_gpio(void)
{
	int ret;
	
	SHPEM_DEBUG_LOG_2();
	
	ret = gpio_request(SHPEM_GPIO_REQ_FW_CTL);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_request Error");
		return ret;
	}
	
	ret = gpio_request(SHPEM_GPIO_REQ_COMREQ);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_request Error");
		return ret;
	}
	
	ret = gpio_request(SHPEM_GPIO_REQ_COMRDY);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_request Error");
		return ret;
	}
	
	ret = gpio_request(SHPEM_GPIO_REQ_INT);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_request Error");
		return ret;
	}
	
	ret = gpio_request(SHPEM_GPIO_REQ_RESET);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_request Error");
		return ret;
	}
	
	ret = gpio_tlmm_config(SHPEM_GPIO_CNF_FW_CTL);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_tlmm_config Error");
		return ret;
	}
	ret = gpio_tlmm_config(SHPEM_GPIO_CNF_COMREQ);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_tlmm_config Error");
		return ret;
	}
	ret = gpio_tlmm_config(SHPEM_GPIO_CNF_COMRDY);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_tlmm_config Error");
		return ret;
	}
	ret = gpio_tlmm_config(SHPEM_GPIO_CNF_INT);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_tlmm_config Error");
		return ret;
	}
	ret = gpio_tlmm_config(SHPEM_GPIO_CNF_RESET);
	if(ret != 0)
	{
		SHPEM_DEBUG_LOG_1("gpio_tlmm_config Error");
		return ret;
	}
	
	return 0;
}

static int shpem_release_gpio(void)
{
	SHPEM_DEBUG_LOG_2();
	
	gpio_free(SHPEM_GPIO_NUM_FW_CTL);
	gpio_free(SHPEM_GPIO_NUM_COMREQ);
	gpio_free(SHPEM_GPIO_NUM_COMRDY);
	gpio_free(SHPEM_GPIO_NUM_INT);
	gpio_free(SHPEM_GPIO_NUM_RESET);
	
	return 0;
}

static int shpem_initialize(void)
{
	shpem_result_t ret;
	
	SHPEM_DEBUG_LOG_2();
	
	init_MUTEX(&shpem_sem);
	init_MUTEX(&shpem_ioctl_sem);
	init_MUTEX(&shpem_sleep_sem);
	init_MUTEX(&shpem_sensor_sem);
	atomic_set( &shpem_com_flag, 0 );
	atomic_set( &shpem_int_flag, 0 );
	
	shpem_micon_reset();
	
	ret = shpem_init_send();
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_init_send Error %d", ret);
	}
	shpem_init_f = 0;
	
	if(SHPEM_IRQ_INT_HIGH < 0)
	{
		SHPEM_DEBUG_LOG_1("request_irq Error");
		return -1;
	}
	
	return 0;
}

int shpem_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	int rc;
	
	SHPEM_DEBUG_LOG_2();
	
	rc = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if(rc == 0)
	{
		SHPEM_DEBUG_LOG_1("i2c_check_functionality Error %d", rc);
		goto exit_check_functionality_failed;
	}
	shpem_i2c_client = client;
	
	shpem_wq = create_singlethread_workqueue(SHPEM_WORKQUEUE_NAME);
	if(!shpem_wq)
	{
		SHPEM_DEBUG_LOG_1("create_singlethread_workqueue Error");
		goto exit_create_singlethread_failed;
	}
	INIT_WORK(&shpem_work, shpem_wq_handler);
	
	rc = shpem_config_gpio();
	if(rc != 0)
	{
		SHPEM_DEBUG_LOG_1("shpem_config_gpio Error %d", rc);
		goto exit_gpio_config_failed;
	}
	
	rc = shpem_initialize();
	if(rc != 0)
	{
		SHPEM_DEBUG_LOG_1("shpem_initialize Error %d", rc);
		goto exit_initialize_failed;
	}
	
	return 0;

exit_initialize_failed:
exit_gpio_config_failed:
	shpem_release_gpio();
exit_create_singlethread_failed:
exit_check_functionality_failed:
	return -ENODEV;
}

static int shpem_remove(struct i2c_client *client)
{
	SHPEM_DEBUG_LOG_2();
	
	return 0;
}

static void shpem_shutdown(struct i2c_client *client)
{
	shpem_result_t ret;
	
	SHPEM_DEBUG_LOG_2();
	
	ret = shpem_shdn_send();
	if(ret != SHPEM_SUCCESS)
	{
		SHPEM_DEBUG_LOG_1("shpem_com_kerl Error %d", ret);
	}
}

static int shpem_open(struct inode *inode, struct file *filp)
{
	SHPEM_DEBUG_LOG_2();
	return 0;
}

static int shpem_release( struct inode *inode, struct file *filp )
{
	SHPEM_DEBUG_LOG_2();
	return 0;
}
static int shpem_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char id[2];
	unsigned char gpio[2];
	unsigned char dummy = 0;
	int ret, len, wait_event_ret = 0;
	static unsigned char send_buf[SHPEM_MAX_SEND_BUF] = {0};
	static unsigned char recv_buf[SHPEM_MAX_RECV_BUF] = {0};
	
	SHPEM_DEBUG_LOG_2("cmd = %d", cmd);
	
	if( (cmd != SHPEM_IOCTL_FW_WRITE_START) && 
		(cmd != SHPEM_IOCTL_FW_WRITE_END)   &&
		(cmd != SHPEM_IOCTL_RESET) )
	{
		if(argp == NULL)
		{
			SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR");
			return SHPEM_PARAM_ERROR;
		}
	}
	
	switch (cmd)
	{
		case SHPEM_IOCTL_SET_SLEEP_STATE:
		case SHPEM_IOCTL_SET_SENSOR_STATE:
			if(copy_from_user(&id, argp, sizeof(id)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				return SHPEM_IO_ERROR;
			}
			break;
		case SHPEM_IOCTL_GPIO_CTL:
			if(copy_from_user(&gpio, argp, sizeof(gpio)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				return SHPEM_IO_ERROR;
			}
			break;
		case SHPEM_IOCTL_I2C_WRITE:
			if(copy_from_user(&len, &(((shpem_i2c_io_t*)argp)->len), sizeof(len)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				return SHPEM_IO_ERROR;
			}
			if(len > SHPEM_MAX_SEND_BUF)
			{
				SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR len = %d", len);
				return SHPEM_PARAM_ERROR;
			}
			if(copy_from_user(send_buf, ((shpem_i2c_io_t*)argp)->buf, len))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				return SHPEM_IO_ERROR;
			}
			break;
		case SHPEM_IOCTL_I2C_READ:
			if(copy_from_user(&len, &(((shpem_i2c_io_t*)argp)->len), sizeof(len)))
			{
				SHPEM_DEBUG_LOG_1("copy_from_user Error");
				return SHPEM_IO_ERROR;
			}
			if(len > SHPEM_MAX_RECV_BUF)
			{
				SHPEM_DEBUG_LOG_1("SHPEM_PARAM_ERROR len = %d", len);
				return SHPEM_PARAM_ERROR;
			}
			break;
		default:
			break;
	}
	
	switch (cmd)
	{
		case SHPEM_IOCTL_COM:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_COM");
			ret = shpem_ioctl_com((shpem_com_param_t*)argp);
			break;
		case SHPEM_IOCTL_SET_SLEEP_STATE:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_SET_SLEEP_STATE");
			ret = shpem_set_sleep_state_kerl(id[0], id[1]);
			break;
		case SHPEM_IOCTL_SET_SENSOR_STATE:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_SET_SENSOR_STATE");
			ret = shpem_set_sensor_state_kerl(id[0], id[1]);
			break;
		case SHPEM_IOCTL_HINT:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_HINT");
			wait_event_ret = wait_event_interruptible(shpem_int_wq, (atomic_read(&shpem_int_flag) != 0));
			atomic_set( &shpem_int_flag, 0 );
			if(wait_event_ret == 0)
			{
				if(copy_to_user(argp, &shpem_int_type, sizeof(shpem_int_type)))
				{
					SHPEM_DEBUG_LOG_1("copy_from_user Error");
					return SHPEM_IO_ERROR;
				}
			}
			else
			{
				if(copy_to_user(argp, &dummy, sizeof(dummy)))
				{
					SHPEM_DEBUG_LOG_1("copy_from_user Error");
					return SHPEM_IO_ERROR;
				}
			}
			ret = SHPEM_SUCCESS;
			break;
		case SHPEM_IOCTL_GPIO_CTL:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_GPIO_CTL");
			ret = shpem_gpio_ctl(gpio);
			break;
		case SHPEM_IOCTL_I2C_WRITE:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_I2C_WRITE");
			ret = shpem_i2c_write(send_buf, len);
			break;
		case SHPEM_IOCTL_I2C_READ:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_I2C_READ");
			ret = shpem_i2c_read(recv_buf, len);
			break;
		case SHPEM_IOCTL_FW_WRITE_START:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_FW_WRITE_START");
			shpem_firm_write_start();
			ret = SHPEM_SUCCESS;
			break;
		case SHPEM_IOCTL_FW_WRITE_END:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_FW_WRITE_END");
			shpem_firm_write_end();
			ret = SHPEM_SUCCESS;
			break;
		case SHPEM_IOCTL_RESET:
			SHPEM_DEBUG_LOG_2("SHPEM_IOCTL_RESET");
			shpem_micon_reset();
			ret = SHPEM_SUCCESS;
			break;
		default:
			SHPEM_DEBUG_LOG_1("cmd Error 0x%X", cmd);
			ret = SHPEM_PARAM_ERROR;
			break;
	}
	
	if(ret == SHPEM_SUCCESS)
	{
		switch (cmd)
		{
			case SHPEM_IOCTL_I2C_READ:
				if(copy_to_user(((shpem_i2c_io_t*)argp)->buf, recv_buf, len))
				{
					SHPEM_DEBUG_LOG_1("copy_from_user Error");
					return SHPEM_IO_ERROR;
				}
				break;
			default:
				break;
		}
	}
	
	return ret;
}

static const struct i2c_device_id shpem_i2c_id[] = {
	{ SHPEM_I2C_DEVNAME, 0 },
	{ }
};

static struct i2c_driver shpem_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = shpem_probe,
	.remove = shpem_remove,
	.shutdown = shpem_shutdown,
	.id_table = shpem_i2c_id,
	.driver ={
		   .owner = THIS_MODULE,
		   .name = SHPEM_I2C_DEVNAME,
		   },
};

static struct file_operations shpem_fops =
{
	.owner      = THIS_MODULE,
	.open       = shpem_open,
	.release    = shpem_release,
	.ioctl      = shpem_ioctl,
};

static struct miscdevice shpem_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = SHPEM_MISC_DEVNAME,
	.fops = &shpem_fops,
};

static int __init shpem_init(void)
{
	int ret;
	
	SHPEM_DEBUG_LOG_2();
	
	ret = i2c_add_driver(&shpem_driver);
	if(ret)
	{
		SHPEM_DEBUG_LOG_1("i2c_add_driver Error %d", ret);
		return ret;
	}
	
	ret = misc_register(&shpem_dev);
	if(ret)
	{
		SHPEM_DEBUG_LOG_1("misc_register Error %d", ret);
		return ret;
	}
	
	return ret;
}

static void __exit shpem_exit(void)
{
	SHPEM_DEBUG_LOG_2();
	
	i2c_del_driver(&shpem_driver);
}

module_init(shpem_init);
module_exit(shpem_exit);

MODULE_DESCRIPTION("SHARP PERIPHERAL CONTROL MICROPROCESSOR DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
