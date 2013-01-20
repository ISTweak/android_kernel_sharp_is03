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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/proximity.h>
#include <linux/hrtimer.h>

#define DEBUG 0

#define Y2659_DEBUG_MSG		0
#define Y2659_DEBUG_FUNC	0


#if Y2659_DEBUG_FUNC
#define FUNC_LOG() printk(KERN_DEBUG "[PROXIMITY] %s is called\n", __func__)
#else
#define FUNC_LOG()
#endif

#if Y2659_DEBUG_MSG
#define DEBUG_LOG(format, ...)	printk(KERN_DEBUG "[PROXIMITY] " format "\n", ## __VA_ARGS__)
#else
#define DEBUG_LOG(format, ...)
#endif


/*+-------------------------------------------------------------------------+*/
/*|																			|*/
/*+-------------------------------------------------------------------------+*/
typedef struct y2659_data_tag   y2659_data;
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;
typedef struct work_struct      WorkStruct;
typedef struct input_dev        InputDev;
typedef struct device           Device;

#define	I2C_RETRY			3

struct y2659_data_tag
{
	struct input_dev	*input_dev;
	int			mnEnablePin;
	int			(*mpfPinSetupFunc)(void);
	void		(*mpfPinShutdownFunc)(void);
	WorkStruct	IrqWork;
};

typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;


static I2cClt		*this_client;
static char			MVO = 0;
static atomic_t		open_flag = ATOMIC_INIT(0);
static atomic_t		sensor_data = ATOMIC_INIT(7);	/* Init = Far */
static atomic_t		enable_mode = ATOMIC_INIT(0);	/* 0=Disable,1=Enable */
static const short CYCLE_DATA[8][2] =	{	{0x04,    8},	/*    8ms */
											{0x0C,   16},	/*   16ms */
											{0x14,   32},	/*   32ms */
											{0x1C,   64},	/*   64ms */
											{0x24,  128},	/*  128ms */
											{0x2C,  256},	/*  256ms */
											{0x34,  512},	/*  512ms */
											{0x3C, 1024},	/* 1024ms */
										};

static int __devinit Y2659_Probe(I2cClt *client, const I2cDevID *poDevId);


/*+-------------------------------------------------------------------------+*/
/*|	I2C Read																|*/
/*+-------------------------------------------------------------------------+*/
static int Y2659_I2cRead(I2cClt *client, char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}
	
	if (loop_i >= I2C_RETRY) {
		printk(KERN_ERR "I2cRead: error\n");

		return -EIO;
	}

	return 0;
}

/*+-------------------------------------------------------------------------+*/
/*|	I2C Write																|*/
/*+-------------------------------------------------------------------------+*/
static int Y2659_I2cWrite(I2cClt *client, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= client->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	FUNC_LOG();

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(client->adapter, oMsgs, 1);
		if(nResult == 1)
		{
            DEBUG_LOG("I2cWrite success(%02X,reg:%02X,Data:%02X)=%d", client->addr, bRegAdr, bData, nResult);
			return 0;
		}
	}

	printk(KERN_ERR "I2cWrite: error\n");
	return -1;
}


static int IOECS_Enable(void)
{
	y2659_data *data = NULL;
	uint8_t bData;
	int nResult = 0;

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
		data = i2c_get_clientdata(this_client);

		/* LED Output Enable = ON */
	    gpio_direction_output(data->mnEnablePin, 1);

		/*  Proximity Report Data Initialize */
		data->input_dev->abs[ABS_DISTANCE] = 99;

		/* Master Detection value Initialize */
		MVO = 0;

		/* Init = Far */
		atomic_set(&sensor_data, 7);

		/* OPMOD = H'03 */
		bData = 0x03;
		nResult = Y2659_I2cWrite(this_client, Y2659_REG_OPMOD, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}

		enable_irq(this_client->irq);
	}

	atomic_set(&enable_mode, 1);
	return 0;
}


static int IOECS_Disable(void)
{
	y2659_data *data = NULL;
	uint8_t bData;
	int nResult = 0;

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 1)
	{
		data = i2c_get_clientdata(this_client);

		disable_irq(this_client->irq);

		/* LED Output Enable = Off */
	    gpio_direction_output(data->mnEnablePin, 0);

		/* OPMOD = H'02 */
		bData = 0x02;
		nResult = Y2659_I2cWrite(this_client, Y2659_REG_OPMOD, bData);
		if(nResult < 0)
		{
			DEBUG_LOG("I2cWrite-->Error");
			return -EIO;
		}
	}

	atomic_set(&enable_mode, 0);
	return 0;
}


static int IOECS_SetCycle(short cycle_data)
{
	uint8_t bData;
	int nResult = 0;
	uint8_t loop;
	short cmp_data;

	FUNC_LOG();

	for(loop=0; loop<7; loop++)
	{
		cmp_data = CYCLE_DATA[loop][1] + ((CYCLE_DATA[loop+1][1] - CYCLE_DATA[loop][1]) / 2);

		DEBUG_LOG("cycle_data : %d,cmp_data : %d",cycle_data,cmp_data);

		if(cycle_data <= cmp_data)
		{
			break;
		}
	}

	DEBUG_LOG("CYCLE_DATA : %d",CYCLE_DATA[loop][1]);

	/* CYCLE Setting */
	bData = CYCLE_DATA[loop][0];
	nResult = Y2659_I2cWrite(this_client, Y2659_REG_CYCLE, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	return 0;
}


static int IOECS_GetVO_DATA(uint8_t *cycle_data)
{
	uint8_t buffer[2] = {0};

	FUNC_LOG();

	Y2659_I2cRead(this_client, buffer,2);

	*cycle_data = (buffer[1] & 0x01);

	return 0;
}


static int Y2659_open(struct inode *inode, struct file *filp)
{
	int ret = -1;

	FUNC_LOG();

	if (atomic_cmpxchg(&open_flag, 0, 1) == 0)
	{
		/* Init = Far */
		atomic_set(&sensor_data, 7);
		ret = 0;
	}

	return ret;
}


static int Y2659_release(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	IOECS_Disable();

	atomic_set(&open_flag, 0);

	return 0;
}

static int Y2659_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *ppos)

{
	char tmp = (char)atomic_read(&sensor_data);

	if (copy_to_user(buf, &tmp, sizeof(tmp))) {
		return -EFAULT;
	}

	return 0;
}

static int Y2659_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t bData;
	short cycle;

	FUNC_LOG();

	switch (cmd) {
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");

			if (copy_from_user(&cycle, argp, sizeof(cycle))) {
				DEBUG_LOG("ECS_IOCTL_SET_CYCLE ERR");
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_ENABLE:
			DEBUG_LOG("ECS_IOCTL_ENABLE");
			if(IOECS_Enable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_DISABLE:
			DEBUG_LOG("ECS_IOCTL_DISABLE");
			if(IOECS_Disable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");
			if(IOECS_SetCycle(cycle) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_GET_VO_DATA:
			DEBUG_LOG("ECS_IOCTL_GET_VO_DATA");
			if(IOECS_GetVO_DATA(&bData) < 0)
			{
				return -EIO;
			}
			break;
		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_GET_VO_DATA:
			if (copy_to_user(argp, &bData, sizeof(bData))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	return 0;
}


static struct file_operations Y2659_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= Y2659_open,
	.release	= Y2659_release,
	.ioctl		= Y2659_ioctl,
	.read		= Y2659_read,
};

static struct miscdevice Y2659_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity_dev",
	.fops = &Y2659_ctl_fops,
};


static irqreturn_t Y2659_interrupt(int irq, void *dev_id)
{
	y2659_data *poProximityRec = dev_id;

	FUNC_LOG();

	disable_irq_nosync(this_client->irq);
	schedule_work(&poProximityRec->IrqWork);

	if(MVO == 1)
	{
		MVO = 0;
	}
	else
	{
		MVO = 1;
	}

	return IRQ_HANDLED;
}

static void Y2659_Irq_workfunc(WorkStruct *work)
{
	uint8_t buffer[5] = {0};
	uint8_t bData;
	y2659_data *data = i2c_get_clientdata(this_client);

	FUNC_LOG();

	if(MVO == 1)
	{
		/* HYS = H'00 */
		bData = 0x00;
	}
	else
	{
		/* HYS = H'20 */
		bData = 0x20;
	}
	Y2659_I2cWrite(this_client, Y2659_REG_HYS, bData);

	/* VOUT = High(CON = H'18) */
	bData = 0x18;
	Y2659_I2cWrite(this_client, Y2659_REG_CON, bData);

	Y2659_I2cRead(this_client, buffer,2);
	DEBUG_LOG("%02X:%02X",buffer[0],buffer[1]);

	/* Err Check */
	if(MVO != (buffer[1] & 0x01))
	{
		DEBUG_LOG("Dephasing");

		/* OPMOD = H'02 */
		bData = 0x02;
		Y2659_I2cWrite(this_client, Y2659_REG_OPMOD, bData);

		/* HYS = H'20 */
		bData = 0x20;
		Y2659_I2cWrite(this_client, Y2659_REG_HYS, bData);

		/* OPMOD = H'03 */
		bData = 0x03;
		Y2659_I2cWrite(this_client, Y2659_REG_OPMOD, bData);

		/* MVO mode reset */
		MVO = 0;
	}
	else
	{
		/* Change(near = 0,far = 7) */
		if((buffer[1] & 0x01) == 0)
		{
			atomic_set(&sensor_data, 7);
		}
		else
		{
			atomic_set(&sensor_data, 0);
		}
		input_report_abs(data->input_dev, ABS_DISTANCE, atomic_read(&sensor_data));
		input_sync(data->input_dev);
	}

	enable_irq(this_client->irq);

	/* VOUT = Low(CON = H'00) */
	bData = 0x00;
	Y2659_I2cWrite(this_client, Y2659_REG_CON, bData);
}

static int Y2659_ReleaseGPIO(y2659_data *poProximityRec)
{
	FUNC_LOG();

	if(poProximityRec == NULL)
		return -EINVAL;

	/* GPIO release */
	poProximityRec->mpfPinShutdownFunc();
	return 0;
}

static int Y2659_ConfigGPIO(y2659_data *poProximityRec)
{
	FUNC_LOG();

	if(poProximityRec == NULL)
		return -EINVAL;
	return poProximityRec->mpfPinSetupFunc();
}

static int Y2659_Initialize(y2659_data *poProximityRec, I2cClt *client)
{
	int nResult = 0;
	uint8_t bData;
	uint8_t buffer[2] = {0};

	FUNC_LOG();

	/* IRQ clear(DummyRead) */
	nResult = Y2659_I2cRead(client, buffer,2);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cRead-->Error");
		return -EIO;
	}

	/* LED Output Enable = Off */
    gpio_direction_output(poProximityRec->mnEnablePin, 0);

	/* GAIN = H'08 */
	bData = 0x08;
	nResult = Y2659_I2cWrite(client, Y2659_REG_GAIN, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* HYS = H'20 */
	bData = 0x20;
	nResult = Y2659_I2cWrite(client, Y2659_REG_HYS, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* CYCLE = H'04 */
	bData = 0x04;
	nResult = Y2659_I2cWrite(client, Y2659_REG_CYCLE, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	/* OPMOD = H'02 */
	bData = 0x02;
	nResult = Y2659_I2cWrite(client, Y2659_REG_OPMOD, bData);
	if(nResult < 0)
	{
		DEBUG_LOG("I2cWrite-->Error");
		return -EIO;
	}

	return 0;
}

static int __devexit Y2659_Remove(I2cClt *client)
{
	y2659_data *poProximityRec = i2c_get_clientdata(client);

	FUNC_LOG();

	DEBUG_LOG("Remove()");

	dev_info(&client->dev, "removing driver\n");
	device_init_wakeup(&client->dev, 0);
	Y2659_ReleaseGPIO(poProximityRec);
	kfree(poProximityRec);
	return 0;
}

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_PROXIMITY_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);


static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_PROXIMITY_I2C_DEVNAME,
	},
	.probe	  = Y2659_Probe,
	.remove	  = __devexit_p(Y2659_Remove),
	.id_table = gI2cDevIdTableAcc,
};

static int __devinit Y2659_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	struct sh_i2c_proximity_platform_data *poSetupData;
	y2659_data *poProximityRec = NULL;
	int nResult;

	FUNC_LOG();

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	poProximityRec = kzalloc(sizeof(y2659_data), GFP_KERNEL);
	if (!poProximityRec) {
		DEBUG_LOG("memory allocation failed.");
		nResult = -ENOMEM;
		goto exit1;
	}

	INIT_WORK(&poProximityRec->IrqWork, Y2659_Irq_workfunc);
	i2c_set_clientdata(client, poProximityRec);

	/* Check platform data */
	if (client->dev.platform_data == NULL) {
		DEBUG_LOG("platform data is NULL.");
		nResult = -ENOMEM;
		goto exit2;
	}

	/* Copy to global variable */
	poSetupData = client->dev.platform_data;
	this_client = client;
	poProximityRec->mnEnablePin		 = poSetupData->gpio_enable;
	poProximityRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poProximityRec->mpfPinShutdownFunc = poSetupData->gpio_shutdown;

	nResult = Y2659_ConfigGPIO(poProximityRec);
	if(nResult < 0)
	{
		DEBUG_LOG("ConfigGPIO is Err.");
		goto exit2;
	}

	/* Y2659 Initialize */
	nResult = Y2659_Initialize(poProximityRec,this_client);
	if (nResult < 0) {
		DEBUG_LOG("initialize failed.");
		goto exit3;
	}

	nResult = misc_register(&Y2659_device);
	if (nResult)
	{
		DEBUG_LOG("misc_register failed.");
		goto exit4;
	}

	/* IRQ */
	nResult = request_irq(client->irq, Y2659_interrupt, IRQF_TRIGGER_LOW | IRQF_DISABLED,
					  "Y2659_VOUT", poProximityRec);
	if (nResult < 0) {
		DEBUG_LOG("request irq failed.");
		goto exit5;
	}

	disable_irq(this_client->irq);


	/* Declare input device */
	poProximityRec->input_dev = input_allocate_device();
	if (!poProximityRec->input_dev) {
		nResult = -ENOMEM;
		DEBUG_LOG("Failed to allocate input device.");
		goto exit5;
	}
	/* Setup input device */
	set_bit(EV_ABS, poProximityRec->input_dev->evbit);

	/* proximity value near=7, far=0 */
	input_set_abs_params(poProximityRec->input_dev, ABS_DISTANCE, 0, 7, 0, 0);

	/* Set name */
	poProximityRec->input_dev->name = "proximity";
	
	/* Register */
	nResult = input_register_device(poProximityRec->input_dev);
	if (nResult) {
		DEBUG_LOG("Unable to register input device.");
		goto exit6;
	}

	return 0;

exit6:
	input_free_device(poProximityRec->input_dev);
exit5:
	misc_deregister(&Y2659_device);
exit4:
exit3:
	Y2659_ReleaseGPIO(poProximityRec);
exit2:
	kfree(poProximityRec);
exit1:
exit0:
	return nResult;
}

static int __init Y2659_Init(void)
{
	FUNC_LOG();

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);
}

static void __exit Y2659_Exit(void)
{
	FUNC_LOG();

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}

module_init(Y2659_Init);
module_exit(Y2659_Exit);


MODULE_DESCRIPTION("proximity sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
