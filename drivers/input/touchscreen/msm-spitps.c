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

#define	TPS_SETPARAM				/* Firm parameter setting */
#define	TPS_PRNERR				/* Log output(Error Log) */
/* #define	TPS_PRNLOG */				/* Log output(Normal Log) */
/* #define	TPS_PRNDEB */				/* Log output(Debug Log) */

/*+-------------------------------------------------------------------------+*/
/*|	Include files															|*/
/*+-------------------------------------------------------------------------+*/
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <mach/gpio.h>
#include <mach/msm_spitps.h>
//#include <sharp/shlcdc_kerl.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>

#include <linux/spi/spi.h>
#include <mach/sharp_smem.h>	/* HW Rev */

/*+-------------------------------------------------------------------------+*/
/*|	Constant declaration													|*/
/*+-------------------------------------------------------------------------+*/

#define	KPD_KEYPRESS			1
#define	KPD_KEYRELEASE			0

/* Command */
enum
{
	QPHYSLEN			= 128,
	QCVENDOR_ID			= 0x5143,
	QCPRODUCT_ID		= 2,
	QCVERSION_ID		= 1
};

#define	INITDELAY_TIMER			600
#define	RECOVER_TIMER			1000

#define	RETRY_MAX			3
#define	RETRY_WAIT			15

typedef enum
{
	TPS_STATE_HOVER = 0,
	TPS_STATE_DOWN,
	TPS_STATE_MAX,
} TpsState;

/* Adjustment Parameters */
#define	POS_X0				0
#define	POS_X1				160
#define	POS_X2				480
#define	POS_X3				640
#define	POS_Y0				0
#define	POS_Y1				235
#define	POS_Y2				547
#define	POS_Y3				860
#define	POS_Y4				1095
#define	POS_LIMIT			100

#define	ADJUST_POINT		6					/* Number of Adjustment points */
#define	AREA_COUNT			(ADJUST_POINT*2)	/* Number of Adjustment Area */
#define DOUBLE_ACCURACY		10000

#define	TPS_DISABLE_FLIP	0x01
#define	TPS_DISABLE_SLEEP	0x02
#define	TPS_DISABLE_API		0x04

#define	TPS_DISABLE_ON		0xFF
#define	TPS_DISABLE_OFF		0x00

#define	TPS_CHECK_ON		0x01
#define	TPS_CHECK_OFF		0x00

#define	TPS_RETURN_ON		0x01
#define	TPS_RETURN_OFF		0x00

#define	TPS_RESET_INTERVAL	300

#define	TPS_CALIB_ON		0x01
#define	TPS_CALIB_OFF		0x00
#define	TPS_CALIB_WAIT		1400
#define	TPS_CALIB_MIN		70
#define	TPS_CALIB_MAX		150
#define	TPS_CALIB_TIMEOUT	500

/*+-------------------------------------------------------------------------+*/
/*|	Type declaration														|*/
/*+-------------------------------------------------------------------------+*/
typedef struct spitps_record	SpiTpsRec;
typedef struct spi_device       SpiDev;

typedef struct work_struct		WorkStruct;
typedef struct input_dev		InputDev;
typedef struct device			Device;

struct spitps_record
{
	SpiDev  *mpoSpidev;
	InputDev *mpoInDev;
	int		mnProductInfo;
	char	mcPhysInfo[QPHYSLEN];
	int		mnIrqPin;
	int		(*mpfPinSetupFunc)(void);
	void	(*mpfPinShutdownFunc)(void);
	uint8_t	mbIsActive;					/* Driver Status(1:Active/0:Inactive) */
	struct delayed_work moCmdQ;
	WorkStruct moIrqWork;
	int		mnReset;
	int		mnStandby;
	int		mnHsspClkPin;
	int		mnHsspDataPin;
	TpsState mnState;
	uint8_t	mbIsEnable;					/* Status(1:Enable/0:Disable) */
	uint8_t	mbIsTestMode;				/* Test mode Status(1:Test mode/0:Normal mode) */
	uint8_t	mbAdjustEnable;				/* Adjustment(1:Validity 0:Invalid) */
	uint8_t	mbAccessState;				/* Access state */
	uint8_t	mbIsFirst;					/* flag in First start */
	uint8_t	mbIsUpdate;
};

typedef struct
{
	short x;
	short y;
} TpsPoint_t;

typedef struct
{
	TpsPoint_t p;		/* Upper left */
	TpsPoint_t q;		/* Upper right */
	TpsPoint_t r;		/* Lower left */
	TpsPoint_t s;		/* Lower right */
} TpsArea_t;

typedef struct
{
	short mValue;
	short mNo;
} Qsort_t;

typedef struct
{
	uint8_t mbID;
	uint16_t mwPosX;
	uint16_t mwPosY;
	uint16_t mwPosZ;
} TpsEvent;

/* Defined type dispatch table */
typedef struct
{
	uint8_t mbValid;							/* 0:Invalid */
												/* Event Handling Functions */
	void (*mpReportInit)(void);
	void (*mpReportPos)(InputDev *, int nCnt, TpsEvent *poEv);
} TpsDispatch;

static struct semaphore sem;
static uint8_t gSense[171];

/*+-------------------------------------------------------------------------+*/
/*|	Defining Global Variables												|*/
/*+-------------------------------------------------------------------------+*/
#ifdef TPS_SETPARAM
static uint8_t gbSetParam = 0x03;
#endif	/* TPS_SETPARAM */

/* Coordinates of six criteria points for adjusting */
static const TpsPoint_t gBasePt[ADJUST_POINT] =
		{
			{POS_X1, POS_Y1}, {POS_X2, POS_Y1},
			{POS_X1, POS_Y2}, {POS_X2, POS_Y2},
			{POS_X1, POS_Y3}, {POS_X2, POS_Y3},
		};
/* Coordinates of the split area for adjusting */
static const TpsArea_t gAreaRect[AREA_COUNT] =
		{
			{{POS_X0, POS_Y0}, {POS_X1, POS_Y0}, {POS_X0, POS_Y1}, {POS_X1, POS_Y1}},
			{{POS_X1, POS_Y0}, {POS_X2, POS_Y0}, {POS_X1, POS_Y1}, {POS_X2, POS_Y1}},
			{{POS_X2, POS_Y0}, {POS_X3, POS_Y0}, {POS_X2, POS_Y1}, {POS_X3, POS_Y1}},
			{{POS_X0, POS_Y1}, {POS_X1, POS_Y1}, {POS_X0, POS_Y2}, {POS_X1, POS_Y2}},
			{{POS_X1, POS_Y1}, {POS_X2, POS_Y1}, {POS_X1, POS_Y2}, {POS_X2, POS_Y2}},
			{{POS_X2, POS_Y1}, {POS_X3, POS_Y1}, {POS_X2, POS_Y2}, {POS_X3, POS_Y2}},
			{{POS_X0, POS_Y2}, {POS_X1, POS_Y2}, {POS_X0, POS_Y3}, {POS_X1, POS_Y3}},
			{{POS_X1, POS_Y2}, {POS_X2, POS_Y2}, {POS_X1, POS_Y3}, {POS_X2, POS_Y3}},
			{{POS_X2, POS_Y2}, {POS_X3, POS_Y2}, {POS_X2, POS_Y3}, {POS_X3, POS_Y3}},
			{{POS_X0, POS_Y3}, {POS_X1, POS_Y3}, {POS_X0, POS_Y4}, {POS_X1, POS_Y4}},
			{{POS_X1, POS_Y3}, {POS_X2, POS_Y3}, {POS_X1, POS_Y4}, {POS_X2, POS_Y4}},
			{{POS_X2, POS_Y3}, {POS_X3, POS_Y3}, {POS_X2, POS_Y4}, {POS_X3, POS_Y4}},

		};
static TpsArea_t gAreaDiff[AREA_COUNT];
static TpsPoint_t gAdjustPrm[ADJUST_POINT];

static TpsSensorData gSensData;
static TpsEvent gPrev[4];
static sharp_smem_common_type * sh_smem_common_ptr = NULL;
static int gnHwRev = 0;

static uint32_t gdwWait = 1;
static int gnResult = 0;
static int gnFirstCalib = 0;

/*+-------------------------------------------------------------------------+*/
/*|	Prototype declaration													|*/
/*+-------------------------------------------------------------------------+*/
/* SPI access */
static int ShSpiTps_SpiWriteOne(uint8_t bReg, uint8_t Data, uint8_t bLen);
static int ShSpiTps_SpiRead(uint8_t bReg, uint8_t *Data_p, uint8_t bLen);

static int ShSpiTps_Command(SpiDev *poSpidev, unsigned int wCmd, void *pArg);

static int __init ShSpiTps_Init(void);
static void __exit ShSpiTps_Exit(void);
static int __devinit ShSpiTps_Probe(struct spi_device *spi);
static int __devexit ShSpiTps_Remove(struct spi_device *spi);

static void ShSpiTps_Connect2InputSys(WorkStruct *poWork);
static InputDev *ShSpiTps_CreateInputDev(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ConfigGPIO(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ReleaseGPIO(SpiTpsRec *poSpiTpsRec);

static int ShSpiTps_OpenCB(InputDev *poInDev);
static void ShSpiTps_CloseCB(InputDev *pInDev);
static void ShSpiTps_Shutdown(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Start(SpiTpsRec *poSpiTpsRec, int nCalib);
static void ShSpiTps_Stop(SpiTpsRec *poSpiTpsRec);

static void ShSpiTps_Reset(int nOnOff);
static void ShSpiTps_Standby(int nOnOff);
static void ShSpiTps_HsspClk(int nOnOff);
static void ShSpiTps_HsspData(int nOnOff);
static void ShSpiTps_PowerOn(void);
static void ShSpiTps_PowerOff(void);
static int ShSpiTps_ReStart(void *poPt);

static irqreturn_t ShSpiTps_IrqHandler(int nIrq, void *pvDevId);
static void ShSpiTps_FetchInt(WorkStruct *poWork);
static void ShSpiTps_Recover(WorkStruct *poWork);

static int ShSpiTps_DelayEnable(void *poPt);
static int ShSpiTps_SetState(SpiTpsRec *poSpiTpsRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, uint8_t bResult);
static int ShSpiTps_Enable_Phase1(SpiTpsRec *poSpiTpsRec, uint8_t bResult);
static int ShSpiTps_Enable_Phase2(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Disable(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_GetFwVer(SpiTpsRec *poSpiTpsRec);
#ifdef TPS_SETPARAM
static int ShSpiTps_WriteFirmParam(SpiDev *poSpidev);
#endif	/* TPS_SETPARAM */
static int ShSpiTps_TestMode_Start(SpiTpsRec *poSpiTpsRec, int nMode);
static int ShSpiTps_TestMode_Stop(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ParamSetting(SpiTpsRec *poSpiTpsRec, int nParam);
static int ShSpiTps_Calibration(void);
static int ShSpiTps_SetReCalibration(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_FirstCalibration(void *poPt);
static int ShSpiTps_SensorCheck(void);

static int ShSpiTps_FirmUp_Set(SpiTpsRec *poSpiTpsRec, int nMode);
static int ShSpiTps_Gpio_Reset(void);
static int ShSpiTps_Gpio_HsspClk(int nOnOff);
static int ShSpiTps_Gpio_HsspData(int nOnOff);
static int ShSpiTps_Gpio_HsspClkCh(int nInOut);
static int ShSpiTps_Gpio_HsspDataCh(int nInOut);
static int ShSpiTps_SDATACheck(void);
static int ShSpiTps_RunClock(int nNumCycles);
static uint8_t ShSpiTps_ReceiveBit(void);
static uint8_t ShSpiTps_ReceiveByte(void);
static int ShSpiTps_SendByte(uint8_t bData, int nNumBits);
static int ShSpiTps_DetectHiLoTransition(void);

/* Adjustment of coordinates */
static void ShSpiTps_Qsort(Qsort_t *pTable, int nTop, int nEnd);
static void ShSpiTps_RoundValue(short *pValue);
static int ShSpiTps_SetAdjustParam(SpiTpsRec *poSpiTpsRec, uint16_t *pParam);
static void ShSpiTps_AdjustPt(short *pX, short *pY);
static void ShSpiTps_PosInit(void);
static void ShSpiTps_PosSet(InputDev *pInDev, int nCnt, TpsEvent *poEv);
static DEFINE_MUTEX(goTpsAccessMutex);

/*+-----------------------------------------------------------------------------+*/
/*|	Macro definition															|*/
/*+-----------------------------------------------------------------------------+*/
#define	MINMAX(min,max,val)	((min)>(val) ? (min) : ((max)<(val) ? (max) : (val)))
#define	SET_POINT(val,x1,y1)	val.x=(x1);val.y=(y1)
#define	SET_AREA(val,x1,y1,x2,y2,x3,y3,x4,y4)	\
								val.p.x=x1;val.p.y=y1;val.q.x=x2;val.q.y=y2;	\
								val.r.x=x3;val.r.y=y3;val.s.x=x4;val.s.y=y4;

#define	SCLKHigh()			ShSpiTps_HsspClk(1);
#define	SCLKLow()			ShSpiTps_HsspClk(0);
#define	SetSDATAHigh()		ShSpiTps_HsspData(1);
#define	SetSDATALow()		ShSpiTps_HsspData(0);
#define	AssertXRES()		ShSpiTps_Reset(1);
#define	DeassertXRES()		ShSpiTps_Reset(0);
#define	SetSDATAHiZ()		ShSpiTps_HsspData(0);	\
							gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_DATA, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
#define	SetSDATAStrong()	gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_DATA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);

static const TpsDispatch gTpsDispatch[TPS_STATE_MAX][TPS_STATE_MAX] =
{
	/* [EVENT] HOVER */
	{
		{	0,	NULL,				NULL			},	/* [STATE] HOVER */
		{	1,	NULL,				ShSpiTps_PosSet	},	/* [STATE] DOWN */
	},
	/* [EVENT] DOWN */
	{
		{	1,	ShSpiTps_PosInit,	ShSpiTps_PosSet	},	/* [STATE] HOVER */
		{	1,	NULL,				ShSpiTps_PosSet	},	/* [STATE] DOWN */
	},
};

static SpiTpsRec *gpoSpiTpsRec = NULL;
static struct cdev goTpsCDev;
static struct class *gpoTpsClass;
static dev_t gnTpsDev;

/*+-------------------------------------------------------------------------+*/
/*|	SPI access																|*/
/*+-------------------------------------------------------------------------+*/
static int ShSpiTps_SpiWriteOne(uint8_t bReg, uint8_t Data, uint8_t bLen)
{
	int err;
	char tx_buf[40];
	char rx_buf[40];
	struct spi_message  m;
	struct spi_transfer t;
	int nI;

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, 40);
	memset(rx_buf, 0, 40);

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;

	t.len = 4+bLen;
	tx_buf[0] = 0x00;
	tx_buf[1] = 0xFF;
	tx_buf[2] = bReg;
	tx_buf[3] = 0x00;
	tx_buf[4] = Data;

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		err = spi_sync(gpoSpiTpsRec->mpoSpidev, &m);

		if (err < 0)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Write spi sync Error %d\n", err);
#endif /* TPS_PRNERR */
			return err;
		}

		/* ACK completion */
		if(rx_buf[2] == 0x62)
			return 0;

#ifdef TPS_PRNLOG
		printk(KERN_DEBUG "[ShSpiTps]Write Retry %d\n", nI);
#endif	/* TPS_PRNLOG */

		/* Waiting and retry */
		udelay(RETRY_WAIT);

	}
	return -1;
}

static int ShSpiTps_SpiRead(uint8_t bReg, uint8_t *Data_p, uint8_t bLen)
{
	int err;
	char tx_buf[40];
	struct spi_message  m;
	struct spi_transfer t;
	int nI;

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, 40);

	t.tx_buf = tx_buf;
	t.rx_buf = Data_p;

	t.len = bLen;
	tx_buf[0] = 0;
	tx_buf[1] = 0xFF;
	tx_buf[2] = bReg;
	tx_buf[3] = 0x01;

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		err = spi_sync(gpoSpiTpsRec->mpoSpidev, &m);

		if (err < 0)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Read spi sync Error %d\n", err);
#endif /* TPS_PRNERR */
			return err;
		}

		/* ACK completion */
		if(Data_p[2] == 0x62)
			return 0;

#ifdef TPS_PRNLOG
		printk(KERN_DEBUG "[ShSpiTps]Read Retry %d\n", nI);
#endif	/* TPS_PRNLOG */

		/* Waiting and retry */
		udelay(RETRY_WAIT);
	}
	return -1;
}

static int TpsIf_Open(struct inode *pINode, struct file *poFile)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Open(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	return 0;
}

static int TpsIf_Ioctl(struct inode *pINode, struct file *poFile, unsigned int wCmd, unsigned long dwArg)
{
	int nResult = -EINVAL;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Ioctl(PID:%ld,CMD:%d,ARG:%lx)\n", sys_getpid(), wCmd, dwArg);
#endif	/* TPS_PRNLOG */
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		nResult = ShSpiTps_Command(gpoSpiTpsRec->mpoSpidev, wCmd, (void *)dwArg);
		switch(nResult)
		{
		case -1:
			nResult = -EIO;
			break;
		case -2:
			nResult = -EINVAL;
			break;
		case -3:
			nResult = -EFAULT;
			break;
		}
	}
	mutex_unlock(&goTpsAccessMutex);
	return nResult;
}

static const struct file_operations goTpsIf_Fops =
{
	.owner	= THIS_MODULE,
	.open	= TpsIf_Open,
	.ioctl	= TpsIf_Ioctl,
};

int __init TpsIf_Setup(void)
{
	dev_t nMajor = 0;
	dev_t nMinor = 0;
	int nResult;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Setup(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	nResult = alloc_chrdev_region(&gnTpsDev, 0, 1, TPSIF_DEV_NAME);
	if(!nResult)
	{
		nMajor = MAJOR(gnTpsDev);
		nMinor = MINOR(gnTpsDev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]alloc_chrdev_region %d:%d\n", nMajor, nMinor);
#endif	/* TPS_PRNLOG */
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]alloc_chrdev_region error\n");
#endif	/* TPS_PRNERR */
		return -1;
	}

	cdev_init(&goTpsCDev, &goTpsIf_Fops);

	goTpsCDev.owner = THIS_MODULE;
	goTpsCDev.ops = &goTpsIf_Fops;

	nResult = cdev_add(&goTpsCDev, gnTpsDev, 1);
	if(nResult)
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]cdev_add error\n");
#endif	/* TPS_PRNERR */
		cdev_del(&goTpsCDev);
		return -1;
	}

	gpoTpsClass = class_create(THIS_MODULE, TPSIF_DEV_NAME);
	if(IS_ERR(gpoTpsClass))
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]class_create error\n");
#endif	/* TPS_PRNERR */
		cdev_del(&goTpsCDev);
		return -1;
	}
	device_create(gpoTpsClass, NULL, gnTpsDev, &goTpsCDev, TPSIF_DEV_NAME);
	return 0;
}

void __exit TpsIf_Cleanup(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Cleanup(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	device_destroy(gpoTpsClass, gnTpsDev);
	class_destroy(gpoTpsClass);
	cdev_del(&goTpsCDev);
}

module_init(TpsIf_Setup);
module_exit(TpsIf_Cleanup);

/*+-------------------------------------------------------------------------+*/
/*|	Touch panel Driver														|*/
/*+-------------------------------------------------------------------------+*/
static int ShSpiTps_Command(SpiDev *poSpidev, unsigned int wCmd, void *pArg)
{
	int nResult;
	uint8_t bData;
	int nResponse;
	int nHwRev;
	int ret = 0;
	Tps_send_data data;
	unsigned long dwArg = 0;
	unsigned short wParam[12];
#ifdef TPS_PRNLOG
	int nCnt;
#endif	/* TPS_PRNLOG */

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Command(PID:%ld,CMD:%d,ARG:%lx)\n", sys_getpid(), wCmd, (long)pArg);
#endif	/* TPS_PRNLOG */
	/* IOCTL that can be executed without device running */
	switch(wCmd)
	{
	case TPSDEV_FW_VERSION:
		return ShSpiTps_GetFwVer(gpoSpiTpsRec);
	case TPSDEV_FW_DOWNLOAD:
		return -2;
	case TPSDEV_FW_UPDATE:
		return -2;
	case TPSDEV_FW_UPDATE2:
		return -2;
	case TPSDEV_UPDATE_START:
		nResult = 0;
		nHwRev = ShSpiTps_FirmUp_Set(gpoSpiTpsRec, 1);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]FirmUp_Set(%d)\n", nHwRev);
#endif	/* TPS_PRNLOG */

		if(copy_to_user((int*)pArg, &nHwRev, sizeof(nHwRev)))
		{
			nResult = -3;
		}
		return nResult;
	case TPSDEV_UPDATE_STOP:
		nResult = 0;
		ShSpiTps_FirmUp_Set(gpoSpiTpsRec, 0);
		return nResult;
	case TPSDEV_GPIO_RESET:
		/* If not beginning to rewriting firmware */
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_Reset();
	case TPSDEV_GPIO_HSSPCLK:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_HsspClk((int)pArg);
	case TPSDEV_GPIO_HSSPDATA:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_HsspData((int)pArg);
	case TPSDEV_GPIO_CLK_INOUT:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* Switching I/O */
		return ShSpiTps_Gpio_HsspClkCh((int)pArg);
	case TPSDEV_GPIO_DATA_INOUT:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* Switching I/O */
		return ShSpiTps_Gpio_HsspDataCh((int)pArg);
	case TPSDEV_RUN_CLOCK:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		return ShSpiTps_RunClock((int)pArg);
	case TPSDEV_RECV_BYTE:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		nResult = 0;
		bData = ShSpiTps_ReceiveByte();
		if(copy_to_user((uint8_t*)pArg, &bData, sizeof(bData)))
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error(ReceiveByte)\n");
#endif	/* TPS_PRNERR */
			nResult = -3;
		}
		return nResult;
	case TPSDEV_SEND_BYTE:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		ret = copy_from_user(&data, (Tps_send_data *)pArg, sizeof(Tps_send_data));
		if (ret != 0) {
			return -2;
	    }
	    if(data.bReset != 0)
	    {
			/* reset */
			ShSpiTps_Gpio_Reset();
		}
		return ShSpiTps_SendByte(data.bData, data.nNumBits);
	case TPSDEV_DETECT_HILO:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		nResult = 0;
		nResponse = ShSpiTps_DetectHiLoTransition();
		if(copy_to_user((int*)pArg, &nResponse, sizeof(nResponse)))
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error(ReceiveByte)\n");
#endif	/* TPS_PRNERR */
			nResult = -3;
		}
		return nResult;
	}

	/* If Device is not open or Being written */
	if(gpoSpiTpsRec->mbIsActive == 0 || gpoSpiTpsRec->mbIsUpdate != 0)
		return -2;
	switch(wCmd)
	{
	case TPSDEV_ENABLE:
		/* Setting the touchpanel operating state */
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_OFF, TPS_CHECK_ON, TPS_RETURN_ON);
	case TPSDEV_DISABLE:
		/* Setting the touchpanel standby mode */
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_ON);
	case TPSDEV_START_TESTMODE:
		/* Starting the Test mode */
		return ShSpiTps_TestMode_Start(gpoSpiTpsRec, (int)pArg);
	case TPSDEV_STOP_TESTMODE:
		/* Stopping the Test mode */
		return ShSpiTps_TestMode_Stop(gpoSpiTpsRec);
	case TPSDEV_GET_SENSOR:
		nResult = -2;
		if(gpoSpiTpsRec->mbIsTestMode)
		{
			nResult = 0;
			/* copy acquisition data in the user space */
			if(copy_to_user((TpsSensorData*)pArg, &gSensData, sizeof(TpsSensorData)))
			{
#ifdef TPS_PRNERR
				printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error\n");
#endif	/* TPS_PRNERR */
				nResult = -3;
			}
		}
		return nResult;
	case TPSDEV_SET_FIRMPARAM:
		if(0 != copy_from_user(&dwArg,  (unsigned long *)pArg, sizeof(dwArg)))
			return -2;
		return ShSpiTps_ParamSetting(gpoSpiTpsRec, (int)dwArg);
	case TPSDEV_CALIBRATION_PARAM:
		if(pArg != NULL){
			if(0 != copy_from_user(&wParam,  (unsigned long *)pArg, sizeof(wParam)))
				return -2;
			return ShSpiTps_SetAdjustParam(gpoSpiTpsRec, wParam);
		}
		return ShSpiTps_SetAdjustParam(gpoSpiTpsRec, (uint16_t*)pArg);
	case TPSDEV_SLEEP_ON:
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	case TPSDEV_SLEEP_OFF:
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
	case TPSDEV_RECALIBRATION_IDAC:
		return ShSpiTps_SetReCalibration(gpoSpiTpsRec);
	}
	return -2;
}

module_init(ShSpiTps_Init);
module_exit(ShSpiTps_Exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("SPI TOUCH sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:SH_touchpanel");


/* Structure for SPI driver call */
static struct spi_driver goSpiTpsDriver =
{
	.driver =
	{
		.name  = SH_TOUCH_SPI_DEVNAME,
		.owner = THIS_MODULE,
		.bus	= &spi_bus_type,
	},
	.probe	  = ShSpiTps_Probe,
	.remove	  = __devexit_p(ShSpiTps_Remove),
};

static int __init ShSpiTps_Init(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Init(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	/* SPI driver use start */
	return spi_register_driver(&goSpiTpsDriver);
}

static void __exit ShSpiTps_Exit(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Exit(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	/* SPI driver use end */
	spi_unregister_driver(&goSpiTpsDriver);
}

static int __devinit ShSpiTps_Probe(struct spi_device *spi)
{
	struct msm_sh_spitps_platform_data *poSetupData;
	SpiTpsRec *poSpiTpsRec = NULL;
	int nResult;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Probe(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	if(!spi->dev.platform_data)
	{
		dev_err(&spi->dev, "platform device data is required\n");
		return -ENODEV;
	}

	/* The memory for touch panel driver information is secured.  */
	poSpiTpsRec = kzalloc(sizeof(SpiTpsRec), GFP_KERNEL);
	if(!poSpiTpsRec)
	{
		return -ENOMEM;
	}

	gpoSpiTpsRec = poSpiTpsRec;

	poSpiTpsRec->mpoSpidev			 = spi;
	poSetupData						 = spi->dev.platform_data;
	/* Setup information is obtained.(It defined by "board-xxxx.c") */
	poSpiTpsRec->mnIrqPin			 = poSetupData->gpio_irq;
	poSpiTpsRec->mnHsspClkPin		 = poSetupData->gpio_hssp_clk;
	poSpiTpsRec->mnHsspDataPin		 = poSetupData->gpio_hssp_data;
	poSpiTpsRec->mnReset			 = poSetupData->gpio_reset;
	poSpiTpsRec->mnStandby			 = poSetupData->gpio_standby;
	poSpiTpsRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poSpiTpsRec->mpfPinShutdownFunc  = poSetupData->gpio_teardown;

	poSpiTpsRec->mnState			 = TPS_STATE_HOVER;
	poSpiTpsRec->mbAccessState		 = 0;
	poSpiTpsRec->mbIsActive			 = 0;
	poSpiTpsRec->mbIsUpdate			 = 0;

	/* Semaphore initialization */
	init_MUTEX(&sem);

	/* getting the top address that structs of common SMEM */
	sh_smem_common_ptr = sh_smem_get_common_address();
	if( sh_smem_common_ptr != NULL)
	{
		/* getting the hardware revision */
		gnHwRev = sh_smem_common_ptr->sh_hw_revision;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]hw_revision(%d)\n", gnHwRev);
#endif	/* TPS_PRNLOG */
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]sh_smem_get_common_address is NULL\n");
#endif	/* TPS_PRNERR */
	}

	/* setting the GPIO */
	if(0 == (nResult = ShSpiTps_ConfigGPIO(poSpiTpsRec)))
	{
		INIT_WORK(&poSpiTpsRec->moIrqWork, ShSpiTps_FetchInt);
		dev_info(&spi->dev, "Detected %s, attempting to initialize\n", SH_TOUCH_SPI_DEVNAME);
		INIT_DELAYED_WORK(&poSpiTpsRec->moCmdQ, ShSpiTps_Connect2InputSys);
		schedule_delayed_work(&poSpiTpsRec->moCmdQ, msecs_to_jiffies(INITDELAY_TIMER));
		device_init_wakeup(&spi->dev, 1);
		return 0;
	}
	/* release the GPIO */
	ShSpiTps_ReleaseGPIO(poSpiTpsRec);
	kfree(poSpiTpsRec);
	return nResult;
}

static int __devexit ShSpiTps_Remove(struct spi_device *spi)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Remove(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	dev_info(&spi->dev, "removing driver\n");
	device_init_wakeup(&spi->dev, 0);
	if(gpoSpiTpsRec->mpoInDev)
	{
		dev_dbg(&spi->dev, "deregister from input system\n");
		input_unregister_device(gpoSpiTpsRec->mpoInDev);
		gpoSpiTpsRec->mpoInDev = NULL;
	}
	ShSpiTps_Shutdown(gpoSpiTpsRec);
	ShSpiTps_ReleaseGPIO(gpoSpiTpsRec);
	kfree(gpoSpiTpsRec);
	gpoSpiTpsRec = NULL;
	return 0;
}

static void ShSpiTps_Connect2InputSys(WorkStruct *poWork)
{
	SpiTpsRec *poSpiTpsRec = container_of(poWork, SpiTpsRec, moCmdQ.work);
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Connect2InputSys(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	poSpiTpsRec->mpoInDev = ShSpiTps_CreateInputDev(poSpiTpsRec);
	if(poSpiTpsRec->mpoInDev)
	{
		if(input_register_device(poSpiTpsRec->mpoInDev) != 0)
		{
			dev_err(poDev, "Failed to register with input system\n");
			input_free_device(poSpiTpsRec->mpoInDev);
		}
	}
}

static InputDev *ShSpiTps_CreateInputDev(SpiTpsRec *poSpiTpsRec)
{
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;
	InputDev *pInDev = input_allocate_device();

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]CreateInputDev(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	if(pInDev)
	{
		pInDev->name = SH_TOUCH_SPI_DEVNAME;
		pInDev->phys = poSpiTpsRec->mcPhysInfo;
		pInDev->id.vendor  = QCVENDOR_ID;
		pInDev->id.product = QCPRODUCT_ID;
		pInDev->id.version = QCVERSION_ID;
		pInDev->open = ShSpiTps_OpenCB;
		pInDev->close = ShSpiTps_CloseCB;
		/* registering valid event */
		__set_bit(EV_KEY, pInDev->evbit);
		__set_bit(EV_ABS, pInDev->evbit);
		/* Vertual key */
		__set_bit(KEY_MENU,   pInDev->keybit);
    	__set_bit(KEY_HOME,   pInDev->keybit);
    	__set_bit(KEY_BACK,   pInDev->keybit);
    	__set_bit(KEY_SEARCH, pInDev->keybit);
		__set_bit(ABS_MT_TOUCH_MAJOR, pInDev->absbit);
		__set_bit(ABS_MT_POSITION_X, pInDev->absbit);
		__set_bit(ABS_MT_POSITION_Y, pInDev->absbit);
		__set_bit(ABS_MT_WIDTH_MAJOR, pInDev->absbit);
		input_set_drvdata(pInDev, poSpiTpsRec);
		/* setting the range of Event parameters */
		input_set_abs_params(pInDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
#if 0
		input_set_abs_params(pInDev, ABS_MT_POSITION_X,  0, SH_TOUCH_MAX_X, 0, 0);
		input_set_abs_params(pInDev, ABS_MT_POSITION_Y,  0, SH_TOUCH_MAX_Y, 0, 0);
#else
		/* Vertual key */
		input_set_abs_params(pInDev, ABS_MT_POSITION_X,  0, SH_TOUCH_VKEY_MAX_X, 0, 0);
		input_set_abs_params(pInDev, ABS_MT_POSITION_Y,  0, SH_TOUCH_VKEY_MAX_Y, 0, 0);
#endif
		input_set_abs_params(pInDev, ABS_MT_WIDTH_MAJOR, 0, SH_TOUCH_MAX_DIAGONAL, 0, 0);
	}
	else
	{
		dev_err(poDev, "Failed to allocate input device for %s\n", SH_TOUCH_SPI_DEVNAME);
	}
	return pInDev;
	
}

static int ShSpiTps_ConfigGPIO(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ConfigGPIO(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec == NULL)
		return -EINVAL;
	return poSpiTpsRec->mpfPinSetupFunc();
}

static int ShSpiTps_ReleaseGPIO(SpiTpsRec *poSpiTpsRec)
{
	if(poSpiTpsRec == NULL)
		return -EINVAL;
	/* release the GPIO */
	dev_info(&poSpiTpsRec->mpoSpidev->dev, "releasing gpio pins %d,%d,%d,%d,%d\n",
			 poSpiTpsRec->mnIrqPin, poSpiTpsRec->mnHsspClkPin, poSpiTpsRec->mnHsspDataPin,
			 poSpiTpsRec->mnReset, poSpiTpsRec->mnStandby);
	poSpiTpsRec->mpfPinShutdownFunc();
	return 0;
}

static int ShSpiTps_OpenCB(InputDev *poInDev)
{
	SpiTpsRec *poSpiTpsRec = input_get_drvdata(poInDev);

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]OpenCB(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	dev_dbg(&poSpiTpsRec->mpoSpidev->dev, "ENTRY: input_dev open callback\n");
	return ShSpiTps_Start(poSpiTpsRec, TPS_CALIB_ON);
}

static void ShSpiTps_CloseCB(InputDev *pInDev)
{
	SpiTpsRec *poSpiTpsRec = input_get_drvdata(pInDev);
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]CloseCB(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	dev_dbg(poDev, "ENTRY: close callback\n");
	ShSpiTps_Shutdown(poSpiTpsRec);
}

static void ShSpiTps_Shutdown(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Shutdown(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	/* If touchpanel driver is active */
	if(poSpiTpsRec->mbIsActive)
	{
		mutex_lock(&goTpsAccessMutex);
		ShSpiTps_Stop(poSpiTpsRec);
		mutex_unlock(&goTpsAccessMutex);
		/* release the work-memory */
		flush_work(&poSpiTpsRec->moIrqWork);
	}
}

//volatile uint32_t dwSpeed = 2000000;
volatile uint32_t dwSpeed = 1000000;	/* 1.0Mbps*/

static int ShSpiTps_Start(SpiTpsRec *poSpiTpsRec, int nCalib)
{
	int err;
	struct task_struct *p;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Start(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	mutex_lock(&goTpsAccessMutex);
	/* disabling a touchpanel */
	poSpiTpsRec->mbIsEnable = 0;
	poSpiTpsRec->mbAccessState |= TPS_DISABLE_API;
	poSpiTpsRec->mbIsActive = 1;
	/* Setting the Normal mode */
	poSpiTpsRec->mbIsTestMode = 0;
	poSpiTpsRec->mbAdjustEnable = 0;
	poSpiTpsRec->mbIsFirst = 1;
	/* Initialization */
	gnFirstCalib = 0;

	/* Setting the SPI */
	poSpiTpsRec->mpoSpidev->bits_per_word = 8;
	poSpiTpsRec->mpoSpidev->mode          = SPI_MODE_1;

	err = spi_setup(poSpiTpsRec->mpoSpidev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]spi_setup = %d\n", err);
#endif	/* TPS_PRNLOG */
	if (err < 0)
	{
		poSpiTpsRec->mbIsActive = 0;
		mutex_unlock(&goTpsAccessMutex);
		return -EIO;
	}

	/* Power ON */
	if(0 != ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_OFF, TPS_CHECK_ON, TPS_RETURN_ON))
	{
		poSpiTpsRec->mbIsActive = 0;
		mutex_unlock(&goTpsAccessMutex);
		return -EIO;
	}
	if(nCalib != TPS_CALIB_OFF)
	{
		/* executes it by the thread */
		p = kthread_run(ShSpiTps_FirstCalibration, poSpiTpsRec, "shspitps_firstcalibration");
		/* If it cannot start a thread, doing the synchronous execution */
		if(IS_ERR(p))
			ShSpiTps_FirstCalibration(poSpiTpsRec);
		/* perform mutex_unlock at the time of the calibration end */
		return 0;
	}

	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static void ShSpiTps_Stop(SpiTpsRec *poSpiTpsRec)
{
	/* disabling a touchpanel */
	ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_OFF);
	poSpiTpsRec->mbIsActive = 0;
}

static void ShSpiTps_Reset(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_RESET , 1);
	} else {
		gpio_direction_output(SH_TOUCH_RESET , 0);
	}
}

static void ShSpiTps_Standby(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_STBY , 1);
	} else {
		gpio_direction_output(SH_TOUCH_STBY , 0);
	}
}

static void ShSpiTps_HsspClk(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_HSSP_CLK , 0);
	} else {
		gpio_direction_output(SH_TOUCH_HSSP_CLK , 1);
	}
}

static void ShSpiTps_HsspData(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_HSSP_DATA, 0);
	} else {
		gpio_direction_output(SH_TOUCH_HSSP_DATA, 1);
	}
}

static void ShSpiTps_PowerOn(void)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PowerOn(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNDEB */
	ShSpiTps_Reset(1);
	udelay(TPS_RESET_INTERVAL);
	ShSpiTps_Reset(0);
	ShSpiTps_Standby(0);
	mdelay(300);
}

static void ShSpiTps_PowerOff(void)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PowerOff(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNDEB */
	/* setting to the standby state */
	ShSpiTps_Standby(1);
}

static int ShSpiTps_ReStart(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;
	int nI;
	int nResult = -1;
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ReStart(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNDEB */
	/* to the hover-state */
	if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mbValid != 0)
	{
		/* Report */
		if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit != NULL)
		{
			gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit();
		}
		for(nI = 0; nI < 4; nI++)
		{
			/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
			if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos != NULL)
			{
				gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos(poSpiTpsRec->mpoInDev, 0, NULL);
			}
		}
		input_sync(poSpiTpsRec->mpoInDev);
	}
	poSpiTpsRec->mnState = TPS_STATE_HOVER;
	/* setting to the standby state */
	ShSpiTps_PowerOff();
	ShSpiTps_PowerOn();
#ifdef TPS_SETPARAM
	/* setting parameter */
	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);
	if(nResult < 0)
	{
#ifdef TPS_PRNERR
		printk(KERN_ERR "WriteFirmParam Error %d\n", nResult);
#endif	/* TPS_PRNERR */
	}
#endif	/* TPS_SETPARAM */
	/* allowing the next interrupt */
	enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static irqreturn_t ShSpiTps_IrqHandler(int nIrq, void *pvDevId)
{
	SpiTpsRec *poSpiTpsRec = pvDevId;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]IrqHandler(PID:%ld,IRQ:%02X)\n", sys_getpid(), nIrq);
#endif	/* TPS_PRNLOG */

	disable_irq_nosync(nIrq);
	schedule_work(&poSpiTpsRec->moIrqWork);
	return IRQ_HANDLED;
}

static void ShSpiTps_FetchInt(WorkStruct *poWork)
{
	struct task_struct *p;
	SpiTpsRec *poSpiTpsRec = container_of(poWork, SpiTpsRec, moIrqWork);
//	SpiDev *poSpidev = poSpiTpsRec->mpoSpidev;
	InputDev *pInDev = poSpiTpsRec->mpoInDev;
	uint8_t rx_buf[40];
	uint8_t  bTt_m, bTt_s;
	int nI, nJ;
	int nNextState;
#ifdef TPS_PRNLOG
	const char *StaName[] = { "HOVER", "DOWN "};
#endif	/* TPS_PRNLOG */
	TpsEvent oEv[4];
	uint8_t bInt = 0;

#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]FetchInt(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNDEB */

	if(gnFirstCalib == 1)
	{
		/* acquire a sensor value */
		if(0 == ShSpiTps_SpiRead(0x00, gSense, 171))
		{
			/* coordinate */
			gSensData.wPosX = (uint16_t)(gSense[7] << 8) + ((uint16_t)gSense[8]);
			gSensData.wPosY = (uint16_t)(gSense[9] << 8) + ((uint16_t)gSense[10]);
			gSensData.bCounter = (uint8_t)(gSense[5] >> 6);
			memcpy(&gSensData.bSensData[0], &gSense[11], 160);

			/* interrupt cancellation */
			bInt = gSense[4];
			if(bInt & 0x80){
				bInt &= ~0x80;
			}else{
				bInt |= 0x80;
			}
			/* Normal mode */
			bInt &= ~0x70;
			ShSpiTps_SpiWriteOne(0x00, bInt, 1);
		}
		gnFirstCalib = 0;
		return;
	}

	mutex_lock(&goTpsAccessMutex);
	/* If Test mode */
	if(poSpiTpsRec->mbIsTestMode)
	{
		/* acquire a sensor value */
		if(0 == ShSpiTps_SpiRead(0x00, gSense, 171))
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps] HST_MODE [0x%2X]\n", gSense[4]);
printk(KERN_DEBUG "[ShSpiTps] MD[0x%2X] ST[0x%2X]\n", gSense[5], gSense[6]);
printk(KERN_DEBUG "[ShSpiTps] NewData[0x%2X]\n", (gSense[5] >> 6));
printk(KERN_DEBUG "[ShSpiTps] X1[%4d] Y1[%4d]\n",(uint16_t)(gSense[7] << 8) + ((uint16_t)gSense[8]),(uint16_t)(gSense[9] << 8) + ((uint16_t)gSense[10]));
#endif	/* TPS_PRNLOG */
			/* coordinate */
			gSensData.wPosX = (uint16_t)(gSense[7] << 8) + ((uint16_t)gSense[8]);
			gSensData.wPosY = (uint16_t)(gSense[9] << 8) + ((uint16_t)gSense[10]);
			gSensData.bCounter = (uint8_t)(gSense[5] >> 6);
			memcpy(&gSensData.bSensData[0], &gSense[11], 160);

			/* interrupt cancellation */
			bInt = gSense[4];
			if(bInt & 0x80){
				bInt &= ~0x80;
			}else{
				bInt |= 0x80;
			}
			if(0 == ShSpiTps_SpiWriteOne(0x00, bInt, 1))
			{
				/* wait */
				msleep(50);
			}
		}
		mutex_unlock(&goTpsAccessMutex);

		/* allowing the next interrupt */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		return;
	}
	if(poSpiTpsRec->mbIsEnable == 0)
	{
		mutex_unlock(&goTpsAccessMutex);
		return;
	}
	/* Reading register */
	if(0 != ShSpiTps_SpiRead(0x01, rx_buf, 32))
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]spi read error\n");
#endif	/* TPS_PRNERR */
		ShSpiTps_Stop(poSpiTpsRec);
		INIT_DELAYED_WORK(&poSpiTpsRec->moCmdQ, ShSpiTps_Recover);
		schedule_delayed_work(&poSpiTpsRec->moCmdQ, msecs_to_jiffies(RECOVER_TIMER));
		mutex_unlock(&goTpsAccessMutex);
		return;
	}

	/* Negative_finger bit check */
	if(rx_buf[31] & 0x40)
	{
		/* executes it by the thread */
		p = kthread_run(ShSpiTps_ReStart, poSpiTpsRec, "shspitps_restart");
		if(IS_ERR(p)){
			/* allowing the next interrupt */
			enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			mutex_unlock(&goTpsAccessMutex);
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]ReStart error\n");
#endif	/* TPS_PRNERR */
		}
		/* perform mutex_unlock after having restarted */
		return;
	}

	bTt_m   = rx_buf[4];
	bTt_s   = rx_buf[5];
	/* getting the Coordinate */
	oEv[0].mbID = rx_buf[11] >> 4;
	oEv[0].mwPosX = (uint16_t)(rx_buf[ 6] << 8) + ((uint16_t)rx_buf[ 7]);
	oEv[0].mwPosY = (uint16_t)(rx_buf[ 8] << 8) + ((uint16_t)rx_buf[ 9]);
	oEv[0].mwPosZ = (uint16_t)rx_buf[10];

	oEv[1].mbID = rx_buf[11] & 0x0f;
	oEv[1].mwPosX = (uint16_t)(rx_buf[12] << 8) + ((uint16_t)rx_buf[13]);
	oEv[1].mwPosY = (uint16_t)(rx_buf[14] << 8) + ((uint16_t)rx_buf[15]);
	oEv[1].mwPosZ = (uint16_t)rx_buf[16];

	oEv[2].mbID = rx_buf[24] >> 4;
	oEv[2].mwPosX = (uint16_t)(rx_buf[19] << 8) + ((uint16_t)rx_buf[20]);
	oEv[2].mwPosY = (uint16_t)(rx_buf[21] << 8) + ((uint16_t)rx_buf[22]);
	oEv[2].mwPosZ = (uint16_t)rx_buf[23];

	oEv[3].mbID = rx_buf[24] & 0x0f;
	oEv[3].mwPosX = (uint16_t)(rx_buf[25] << 8) + ((uint16_t)rx_buf[26]);
	oEv[3].mwPosY = (uint16_t)(rx_buf[27] << 8) + ((uint16_t)rx_buf[28]);
	oEv[3].mwPosZ = (uint16_t)rx_buf[29];

#ifdef TPS_PRNLOG
//	printk(KERN_DEBUG "[ShSpiTps] MD[0x%2X] ST[0x%2X]\n", bTt_m, bTt_s);
//	printk(KERN_DEBUG "[ShSpiTps] X1[%4d] Y1[%4d] Z1[%4d]\n", oEv[0].mwPosX, oEv[0].mwPosY, oEv[0].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X2[%4d] Y2[%4d] Z2[%4d]\n", oEv[1].mwPosX, oEv[1].mwPosY, oEv[1].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X3[%4d] Y3[%4d] Z3[%4d]\n", oEv[2].mwPosX, oEv[2].mwPosY, oEv[2].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X4[%4d] Y4[%4d] Z4[%4d]\n", oEv[3].mwPosX, oEv[3].mwPosY, oEv[3].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] 12ID[0x%2X] 34ID[0x%2X] GCn[%4d] GID[%4d]\n", rx_buf[11], rx_buf[24], rx_buf[17], rx_buf[18]);

	printk(KERN_DEBUG "-------------------------------------\n");
	printk(KERN_DEBUG "MD[0x%2X] ST[0x%2X] GCn[%4d] GID[%4d]\n", bTt_m, bTt_s, rx_buf[17], rx_buf[18]);
	printk(KERN_DEBUG "1 %2d(%4d,%4d,%4d)\n", oEv[0].mbID, oEv[0].mwPosX, oEv[0].mwPosY, oEv[0].mwPosZ);
	printk(KERN_DEBUG "2 %2d(%4d,%4d,%4d)\n", oEv[1].mbID, oEv[1].mwPosX, oEv[1].mwPosY, oEv[1].mwPosZ);
	printk(KERN_DEBUG "3 %2d(%4d,%4d,%4d)\n", oEv[2].mbID, oEv[2].mwPosX, oEv[2].mwPosY, oEv[2].mwPosZ);
	printk(KERN_DEBUG "4 %2d(%4d,%4d,%4d)\n", oEv[3].mbID, oEv[3].mwPosX, oEv[3].mwPosY, oEv[3].mwPosZ);
	printk(KERN_DEBUG "Ver %2x\n"            , rx_buf[30]);
#endif	/* TPS_PRNLOG */
	/* Ignoring LargeObjectDetect */
	bTt_s &= 0x0f;
	/* if touched number is 4 or more */
	if(bTt_s > 4)
		bTt_s = 4;
	if(bTt_s == 0)
	{
		nNextState = TPS_STATE_HOVER;
	}
	else
	{
		nNextState = TPS_STATE_DOWN;
	}
	/* If state can transition */
	if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mbValid != 0)
	{
		/* Report */
		if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportInit != NULL)
		{
			gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportInit();
		}
		for(nI = 0; nI < bTt_s; nI++)
		{
			if(oEv[nI].mwPosX > SH_TOUCH_MAX_X)
			{
				oEv[nI].mwPosX = SH_TOUCH_MAX_X;
			}
			if(oEv[nI].mwPosY > SH_TOUCH_MAX_Y)
			{
				oEv[nI].mwPosY = SH_TOUCH_MAX_Y;
			}
			/* if Adjustment is enabled */
			if(poSpiTpsRec->mbAdjustEnable != 0)
			{
				/* Adjusting the coordinates */
				ShSpiTps_AdjustPt(&oEv[nI].mwPosX, &oEv[nI].mwPosY);
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]Adjust (%4d,%4d)\n", oEv[nI].mwPosX, oEv[nI].mwPosY);
#endif	/* TPS_PRNDEB */
			}

			/* Because a high rank layer does not react when notify of the same coordinate, do not return the same coordinate */
			for(nJ = 0; nJ < nI; nJ++)
			{
				/* The length and breadth same coordinate moves one pixel of coordinate */
				if(oEv[nI].mwPosX == oEv[nJ].mwPosX && oEv[nI].mwPosY == oEv[nJ].mwPosY)
				{
					if(oEv[nI].mwPosX != 0)
						oEv[nI].mwPosX--;
					else
						oEv[nI].mwPosX++;
					if(oEv[nI].mwPosY != 0)
						oEv[nI].mwPosY--;
					else
						oEv[nI].mwPosY++;
				}
			}

		}
		/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
		if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportPos != NULL)
		{
			gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportPos(pInDev, bTt_s, oEv);
		}
		input_sync(pInDev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]State[%s]->[%s] OK\n", StaName[poSpiTpsRec->mnState], StaName[nNextState]);
#endif	/* TPS_PRNLOG */
		poSpiTpsRec->mnState = nNextState;
	}
	else
	{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]State[%s]->[%s] NG\n", StaName[poSpiTpsRec->mnState], StaName[nNextState]);
#endif	/* TPS_PRNLOG */
	}
	/* allowing the next interrupt */
	enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	mutex_unlock(&goTpsAccessMutex);
}

static void ShSpiTps_Recover(WorkStruct *poWork)
{
	int nResult;
	SpiTpsRec *poSpiTpsRec;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Recover(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	poSpiTpsRec = container_of(poWork, SpiTpsRec, moCmdQ.work);

	dev_info(&poSpiTpsRec->mpoSpidev->dev, "touchpanel recovery requested\n");

	nResult = ShSpiTps_Start(poSpiTpsRec, TPS_CALIB_OFF);
	if(nResult != 0)
	{
		dev_err(&poSpiTpsRec->mpoSpidev->dev, "recovery failed with (nResult=%d)\n", nResult);
	}
}

static int ShSpiTps_DelayEnable(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_DelayEnable(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	/* Power ON */
	ShSpiTps_PowerOn();
	gnResult = ShSpiTps_Enable_Phase2(poSpiTpsRec);
	up(&sem);
	return 0;
}

static int ShSpiTps_SetState(SpiTpsRec *poSpiTpsRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, uint8_t bResult)
{
	uint8_t bNew;
	int nResult = 0;

	/* get semaphore */
	down(&sem);

	bValue &= bMask;
	bNew = (poSpiTpsRec->mbAccessState & ~bMask) | bValue;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]SetState[%02X]->[%02X]\n", poSpiTpsRec->mbAccessState, bNew);
#endif	/* TPS_PRNLOG */
	/* If doing the double-check */
	if(bCheck)
	{
		if((poSpiTpsRec->mbAccessState & bMask) == bValue)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]StateCheck NG\n");
#endif	/* TPS_PRNERR */
			/* release semaphore */
			up(&sem);
			return -2;
		}
	}
	/* recording the state */
	poSpiTpsRec->mbAccessState = bNew;
	if(poSpiTpsRec->mbIsActive)
	{
		/* if ready to operate */
		if(bNew == 0x00)
		{
			nResult = ShSpiTps_Enable_Phase1(poSpiTpsRec, bResult);
			return nResult;
		}
		else
		{
			nResult = ShSpiTps_Disable(poSpiTpsRec);
		}
	}
	/* release semaphore */
	up(&sem);

	return nResult;
}

static int ShSpiTps_Enable_Phase1(SpiTpsRec *poSpiTpsRec, uint8_t bResult)
{
	struct task_struct *p;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	/* Initialize */
	gnResult = 0;

	/* If there is no disincentive with the off state now */
	if(poSpiTpsRec->mbIsEnable == 0 && poSpiTpsRec->mbAccessState == 0x00)
	{
		if(poSpiTpsRec->mbIsTestMode == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable ON\n");
#endif	/* TPS_PRNLOG */
			/* executes it by the thread */
			p = kthread_run(ShSpiTps_DelayEnable, poSpiTpsRec, "shspitps_delayenable");
			/* If it cannot start a thread, doing the synchronous execution */
			if(IS_ERR(p))
				ShSpiTps_DelayEnable(poSpiTpsRec);
			/* If there is the return value */
			if(bResult)
			{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Result Wait... \n");
#endif	/* TPS_PRNLOG */
				down(&sem);
				up(&sem);
				return gnResult;
			}
			return 0;
		}
		/* enabling a touchpanel */
		poSpiTpsRec->mbIsEnable = 1;
	}

	/* release semaphore */
	up(&sem);

	return 0;
}

static int ShSpiTps_Enable_Phase2(SpiTpsRec *poSpiTpsRec)
{
	int nResult;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable_Phase2(%d:%d)\n", poSpiTpsRec->mbIsActive, poSpiTpsRec->mbAccessState);
#endif	/* TPS_PRNLOG */

	/* Initialize the state */
	poSpiTpsRec->mnState = TPS_STATE_HOVER;

#ifdef TPS_SETPARAM
	/* setting parameter */
	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);
	if(nResult < 0)
	{
		ShSpiTps_PowerOff();
		return -1;
	}
#endif	/* TPS_SETPARAM */

	/* Registering interrupt handler */
	nResult = request_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), &ShSpiTps_IrqHandler,
					 IRQF_TRIGGER_LOW | IRQF_DISABLED,
				     SH_TOUCH_SPI_DEVNAME, poSpiTpsRec);
	if(nResult < 0)
	{
#ifdef TPS_PRNERR
		printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_TOUCH_SPI_DEVNAME, nResult);
#endif	/* TPS_PRNERR */
		ShSpiTps_PowerOff();
		return -3;
	}
	poSpiTpsRec->mbIsFirst= 0;
	/* enabling a touchpanel */
	poSpiTpsRec->mbIsEnable = 1;

	return 0;
}

static int ShSpiTps_Disable(SpiTpsRec *poSpiTpsRec)
{
	int nI;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Disable(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	/* If there is a disincentive in ON now */
	if(poSpiTpsRec->mbIsEnable != 0 && poSpiTpsRec->mbAccessState != 0x00)
	{
		if(poSpiTpsRec->mbIsTestMode == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Disable OFF\n");
#endif	/* TPS_PRNLOG */
			/* to the hover-state */
			if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mbValid != 0)
			{
				/* Report */
				if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit != NULL)
				{
					gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit();
				}
				for(nI = 0; nI < 4; nI++)
				{
					/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
					if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos != NULL)
					{
						gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos(poSpiTpsRec->mpoInDev, 0, NULL);
					}
				}
				input_sync(poSpiTpsRec->mpoInDev);
			}
			poSpiTpsRec->mnState = TPS_STATE_HOVER;
			/* Interrupt Unsubscribe */
			free_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), poSpiTpsRec);
			/* setting to the standby state */
			ShSpiTps_PowerOff();
		}
		/* disabling a touchpanel */
		poSpiTpsRec->mbIsEnable = 0;
	}
	/* If the first start, do Power On and Off */
	else if(poSpiTpsRec->mbIsFirst)
	{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]First\n");
#endif	/* TPS_PRNLOG */
		ShSpiTps_PowerOn();
		ShSpiTps_PowerOff();
		poSpiTpsRec->mbIsFirst = 0;
	}
	return 0;
}

static int ShSpiTps_GetFwVer(SpiTpsRec *poSpiTpsRec)
{
	int nResult = -1;
	uint8_t rx_buf[8];

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}

	if(0 == ShSpiTps_SpiRead(0x1B, rx_buf, 5))
		nResult = rx_buf[4];
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Fw Version %02X\n", nResult);
#endif	/* TPS_PRNLOG */

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	return nResult;
}

#ifdef TPS_SETPARAM
static int ShSpiTps_WriteFirmParam(SpiDev *poSpidev)
{
	uint8_t bGest = 0;
	uint8_t rx_buf[5];

	if(0 != ShSpiTps_SpiRead(0x1E, rx_buf, 5))
		return -1;

	bGest = rx_buf[4];
	bGest &= ~0x0f;
	bGest |= gbSetParam;

	if(0 != ShSpiTps_SpiWriteOne(0x1E, bGest, 1))
		return -1;
	/* check */
	if(0 != ShSpiTps_SpiRead(0x1E, rx_buf, 5))
		return -1;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Active distance %02X\n", rx_buf[4] & 0x0f);
#endif	/* TPS_PRNLOG */
	return 0;
}
#endif	/* TPS_SETPARAM */

static int ShSpiTps_TestMode_Start(SpiTpsRec *poSpiTpsRec, int nMode)
{
	int nResult = -1;
	uint8_t bSt = 0;
	uint8_t rx_buf[6];

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_TestMode_Start(PID:%ld,%02X)\n", sys_getpid(), nMode);
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec->mbIsTestMode)
	{
		/* if already in test mode, return an error */
		return -2;
	}

	/* parameter check*/
	if(nMode < 0 || nMode > 3)
		return -3;

	/* if touchpanel is Enabled */
	if(poSpiTpsRec->mbIsEnable)
	{
		/* Interrupt Unsubscribe */
		free_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), poSpiTpsRec);
	}
	else
	{
		/* Power ON */
		ShSpiTps_PowerOn();
	}

	/*+-------------------------------------------------------------------------+*/
	/*|	Test mode selection														|*/
	/*+-------------------------------------------------------------------------+*/
	if(ShSpiTps_SpiRead(0x00, rx_buf, 5) == 0)
	{
		bSt = rx_buf[4];

		/* Test mode */
		switch(nMode)
		{
			case 0:
				/* Test mode 0 */
				bSt &= ~0x70;
				bSt |= 0x40;
				break;
			case 1:
				/* Test mode 1 */
				bSt &= ~0x70;
				bSt |= 0x50;
				break;
			case 2:
				/* Test mode 2 */
				bSt &= ~0x70;
				bSt |= 0x60;
				break;
			case 3:
				/* Test mode 3 */
				bSt &= ~0x70;
				bSt |= 0x70;
				break;
		}
		if(ShSpiTps_SpiWriteOne(0x00, bSt, 1) == 0)
		{
			/* Clear a buffer */
			memset(gSense, 0x00, 171);
			/* Registering interrupt handler */
			if(request_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), &ShSpiTps_IrqHandler,
								 IRQF_TRIGGER_LOW | IRQF_DISABLED,
							     SH_TOUCH_SPI_DEVNAME, poSpiTpsRec) == 0)
			{
				/* Setting the Test mode */
				poSpiTpsRec->mbIsTestMode = 1;
				/* Wait */
				mdelay(30);
				return 0;
			}
#ifdef TPS_PRNERR
			printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_TOUCH_SPI_DEVNAME, nResult);
#endif	/* TPS_PRNERR */
			nResult = -3;
		}
	}

	/* When it was an error, return it to a state */
	/* Interrupt Unsubscribe */
	free_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), poSpiTpsRec);
	/* It resets and power supply ON */
	ShSpiTps_PowerOn();

	if(poSpiTpsRec->mbIsEnable)
	{
		/* Registering interrupt handler */
		if(request_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), &ShSpiTps_IrqHandler,
							 IRQF_TRIGGER_LOW | IRQF_DISABLED,
						     SH_TOUCH_SPI_DEVNAME, poSpiTpsRec) < 0)
		{
#ifdef TPS_PRNERR
			printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_TOUCH_SPI_DEVNAME, nResult);
#endif	/* TPS_PRNERR */
			return -3;
		}
	}
	else
	{
		/* setting to the standby state */
		ShSpiTps_PowerOff();
	}
	return nResult;
}

static int ShSpiTps_TestMode_Stop(SpiTpsRec *poSpiTpsRec)
{
	int nResult = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_TestMode_Stop(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	if(!poSpiTpsRec->mbIsTestMode)
	{
		/* if not in test mode, return an error */
		return -2;
	}

	/* Interrupt Unsubscribe */
	free_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), poSpiTpsRec);
	/* It resets and power supply ON */
	ShSpiTps_PowerOn();

	/* if touchpanel is Enabled */
	if(poSpiTpsRec->mbIsEnable)
	{
		/* Registering interrupt handler */
		nResult = request_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), &ShSpiTps_IrqHandler,
							 IRQF_TRIGGER_LOW | IRQF_DISABLED,
						     SH_TOUCH_SPI_DEVNAME, poSpiTpsRec);
		if(nResult < 0)
		{
#ifdef TPS_PRNERR
			printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_TOUCH_SPI_DEVNAME, nResult);
#endif	/* TPS_PRNERR */
			return -3;
		}
	}
	else
	{
		/* setting to the standby state */
		ShSpiTps_PowerOff();
	}
	/* to Normal mode */
	poSpiTpsRec->mbIsTestMode = 0;

	return 0;
}

static int ShSpiTps_ParamSetting(SpiTpsRec *poSpiTpsRec, int nParam)
{
#ifdef TPS_SETPARAM
	int nResult = 0;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ParamSetting %d\n", nParam);
#endif	/* TPS_PRNLOG */
	/* check a parameter */
	if(nParam < 1 || nParam > 15)
		return -2;

	/* store a parameter */
	gbSetParam = (uint8_t)nParam;
	gbSetParam &= 0x0f;

	/* perform the off state in next Enable */
	if(!poSpiTpsRec->mbIsEnable)
		return 0;

	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);

	return nResult;
#else
	return -3;
#endif	/* TPS_SETPARAM */
}

static int ShSpiTps_Calibration(void)
{
	uint8_t rx_buf[5];
	int nResult = -1;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Calibration\n");
#endif	/* TPS_PRNLOG */

	/* carry out calibration */
	if(0 == ShSpiTps_SpiWriteOne(0x1C, 0x02, 1))
	{
		/* Waiting is necessary(1.35sec) */
		msleep(TPS_CALIB_WAIT);
		/* calibration end check */
		if(0 == ShSpiTps_SpiRead(0x1C, rx_buf, 5))
		{
			/* If calibration is not over, wait more */
			if(rx_buf[4] & 0x02)
			{
				/* Waiting is necessary(1.35sec) */
				msleep(TPS_CALIB_WAIT);
			}
			nResult = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Calibration OK\n");
#endif	/* TPS_PRNLOG */
		}
		else
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Read Error\n");
#endif /* TPS_PRNERR */
		}
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Write Error\n");
#endif /* TPS_PRNERR */
	}
	return nResult;
}

static int ShSpiTps_SetReCalibration(SpiTpsRec *poSpiTpsRec)
{
	int nResult = -1;

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	/*+-------------------------------------------------------------------------+*/
	/*|	IDAC Calibration														|*/
	/*+-------------------------------------------------------------------------+*/
	nResult = ShSpiTps_Calibration();

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	return nResult;
}

static int ShSpiTps_FirstCalibration(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;
	int nResult;
	int nI;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_FirstCalibration(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	memset(gSense, 0x00, 171);

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		nResult = ShSpiTps_SensorCheck();
		if(nResult == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Calibration Sensor Check OK \n");
#endif	/* TPS_PRNLOG */
			break;
		}
		else if(nResult == 1)
		{
			/* carry out calibration */
			if(ShSpiTps_Calibration() == 0)
			{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration enforcement \n");
#endif	/* TPS_PRNERR */
			}
			else
			{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Error \n");
#endif	/* TPS_PRNERR */
			}
		}
		else
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps] Sensor Timeout!! \n");
#endif	/* TPS_PRNERR */
			break;
		}
	}
	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static int ShSpiTps_SensorCheck(void)
{
	uint8_t bSt = 0;
	uint8_t rx_buf[6];
	int nCnt;
	int nIndex;
	int nResult = 0;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SensorCheck(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	if(0 != ShSpiTps_SpiRead(0x00, rx_buf, 5))
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]SensorCheck Error\n");
#endif /* TPS_PRNERR */
		return 0;
	}
	else
	{
		bSt = rx_buf[4];
		/* Test mode 0 */
		bSt &= ~0x70;
		bSt |= 0x40;
		if(0 != ShSpiTps_SpiWriteOne(0x00, bSt, 1))
		{
			/* reboot it without calibration */
			ShSpiTps_PowerOn();
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]SensorCheck Error\n");
#endif /* TPS_PRNERR */
			return 0;
		}
	}

	/* Clear a buffer */
	memset(gSense, 0x00, 171);
	/* Registering interrupt handler */
	enable_irq(MSM_GPIO_TO_INT(gpoSpiTpsRec->mnIrqPin));

	gnFirstCalib = 1;
	msleep(10);
	for(nCnt = 0; nCnt < TPS_CALIB_TIMEOUT; nCnt++)
	{
		if(gnFirstCalib == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]TestMode INT interlocking movement!  Cnt = %d \n", nCnt);
			for(nCnt = 0; nCnt < 16; nCnt++)
			{
printk(KERN_DEBUG "[ShSpiTps]%2d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d\n", nCnt, gSensData.bSensData[(16*0+nCnt)],
																				   gSensData.bSensData[(16*1+nCnt)],
																				   gSensData.bSensData[(16*2+nCnt)],
																				   gSensData.bSensData[(16*3+nCnt)],
																				   gSensData.bSensData[(16*4+nCnt)],
																				   gSensData.bSensData[(16*5+nCnt)],
																				   gSensData.bSensData[(16*6+nCnt)],
																				   gSensData.bSensData[(16*7+nCnt)],
																				   gSensData.bSensData[(16*8+nCnt)],
																				   gSensData.bSensData[(16*9+nCnt)]);
			}
#endif	/* TPS_PRNLOG */
			/* Initialization */
			gnFirstCalib = 0;
			nResult = 0;
			for(nIndex = 0; nIndex < 160; nIndex++)
			{
				if(gSensData.bSensData[nIndex] < TPS_CALIB_MIN || gSensData.bSensData[nIndex] > TPS_CALIB_MAX)
				{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Outside Range(Calibration) SNS[%d] = %d \n", nIndex, gSensData.bSensData[nIndex]);
#endif	/* TPS_PRNERR */
					nResult = 1;
				}
			}
			return nResult;
		}
		msleep(1);
	}
	/* Timeout */
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]TestMode INT TimeOut Error !!\n");
#endif	/* TPS_PRNERR */
	/* Initialization */
	gnFirstCalib = 0;
	/* reboot it without calibration */
	ShSpiTps_PowerOn();
	nResult = -1;
	return nResult;
}

static int ShSpiTps_FirmUp_Set(SpiTpsRec *poSpiTpsRec, int nMode)
{
	if(nMode == 1)
	{
		poSpiTpsRec->mbIsUpdate = 1;

		if(poSpiTpsRec->mbIsEnable)
		{
			/* interrupt disabled */
			disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		}
	}
	else
	{
		poSpiTpsRec->mbIsUpdate = 0;

		if(poSpiTpsRec->mbIsEnable)
		{
			/* Resuming interrupt */
			enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		}
		else
		{
			/* Power Off */
			ShSpiTps_PowerOff();
		}
	}
	return gnHwRev;
}

static int ShSpiTps_Gpio_Reset(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Gpio_Reset\n");
#endif	/* TPS_PRNLOG */
	gpio_direction_output(SH_TOUCH_RESET , 0);
	udelay(TPS_RESET_INTERVAL);
	gpio_direction_output(SH_TOUCH_RESET , 1);

	return 0;
}

static int ShSpiTps_Gpio_HsspClk(int nOnOff)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Gpio_HsspClk (%d)\n", nOnOff);
#endif	/* TPS_PRNLOG */
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_HSSP_CLK , 0);
	} else {
		gpio_direction_output(SH_TOUCH_HSSP_CLK , 1);
	}
	return 0;
}

static int ShSpiTps_Gpio_HsspData(int nOnOff)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Gpio_HsspData (%d)\n", nOnOff);
#endif	/* TPS_PRNLOG */
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_HSSP_DATA, 0);
	} else {
		gpio_direction_output(SH_TOUCH_HSSP_DATA, 1);
	}
    return 0;
}

static int ShSpiTps_Gpio_HsspClkCh(int nInOut)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]HsspClkCh (%d)\n", nInOut);
#endif	/* TPS_PRNLOG */
	if(nInOut == 0)
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_CLK, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	}
	else
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_CLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	}
    return 0;
}

static int ShSpiTps_Gpio_HsspDataCh(int nInOut)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]HsspDataCh (%d)\n", nInOut);
#endif	/* TPS_PRNLOG */
	if(nInOut == 0)
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_DATA, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	}
	else
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_HSSP_DATA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	}
    return 0;
}

static int ShSpiTps_SDATACheck(void)
{
	if(gpio_get_value(SH_TOUCH_HSSP_DATA) == 1) {
		return 1;
	}
	return 0;
}

static int ShSpiTps_RunClock(int nNumCycles)
{
	int nI;

	for(nI = 0; nI < nNumCycles; nI++)
	{
		SCLKLow();
		udelay(gdwWait);	/* Waiting */
		SCLKHigh();
		udelay(gdwWait);	/* Waiting */
	}
	return 0;
}

static uint8_t ShSpiTps_ReceiveBit(void)
{
	SCLKLow();
	udelay(gdwWait);	/* Waiting */
	SCLKHigh();
	udelay(gdwWait);	/* Waiting */
	return ShSpiTps_SDATACheck();
}

static uint8_t ShSpiTps_ReceiveByte(void)
{
	int nI;
	uint8_t bData = 0x00;

	for(nI = 0; nI < 8; nI++)
	{
		bData = (bData << 1) + ShSpiTps_ReceiveBit();
	}
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ReceiveByte %x\n", bData);
#endif	/* TPS_PRNLOG */
	return bData;
}

static int ShSpiTps_SendByte(uint8_t bData, int nNumBits)
{
	int nI = 0;

	for(nI = 0; nI < nNumBits; nI++)
	{
		if(bData & 0x80)
		{
			/* Send a '1' */
			SetSDATAHigh();
			SCLKHigh();
			SCLKLow();
		}
		else
		{
			/* Send a '0' */
			SetSDATALow();
			SCLKHigh();
			SCLKLow();
		}
		bData = bData << 1;
	}
	return 0;
}

static int ShSpiTps_DetectHiLoTransition(void)
{
	int nCnt;
	int nIs_Response = -1;

	SetSDATAHiZ();

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		udelay(10);

		if(ShSpiTps_SDATACheck() == 0)
		{
			nIs_Response = 0;
		}
	}

	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]First Low Wait\n");
#endif	/* TPS_PRNERR */
	}

	nIs_Response = -1;

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		SCLKHigh();
		udelay(1);

		SCLKLow();
		udelay(10);

		if(ShSpiTps_SDATACheck() != 0)
		{
			nIs_Response = 0;
		}
	}

	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]High Wait\n");
#endif	/* TPS_PRNERR */
	}

	nIs_Response = -1;
	msleep(10);

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		udelay(10);

		if(ShSpiTps_SDATACheck() == 0)
		{
			nIs_Response = 0;
		}
	}
	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Low Wait\n");
#endif	/* TPS_PRNERR */
	}

	SetSDATAStrong();
//	ShSpiTps_SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

	return nIs_Response;
}

/*+-----------------------------------------------------------------------------+*/
/*|	adjustment of Coordinate													|*/
/*+-----------------------------------------------------------------------------+*/
static void ShSpiTps_Qsort(Qsort_t *pTable, int nTop, int nEnd)
{
    int i, j;
    int nCenter;
	Qsort_t Swap;

    i = nTop;
    j = nEnd;

    nCenter = pTable[(nTop + nEnd) / 2].mValue;

	while(1)
	{
		while (pTable[i].mValue < nCenter)
			i++;
		while (nCenter < pTable[j].mValue)
			j--;
		if(i >= j)
			break;
		memcpy(&Swap, &pTable[i], sizeof(Qsort_t));
		memcpy(&pTable[i], &pTable[j], sizeof(Qsort_t));
		memcpy(&pTable[j], &Swap, sizeof(Qsort_t));
		i++;
		j--;
	}
	if(nTop < i - 1)
		ShSpiTps_Qsort(pTable, nTop, i - 1);
	if(j + 1 <  nEnd)
		ShSpiTps_Qsort(pTable, j + 1, nEnd);
}

static void ShSpiTps_RoundValue(short *pValue)
{
	Qsort_t pTable[6];
	int nI;

	for(nI = 0; nI < 6; nI++)
	{
		pTable[nI].mNo = nI;
		pTable[nI].mValue = pValue[nI];
	}
	ShSpiTps_Qsort(pTable, 0, 5);
	pValue[pTable[0].mNo] = pValue[pTable[1].mNo];
	pValue[pTable[5].mNo] = pValue[pTable[4].mNo];
}

static int ShSpiTps_SetAdjustParam(SpiTpsRec *poSpiTpsRec, uint16_t *pParam)
{
	int nI;
	TpsPoint_t sD[ADJUST_POINT];
	short nDiff[2][6];

	/* adjustment of Coordinate is Invalid */
	if(pParam == NULL)
	{
		poSpiTpsRec->mbAdjustEnable = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetAdjustParam(NULL)\n");
#endif	/* TPS_PRNLOG */
		return 0;
	}
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetAdjustParam(%4d,%4d)(%4d,%4d)\n", pParam[ 0], pParam[ 1], pParam[ 2], pParam[ 3]);
printk(KERN_DEBUG "                                 (%4d,%4d)(%4d,%4d)\n", pParam[ 4], pParam[ 5], pParam[ 6], pParam[ 7]);
printk(KERN_DEBUG "                                 (%4d,%4d)(%4d,%4d)\n", pParam[ 8], pParam[ 9], pParam[10], pParam[11]);
#endif	/* TPS_PRNLOG */

	/* Parameter checking :Effective range(+-100) */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		if(pParam[nI*2+0] > gBasePt[nI].x + POS_LIMIT ||
		   pParam[nI*2+0] < gBasePt[nI].x - POS_LIMIT)
			return -2;
		if(pParam[nI*2+1] > gBasePt[nI].y + POS_LIMIT ||
		   pParam[nI*2+1] < gBasePt[nI].y - POS_LIMIT)
			return -2;
	}

	/* Save Parameters */
	SET_POINT(gAdjustPrm[0], pParam[ 0], pParam[ 1]);
	SET_POINT(gAdjustPrm[1], pParam[ 2], pParam[ 3]);
	SET_POINT(gAdjustPrm[2], pParam[ 4], pParam[ 5]);
	SET_POINT(gAdjustPrm[3], pParam[ 6], pParam[ 7]);
	SET_POINT(gAdjustPrm[4], pParam[ 8], pParam[ 9]);
	SET_POINT(gAdjustPrm[5], pParam[10], pParam[11]);
#if 0	/* Changing the calculation method of diff */
	/* calculate diff value */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		sD[nI].x = (gAdjustPrm[nI].x - gBasePt[nI].x) * 3 / 4;
		sD[nI].y = (gAdjustPrm[nI].y - gBasePt[nI].y) * 3 / 4;
	}
#else
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		nDiff[0][nI] = (gAdjustPrm[nI].x - gBasePt[nI].x);
		nDiff[1][nI] = (gAdjustPrm[nI].y - gBasePt[nI].y);
	}
	/* truncate the maximum and minimum */
	ShSpiTps_RoundValue(nDiff[0]);			/* X */
	ShSpiTps_RoundValue(nDiff[1]);			/* Y */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		sD[nI].x = nDiff[0][nI] * 75 / 100;
		sD[nI].y = nDiff[1][nI] * 75 / 100;
	}
#endif	/* Changing the calculation method of diff */
	/* store the blurring value of each four area corners */
	/*                     |-------p-------| |-------q-------| |-------r-------| |-------s-------|*/
	SET_AREA(gAreaDiff[ 0], 0      , 0      , sD[0].x, 0      , 0      , sD[0].y, sD[0].x, sD[0].y);
	SET_AREA(gAreaDiff[ 1], sD[0].x, 0      , sD[1].x, 0      , sD[0].x, sD[0].y, sD[1].x, sD[1].y);
	SET_AREA(gAreaDiff[ 2], sD[1].x, 0      , 0      , 0      , sD[1].x, sD[1].y, 0      , sD[1].y);
	SET_AREA(gAreaDiff[ 3], 0      , sD[0].y, sD[0].x, sD[0].y, 0      , sD[2].y, sD[2].x, sD[2].y);
	SET_AREA(gAreaDiff[ 4], sD[0].x, sD[0].y, sD[1].x, sD[1].y, sD[2].x, sD[2].y, sD[3].x, sD[3].y);
	SET_AREA(gAreaDiff[ 5], sD[1].x, sD[1].y, 0      , sD[1].y, sD[3].x, sD[3].y, 0      , sD[3].y);
	SET_AREA(gAreaDiff[ 6], 0      , sD[2].y, sD[2].x, sD[2].y, 0      , sD[4].y, sD[4].x, sD[4].y);
	SET_AREA(gAreaDiff[ 7], sD[2].x, sD[2].y, sD[3].x, sD[3].y, sD[4].x, sD[4].y, sD[5].x, sD[5].y);
	SET_AREA(gAreaDiff[ 8], sD[3].x, sD[3].y, 0      , sD[3].y, sD[5].x, sD[5].y, 0      , sD[5].y);
	SET_AREA(gAreaDiff[ 9], 0      , sD[4].y, sD[4].x, sD[4].y, 0      , 0      , sD[4].x, 0      );
	SET_AREA(gAreaDiff[10], sD[4].x, sD[4].y, sD[5].x, sD[5].y, sD[4].x, 0      , sD[5].x, 0      );
	SET_AREA(gAreaDiff[11], sD[5].x, sD[5].y, 0      , sD[5].y, sD[5].x, 0      , 0      , 0      );
	/* to valid an adjustment of Coordinate */
	poSpiTpsRec->mbAdjustEnable = 1;
	return 0;
}

static void ShSpiTps_AdjustPt(short *pX, short *pY)
{
	int nI;
	int32_t lXPQ;
	int32_t lXRS;
	int32_t lX;
	int32_t lYPR;
	int32_t lYQS;
	int32_t lY;

	/* divide the area */
	for(nI = 0; nI < AREA_COUNT; nI++)
	{
		if(gAreaRect[nI].p.x <= *pX && gAreaRect[nI].s.x > *pX &&
		   gAreaRect[nI].p.y <= *pY && gAreaRect[nI].s.y > *pY)
		{
			break;
		}
	}
	/* If not belong to any area, do not adjust */
	if(nI != AREA_COUNT)
	{
		/* do an adjustment of Coordinate */
		lXPQ = (((gAreaDiff[nI].q.x*DOUBLE_ACCURACY) - (gAreaDiff[nI].p.x*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].q.x - gAreaRect[nI].p.x)) * (*pX - gAreaRect[nI].p.x) + (gAreaDiff[nI].p.x*DOUBLE_ACCURACY);
		lXRS = (((gAreaDiff[nI].s.x*DOUBLE_ACCURACY) - (gAreaDiff[nI].r.x*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].s.x - gAreaRect[nI].r.x)) * (*pX - gAreaRect[nI].r.x) + (gAreaDiff[nI].r.x*DOUBLE_ACCURACY);
		lX   = ((lXRS - lXPQ) / (gAreaRect[nI].r.y - gAreaRect[nI].p.y)) * (*pY - gAreaRect[nI].p.y) + lXPQ;
		lYPR = (((gAreaDiff[nI].r.y*DOUBLE_ACCURACY) - (gAreaDiff[nI].p.y*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].r.y - gAreaRect[nI].p.y)) * (*pY - gAreaRect[nI].p.y) + (gAreaDiff[nI].p.y*DOUBLE_ACCURACY);
		lYQS = (((gAreaDiff[nI].s.y*DOUBLE_ACCURACY) - (gAreaDiff[nI].q.y*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].s.y - gAreaRect[nI].q.y)) * (*pY - gAreaRect[nI].q.y) + (gAreaDiff[nI].q.y*DOUBLE_ACCURACY);
		lY   = ((lYQS - lYPR) / (gAreaRect[nI].q.x - gAreaRect[nI].p.x)) * (*pX - gAreaRect[nI].p.x) + lYPR;
		*pX = *pX - (short)(lX / DOUBLE_ACCURACY);
		*pY = *pY - (short)(lY / DOUBLE_ACCURACY);
	}
	/* to be adjusted inside the range */
	*pX = MINMAX(0, SH_TOUCH_MAX_X, *pX);
	*pY = MINMAX(0, SH_TOUCH_MAX_Y, *pY);
}

static void ShSpiTps_PosInit(void)
{
	int nI;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PosInit\n");
#endif	/* TPS_PRNLOG */
	/* clearing the Last Info */
	for(nI = 0; nI < 4; nI++)
	{
		gPrev[nI].mbID = 0xff;
		gPrev[nI].mwPosX = 0xffff;
		gPrev[nI].mwPosY = 0xffff;
		gPrev[nI].mwPosZ = 0xffff;
	}
}
static void ShSpiTps_PosSet(InputDev *pInDev, int nCnt, TpsEvent *poEv)
{
	int nI, nJ;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PosSet(%d)\n", nCnt);
#endif	/* TPS_PRNLOG */

	/* update a finger notifying you of earlier */
	for(nI = 0; nI < 4; nI++)
	{
		/* Previously notified */
		if(gPrev[nI].mbID != 0xff)
		{
			/* do loop in count of event */
			for(nJ = 0; nJ < nCnt; nJ++)
			{
				/* Found the same number(Drag) */
				if(gPrev[nI].mbID == poEv[nJ].mbID)
				{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[Tps]DRAG(%d:%d %d,%4d,%4d,%4d)\n", nI, nJ, poEv[nJ].mbID, poEv[nJ].mwPosX, poEv[nJ].mwPosY, poEv[nJ].mwPosZ);
#endif	/* TPS_PRNLOG */
					/* updating the coordinate */
					input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 100); 
					input_report_abs(pInDev, ABS_MT_POSITION_X,  poEv[nJ].mwPosX);
					input_report_abs(pInDev, ABS_MT_POSITION_Y,  poEv[nJ].mwPosY);
					input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, poEv[nJ].mwPosZ);
					input_mt_sync(pInDev);
					/* updating the management information */
					gPrev[nI].mbID = poEv[nJ].mbID;
					gPrev[nI].mwPosX = poEv[nJ].mwPosX;
					gPrev[nI].mwPosY = poEv[nJ].mwPosY;
					gPrev[nI].mwPosZ = poEv[nJ].mwPosZ;
					/* handled flag */
					poEv[nJ].mbID = 0xff;
					break;
				}
			}
			/* event of do not touch(Touch up) */
			if(nJ == nCnt)
			{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[Tps]UP  (%d:- %d,%4d,%4d,%4d)\n", nI, gPrev[nI].mbID, gPrev[nI].mwPosX, gPrev[nI].mwPosY, gPrev[nI].mwPosZ);
#endif	/* TPS_PRNLOG */
				/* notice of the previous Information */
				input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 0); 
				input_report_abs(pInDev, ABS_MT_POSITION_X,  gPrev[nI].mwPosX);
				input_report_abs(pInDev, ABS_MT_POSITION_Y,  gPrev[nI].mwPosY);
				input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, gPrev[nI].mwPosZ);
				input_mt_sync(pInDev);
				/* clearing management information */
				gPrev[nI].mbID = 0xff;
				gPrev[nI].mwPosX = 0xffff;
				gPrev[nI].mwPosY = 0xffff;
				gPrev[nI].mwPosZ = 0xffff;
			}
		}
	}
/* packing before */
	for(nI = 0; nI < 3; nI++)
	{
		/* If vacant */
		if(gPrev[nI].mbID == 0xff)
		{
			for(nJ = nI + 1; nJ < 4; nJ++)
			{
				/* Searching in gap */
				if(gPrev[nJ].mbID != 0xff)
				{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[Tps](%d<-%d)\n", nI, nJ);
#endif	/* TPS_PRNLOG */
					gPrev[nI].mbID = gPrev[nJ].mbID;
					gPrev[nI].mwPosX = gPrev[nJ].mwPosX;
					gPrev[nI].mwPosY = gPrev[nJ].mwPosY;
					gPrev[nI].mwPosZ = gPrev[nJ].mwPosZ;
					gPrev[nJ].mbID = 0xff;
					gPrev[nJ].mwPosX = 0xffff;
					gPrev[nJ].mwPosY = 0xffff;
					gPrev[nJ].mwPosZ = 0xffff;
					break;
				}
			}
		}
	}
	/* handling the remaining events */
	for(nJ = 0; nJ < nCnt; nJ++)
	{
		/* found the not handled event */
		if(poEv[nJ].mbID != 0xff)
		{
			for(nI = 0; nI < 4; nI++)
			{
				/* found the space */
				if(gPrev[nI].mbID == 0xff)
				{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[Tps]DOWN(%d:%d %d,%4d,%4d,%4d)\n", nI, nJ, poEv[nJ].mbID, poEv[nJ].mwPosX, poEv[nJ].mwPosY, poEv[nJ].mwPosZ);
#endif	/* TPS_PRNLOG */
					/* updating the coordinate */
					input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 100); 
					input_report_abs(pInDev, ABS_MT_POSITION_X,  poEv[nJ].mwPosX);
					input_report_abs(pInDev, ABS_MT_POSITION_Y,  poEv[nJ].mwPosY);
					input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, poEv[nJ].mwPosZ);
					input_mt_sync(pInDev);
					/* updating the management information */
					gPrev[nI].mbID = poEv[nJ].mbID;
					gPrev[nI].mwPosX = poEv[nJ].mwPosX;
					gPrev[nI].mwPosY = poEv[nJ].mwPosY;
					gPrev[nI].mwPosZ = poEv[nJ].mwPosZ;
					/* handled flag */
					poEv[nJ].mbID = 0xff;
					break;
				}
			}
		}
	}
#ifdef TPS_PRNLOG
	for(nI = 0; nI < 4; nI++)
	{
printk(KERN_DEBUG "[Tps][%d] %2X,%5d,%5d,%5d)\n", nI, gPrev[nI].mbID, gPrev[nI].mwPosX, gPrev[nI].mwPosY, gPrev[nI].mwPosZ);
	}
#endif	/* TPS_PRNLOG */
}

/*+-------------------------------------------------------------------------+*/
/*|	External public I/F														|*/
/*+-------------------------------------------------------------------------+*/
void msm_spitps_flipchange(int nFlipState)
{
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* if a flip was opening */
		if(nFlipState == 0x00)
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_FLIP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
		else
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_FLIP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	}
	mutex_unlock(&goTpsAccessMutex);
}

void msm_tps_setsleep(int nIsSleep)
{
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* if sleep has been lifted */
		if(nIsSleep == 0x00)
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
		else
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	}
	mutex_unlock(&goTpsAccessMutex);
}

void msm_tps_shutdown(void)
{
	int nIsActive = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]msm_tps_shutdown(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* If touchpanel driver is active */
		if(gpoSpiTpsRec->mbIsActive)
		{
			/* disabling a touchpanel */
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_OFF);
			gpoSpiTpsRec->mbIsActive = 0;
			nIsActive = 1;
		}
		/* Power Off Sequence */
		ShSpiTps_Standby(0);
		ShSpiTps_Reset(1);
		udelay(TPS_RESET_INTERVAL);
		ShSpiTps_Reset(0);
		mdelay(300);
		ShSpiTps_SpiWriteOne(0x1C, 0x01, 1);
	}
	mutex_unlock(&goTpsAccessMutex);

	if(nIsActive == 1)
	{
		/* release the work-memory */
		flush_work(&gpoSpiTpsRec->moIrqWork);
	}
}

