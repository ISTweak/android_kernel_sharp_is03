
#define	ROTATE_90
#define	KBD_DEBPRN

/*+-------------------------------------------------------------------------+*/
/*|	INCLUDE FILE															|*/
/*+-------------------------------------------------------------------------+*/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <mach/gpio.h>
#include <mach/msm_i2ckbd.h>
#include <linux/syscalls.h>

/*+-------------------------------------------------------------------------+*/
/*|	DECLARE																	|*/
/*+-------------------------------------------------------------------------+*/

/* Key register number */
/*+---------------------------------+*/
/*|	Power-on reset register			|*/
/*+---------------------------------+*/
#define	KPD_RSTINTCLR			0x84
#define	KPD_RSTINTCLR_IRQCLR	0x01
/*+---------------------------------+*/
/*|	Clock mode register				|*/
/*+---------------------------------+*/
#define	KPD_CLKEN				0x8A
#define	KPD_CLKEN_TIMOSCEN2		0x20
#define	KPD_CLKEN_TIMOSCEN1		0x10
#define	KPD_CLKEN_TIMOSCEN0		0x08
#define	KPD_CLKEN_TIMEN			0x04
#define	KPD_CLKEN_KBDEN			0x01
/*+---------------------------------+*/
/*|	IRQ status register				|*/
/*+---------------------------------+*/
#define	KPD_IRQST				0x91
#define	KPD_IRQST_PORIRQ		0x80
#define	KPD_IRQST_KBDIRQ		0x40
#define	KPD_IRQST_RESERVED		0x20
/* (0x10 Empty) */
#define	KPD_IRQST_TIM2IRQ		0x08
#define	KPD_IRQST_TIM1IRQ		0x04
#define	KPD_IRQST_TIM0IRQ		0x02
#define	KPD_IRQST_GPIIRQ		0x01
/*+---------------------------------+*/
/*|	Wait time setting register		|*/
/*+---------------------------------+*/
#define	KPD_KBDSETTLE			0x01
/*+---------------------------------+*/
/*|	Debounce setting register		|*/
/*+---------------------------------+*/
#define	KPD_KBDBOUNCE			0x02
/*+---------------------------------+*/
/*|	Matrix setting register			|*/
/*+---------------------------------+*/
#define	KPD_KBDSIZE				0x03
/*+---------------------------------+*/
/*|	Special key setting register	|*/
/*+---------------------------------+*/
#define	KPD_KBDDEDCFG_H			0x04
#define	KPD_KBDDEDCFG_L			0x05
/*+---------------------------------+*/
/*|	Key code register				|*/
/*+---------------------------------+*/
#define	KPD_KEYCODE0			0x0B
#define	KPD_KEYCODE1			0x0C
#define	KPD_KEYCODE2			0x0D
#define	KPD_KEYCODE3			0x0E
/*+---------------------------------+*/
/*|	Event register					|*/
/*+---------------------------------+*/
#define	KPD_EVTCODE				0x10
/*+---------------------------------+*/
/*|	KBD low IRQ register			|*/
/*+---------------------------------+*/
#define	KPD_KBDRIS				0x06
#define	KPD_KBDRIS_RELINT		0x08
#define	KPD_KBDRIS_REVINT		0x04
#define	KPD_KBDRIS_RKLINT		0x02
#define	KPD_KBDRIS_RSINT		0x01
/*+---------------------------------+*/
/*|	KBD mask IRQ register			|*/
/*+---------------------------------+*/
#define	KPD_KBDMIS				0x07
#define	KPD_KBDMIS_MELINT		0x08
#define	KPD_KBDMIS_MEVINT		0x04
#define	KPD_KBDMIS_MKLINT		0x02
#define	KPD_KBDMIS_MSINT		0x01
/*+---------------------------------+*/
/*|	KBD IRQ clearness register		|*/
/*+---------------------------------+*/
#define	KPD_KBDIC				0x08
#define	KPD_KBDIC_SFOFF			0x80
#define	KPD_KBDIC_EVTIC			0x02
#define	KPD_KBDIC_KBDIC			0x01
/*+---------------------------------+*/
/*|	KBD mask register				|*/
/*+---------------------------------+*/
#define	KPD_KBDMSK				0x09
#define	KPD_KBDMSK_MSKELNT		0x08
#define	KPD_KBDMSK_MSKEINT		0x04
#define	KPD_KBDMSK_MSKKLI		0x02
#define	KPD_KBDMSK_MSKSINT		0x01
/*+---------------------------------+*/
/*|	KBD change feature setting register|*/
/*+---------------------------------+*/
#define	KPD_KBDMFS				0x8F

/*+---------------------------------+*/
/*|	Pul setting register			|*/
/*+---------------------------------+*/
#define	KPD_IOPC0_H				0xAA
#define	KPD_IOPC0_L				0xAB
#define	KPD_IOPC1_H				0xAC
#define	KPD_IOPC1_L				0xAD
#define	KPD_IOPC2_H				0xAE
#define	KPD_IOPC2_L				0xAF

#define	KPD_I2C_RETRY			10

#define	KPD_KEYPRESS			1
#define	KPD_KEYRELEASE			0

#define	KPD_KEYBUFF_SIZE		4		/* Key buffer size */
#define	KPD_KEYCODE_MASK		0x7f
#define	KPD_INVALID_KEY			0x7f

#define	KPD_KEY_MAX				128

/* Command */
enum
{
	QPHYSLEN			= 128,
	QCVENDOR_ID			= 0x5143,
	QCPRODUCT_ID		= 1,
	QCVERSION_ID		= 1
};

#define	INITDELAY_TIMER			600
#define	RECOVER_TIMER			1000

/*+-------------------------------------------------------------------------+*/
/*|	TYPEDEF																	|*/
/*+-------------------------------------------------------------------------+*/
typedef struct i2ckybd_record	I2cKbdRec;
typedef struct i2c_client		I2cClient;
typedef struct i2c_device_id	I2cDevID;
typedef struct work_struct		WorkStruct;
typedef struct input_dev		InputDev;
typedef struct device			Device;

struct i2ckybd_record
{
	I2cClient *mpoClient;
	InputDev *mpoInDev;
	int		mnProductInfo;
	char	mcPhysInfo[QPHYSLEN];
	int		mnIrqPin;
	int		(*mpfPinSetupFunc)(void);
	void	(*mpfPinShutdownFunc)(void);
	uint8_t	mbIsActive;					/* Status(1:Active/0:Inactive) */
	struct delayed_work moCmdQ;
	WorkStruct moIrqWork;
	uint16_t	mwPrev[KPD_KEYBUFF_SIZE];
};

typedef struct
{
	uint8_t	mbRegAdr;					/* Register address */
	uint8_t mbData;						/* Data */
} OctKey_I2cWriteData;

/*+-------------------------------------------------------------------------+*/
/*|	TABLE																	|*/
/*+-------------------------------------------------------------------------+*/

static const uint16_t gwOctKeyTable[KPD_KEY_MAX] =
{
					/*      Normal    +Shift     +Alt    +ShiftAlt */
	KEY_1,			/* 00:[   1    ][   !    ][        ][        ] */
	KEY_7,			/* 01:[   7    ][   '    ][Manners ][        ] */
	KEY_Q,			/* 02:[   Q    ][        ][  TAB   ][        ] */
	KEY_U,			/* 03:[   U    ][        ][        ][        ] */
	KEY_S,			/* 04:[   S    ][        ][        ][        ] */
	KEY_K,			/* 05:[   K    ][        ][        ][        ] */
	KEY_C,			/* 06:[   C    ][        ][        ][        ] */
	KEY_DOT,		/* 07:[   .    ][   >    ][   ]    ][   }    ] */
	0,				/* 08:                                         */
	0,				/* 09:                                         */
	0,				/* 0A:                                         */
	0,				/* 0B:                                         */
	0,				/* 0C:                                         */
	0,				/* 0D:                                         */
	0,				/* 0E:                                         */
	0,				/* 0F:                                         */
	KEY_2,			/* 10:[   2    ][   "    ][        ][        ] */
	KEY_8,			/* 11:[   8    ][   (    ][ VolUp  ][        ] */
	KEY_W,			/* 12:[   W    ][        ][        ][        ] */
	KEY_I,			/* 13:[   I    ][        ][        ][        ] */
	KEY_D,			/* 14:[   D    ][        ][        ][        ] */
	KEY_L,			/* 15:[   L    ][        ][   ;    ][   +    ] */
	KEY_V,			/* 16:[   V    ][        ][        ][        ] */
	KEY_CHARACTER,	/* 17:[Character][       ][        ][        ] */
	0,				/* 18:                                         */
	0,				/* 19:                                         */
	0,				/* 1A:                                         */
	0,				/* 1B:                                         */
	0,				/* 1C:                                         */
	0,				/* 1D:                                         */
	0,				/* 1E:                                         */
	0,				/* 1F:                                         */
	KEY_3,			/* 20:[   3    ][   #    ][        ][        ] */
	KEY_9,			/* 21:[   9    ][   )    ][VolDown ][        ] */
	KEY_E,			/* 22:[   E    ][        ][        ][        ] */
	KEY_O,			/* 23:[   O    ][        ][        ][        ] */
	KEY_F,			/* 24:[   F    ][        ][        ][        ] */
	KEY_SLASH,		/* 25:[   /    ][   ?    ][   :    ][   *    ] */
	KEY_B,			/* 26:[   B    ][        ][        ][        ] */
	KEY_SPACE,		/* 27:[  SPACE ][        ][        ][        ] */
	KEY_BACK,		/* 28:[  BACK  ][        ][        ][        ] */
	0,				/* 29:                                         */
	0,				/* 2A:                                         */
	0,				/* 2B:                                         */
	0,				/* 2C:                                         */
	0,				/* 2D:                                         */
	0,				/* 2E:                                         */
	0,				/* 2F:                                         */
	KEY_4,			/* 30:[   4    ][   $    ][        ][        ] */
	KEY_0,			/* 31:[   0    ][   ~    ][   ^    ][        ] */
	KEY_R,			/* 32:[   R    ][        ][        ][        ] */
	KEY_P,			/* 33:[   P    ][        ][   @    ][   `    ] */
	KEY_G,			/* 34:[   G    ][        ][        ][        ] */
	KEY_ENTER,		/* 35:[ Enter  ][        ][Private ][        ] */
	KEY_N,			/* 36:[   N    ][        ][        ][        ] */
	KEY_PICTURE,	/* 37:[Picture ][        ][        ][        ] */
	KEY_MENU,		/* 38:[  MENU  ][        ][        ][        ] */
	0,				/* 39:                                         */
	0,				/* 3A:                                         */
	0,				/* 3B:                                         */
	0,				/* 3C:                                         */
	0,				/* 3D:                                         */
	0,				/* 3E:                                         */
	0,				/* 3F:                                         */
	KEY_5,			/* 40:[   5    ][   %    ][        ][        ] */
	KEY_MINUS,		/* 41:[   -    ][   =    ][   \    ][   _    ] */
	KEY_T,			/* 42:[   T    ][        ][        ][        ] */
	KEY_SEARCH,		/* 43:[ SEARCH ][   `    ][        ][        ] */
	KEY_H,			/* 44:[   H    ][        ][        ][        ] */
	KEY_Z,			/* 45:[   Z    ][        ][        ][        ] */
	KEY_M,			/* 46:[   M    ][        ][        ][        ] */
	KEY_RIGHTSHIFT,	/* 47:[Shift R ][        ][        ][   _    ] */
	BTN_MOUSE,		/* 48:[   OK   ][        ][        ][        ] */
	0,				/* 49:                                         */
	0,				/* 4A:                                         */
	0,				/* 4B:                                         */
	0,				/* 4C:                                         */
	0,				/* 4D:                                         */
	0,				/* 4E:                                         */
	0,				/* 4F:                                         */
	KEY_6,			/* 50:[   6    ][   &    ][        ][        ] */
	KEY_BACKSPACE,	/* 51:[  DEL   ][        ][        ][        ] */
	KEY_Y,			/* 52:[   Y    ][        ][        ][        ] */
	KEY_A,			/* 53:[   A    ][        ][        ][        ] */
	KEY_J,			/* 54:[   J    ][        ][        ][        ] */
	KEY_X,			/* 55:[   X    ][        ][        ][        ] */
	KEY_COMMA,		/* 56:[   ,    ][   <    ][   [    ][    {   ] */
	KEY_HOME,		/* 57:[  HOME  ][        ][        ][        ] */
	0,				/* 58:                                         */
	0,				/* 59:                                         */
	0,				/* 5A:                                         */
	0,				/* 5B:                                         */
	0,				/* 5C:                                         */
	0,				/* 5D:                                         */
	0,				/* 5E:                                         */
	0,				/* 5F:                                         */
#ifdef ROTATE_90
	KEY_RIGHT,		/* 60:[  Up    ][        ][  Top   ][        ] */
	KEY_LEFT,		/* 61:[  Down  ][        ][ Bottom ][        ] */
#else
	KEY_UP,			/* 60:[  Up    ][        ][  Top   ][        ] */
	KEY_DOWN,		/* 61:[  Down  ][        ][ Bottom ][        ] */
#endif	/* ROTATE_90 */
	KEY_LEFTALT,	/* 62:[  Alt   ][        ][        ][        ] */
	0,				/* 63:                                         */
	0,				/* 64:                                         */
	0,				/* 65:                                         */
	0,				/* 66:                                         */
	0,				/* 67:                                         */
	0,				/* 68:                                         */
	0,				/* 69:                                         */
	0,				/* 6A:                                         */
	0,				/* 6B:                                         */
	0,				/* 6C:                                         */
	0,				/* 6D:                                         */
	0,				/* 6E:                                         */
	0,				/* 6F:                                         */
#ifdef ROTATE_90
	KEY_UP,			/* 70:[  Left  ][        ][  Home  ][        ] */
	KEY_DOWN,		/* 71:[  Right ][        ][  End   ][        ] */
#else
	KEY_LEFT,		/* 70:[  Left  ][        ][  Home  ][        ] */
	KEY_RIGHT,		/* 71:[  Right ][        ][  End   ][        ] */
#endif	/* ROTATE_90 */
	KEY_LEFTSHIFT,	/* 72:[Shift L ][        ][        ][        ] */
	0,				/* 73:                                         */
	0,				/* 74:                                         */
	0,				/* 75:                                         */
	0,				/* 76:                                         */
	0,				/* 77:                                         */
	0,				/* 78:                                         */
	0,				/* 79:                                         */
	0,				/* 7A:                                         */
	0,				/* 7B:                                         */
	0,				/* 7C:                                         */
	0,				/* 7D:                                         */
	0,				/* 7E:                                         */
	0,				/* 7F:                                         */
};

static const OctKey_I2cWriteData goInitData[] =
	{
		{	KPD_CLKEN,			KPD_CLKEN_KBDEN},
		{	KPD_KBDSETTLE,		0xA3},			/* 9.68ms */
		{	KPD_KBDBOUNCE,		0xA3},			/* 9.68ms */
		{	KPD_IOPC0_H,		0xAA},			/* KPX:Pull-up */
		{	KPD_IOPC0_L,		0xAA},
		{	KPD_IOPC1_H,		0xAA},			/* KPY:Pull-up */
		{	KPD_IOPC1_L,		0xAA},
		{	KPD_IOPC2_H,		0x5A},
		{	KPD_IOPC2_L,		0x5A},
		{	KPD_KBDSIZE,		0x8A},			/* 8*10 */
		{	KPD_KBDDEDCFG_H,	0xFF},
		{	KPD_KBDDEDCFG_L,	0xFF},
		{	KPD_KBDMSK,			KPD_KBDMSK_MSKELNT | KPD_KBDMSK_MSKEINT | KPD_KBDMSK_MSKKLI},
		{	0xFF,				0x00},			/* Terminal */
	};

/*+-------------------------------------------------------------------------+*/
/*|	PROTO TYPE DECLARE														|*/
/*+-------------------------------------------------------------------------+*/
/* I2C access */
static int OctKey_I2cRead(I2cClient *poClient, uint8_t bRegAdr, uint8_t *pbBuf, uint32_t dwLen);
static int OctKey_I2cWriteOne(I2cClient *poClient, uint8_t bRegAdr, uint8_t bData);
static int OctKey_I2cWriteAny(I2cClient *poClient, const OctKey_I2cWriteData *poData);

/* Key Driver */
static int __init OctKey_Init(void);
static void __exit OctKey_Exit(void);
static int __devinit OctKey_Probe(I2cClient *poClient, const I2cDevID *poDevId);
static int __devexit OctKey_Remove(I2cClient *poClient);
#ifdef CONFIG_PM
static int OctKey_Suspend(I2cClient *poClient, pm_message_t oMsg);
static int OctKey_Resume(I2cClient *poClient);
#else
#define	OctKey_Suspend		NULL
#define	OctKey_Resume		NULL
#endif	/* CONFIG_PM */
static void OctKey_Connect2InputSys(WorkStruct *poWork);
static InputDev *OctKey_CreateInputDev(I2cKbdRec *poKbdRec);
static int OctKey_ConfigGPIO(I2cKbdRec *poKbdRec);
static int OctKey_ReleaseGPIO(I2cKbdRec *poKbdRec);
static int OctKey_OpenCB(InputDev *poInDev);
static void OctKey_CloseCB(InputDev *pInDev);
static void OctKey_Shutdown(I2cKbdRec *poKbdRec);
static int OctKey_Start(I2cKbdRec *poKbdRec);
static void OctKey_Stop(I2cKbdRec *poKbdRec);
static irqreturn_t OctKey_IrqHandler(int nIrq, void *pvDevId);
static void OctKey_Recover(WorkStruct *poWork);
static void OctKey_NotifyKeyCode(I2cKbdRec *poKbdRec, InputDev *pInDev, uint8_t *pbCode);
static void OctKey_FetchKeys(WorkStruct *poWork);
static int OctKey_FetchKeysMain(I2cKbdRec *poKbdRec);

/*+-------------------------------------------------------------------------+*/
/*|	I2C ACCESS																|*/
/*+-------------------------------------------------------------------------+*/
/*+-------------------------------------------------------------------------+*/
/*|	I2C READ																|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_I2cRead(I2cClient *poClient, uint8_t bRegAdr, uint8_t *pbBuf, uint32_t dwLen)
{
	int nResult;
	int nI;
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= poClient->addr,
					.flags	= 0,
					.buf	= (void *)&bRegAdr,
					.len	= 1
				},
			[1] =
				{
					.addr	= poClient->addr,
					.flags	= I2C_M_RD,
					.buf	= (void *)pbBuf,
					.len	= dwLen
				}
		};

	for(nI = 0; nI < KPD_I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(poClient->adapter, oMsgs, 2);
		if(nResult > 0)
		{
			return 0;
		}
		printk(KERN_DEBUG "[OctKey]I2cRead %d(%02X,reg:%02X,Data:%02X,Len:%d)=%d\n", nI, poClient->addr, bRegAdr, pbBuf[0], dwLen, nResult);
	}
	return nResult;
}

/*+-------------------------------------------------------------------------+*/
/*|	I2C WRITE																|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_I2cWriteOne(I2cClient *poClient, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= poClient->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
};

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < KPD_I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(poClient->adapter, oMsgs, 1);
		if(nResult > 0)
		{
			return 0;
		}
		printk(KERN_DEBUG "[OctKey]I2cWrite %d(%02X,reg:%02X,Data:%02X)=%d\n", nI, poClient->addr, bRegAdr, bData, nResult);
	}
	return nResult;
}

static int OctKey_I2cWriteAny(I2cClient *poClient, const OctKey_I2cWriteData *poData)
{
	int nI;
	int nResult;

	/* Set */
	for(nI = 0; poData[nI].mbRegAdr != 0xFF; nI++)
	{
		nResult = OctKey_I2cWriteOne(poClient, poData[nI].mbRegAdr, poData[nI].mbData);
		if(nResult < 0)
		{
			return -1;
		}
	}
	return 0;
}

/*+-------------------------------------------------------------------------+*/
/*|	KEYBOARD DRIVER															|*/
/*+-------------------------------------------------------------------------+*/
module_init(OctKey_Init);
module_exit(OctKey_Exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("I2C QWERTY keyboard driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:SH_qwerty_key");

static const I2cDevID gI2cDevIdTable[] =
{
   { NB30_KBD_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTable);

/* Structure */
static struct i2c_driver goI2cKbdDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = NB30_KBD_I2C_DEVNAME,
	},
	.probe	  = OctKey_Probe,
	.remove	  = __devexit_p(OctKey_Remove),
	.suspend  = OctKey_Suspend,
	.resume   = OctKey_Resume,
	.id_table = gI2cDevIdTable,
};

static int __init OctKey_Init(void)
{
printk(KERN_DEBUG "[OctKey]Init(PID:%ld)\n", sys_getpid());
	/* I2C driver use start */
	return i2c_add_driver(&goI2cKbdDriver);
}

static void __exit OctKey_Exit(void)
{
printk(KERN_DEBUG "[OctKey]Exit(PID:%ld)\n", sys_getpid());
	/* I2C driver use stop */
	i2c_del_driver(&goI2cKbdDriver);
}

/*+-------------------------------------------------------------------------+*/
/*|	KEY DRIVER START														|*/
/*+-------------------------------------------------------------------------+*/
static int __devinit OctKey_Probe(I2cClient *poClient, const I2cDevID *poDevId)
{
	struct msm_sh_i2ckbd_platform_data *poSetupData;
	I2cKbdRec *poKbdRec = NULL;
	int nResult;

printk(KERN_DEBUG "[OctKey]Probe(PID:%ld)\n", sys_getpid());
	if(!poClient->dev.platform_data)
	{
		dev_err(&poClient->dev, "keyboard platform device data is required\n");
		return -ENODEV;
	}
	/* Memory securing */
	poKbdRec = kzalloc(sizeof(I2cKbdRec), GFP_KERNEL);
	if(!poKbdRec)
	{
		return -ENOMEM;
	}
	poClient->driver = &goI2cKbdDriver;
	i2c_set_clientdata(poClient, poKbdRec);
	poKbdRec->mpoClient			 = poClient;
	poSetupData					 = poClient->dev.platform_data;
	/* Get setup info (board-xxxx.c) */
	poKbdRec->mnIrqPin			 = poSetupData->gpio_kbdirq;
	poKbdRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poKbdRec->mpfPinShutdownFunc = poSetupData->gpio_shutdown;
	/* GPIO Setting */
	if(0 == (nResult = OctKey_ConfigGPIO(poKbdRec)))
	{
		INIT_WORK(&poKbdRec->moIrqWork, OctKey_FetchKeys);
		dev_info(&poClient->dev, "Detected %s, attempting to initialize keyboard\n", NB30_KBD_I2C_DEVNAME);
		snprintf(poKbdRec->mcPhysInfo, QPHYSLEN, "%s/%s/event0",
				 poClient->adapter->dev.bus_id, poClient->dev.bus_id);
		INIT_DELAYED_WORK(&poKbdRec->moCmdQ, OctKey_Connect2InputSys);
		schedule_delayed_work(&poKbdRec->moCmdQ, msecs_to_jiffies(INITDELAY_TIMER));
		device_init_wakeup(&poClient->dev, 1);
		return 0;
	}
	/* GPIO Liberating */
	OctKey_ReleaseGPIO(poKbdRec);
	kfree(poKbdRec);
	return nResult;
}

/*+-------------------------------------------------------------------------+*/
/*|	KEY DRIVER Remove														|*/
/*+-------------------------------------------------------------------------+*/
static int __devexit OctKey_Remove(I2cClient *poClient)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[OctKey]Remove(PID:%ld)\n", sys_getpid());
	dev_info(&poClient->dev, "removing keyboard driver\n");
	device_init_wakeup(&poClient->dev, 0);
	if(poKbdRec->mpoInDev)
	{
		dev_dbg(&poClient->dev, "deregister from input system\n");
		input_unregister_device(poKbdRec->mpoInDev);
		poKbdRec->mpoInDev = NULL;
	}
	OctKey_Shutdown(poKbdRec);
	OctKey_ReleaseGPIO(poKbdRec);
	kfree(poKbdRec);
	return 0;
}

#ifdef CONFIG_PM
/*+-------------------------------------------------------------------------+*/
/*|	KEY DRIVER SUSPEND														|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_Suspend(I2cClient *poClient, pm_message_t oMsg)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[OctKey]Suspend(PID:%ld)\n", sys_getpid());
	if(device_may_wakeup(&poClient->dev))
	{
		enable_irq_wake(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
	return 0;
}

/*+-------------------------------------------------------------------------+*/
/*|	KEY DRIVER RESUME														|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_Resume(I2cClient *poClient)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[OctKey]Resume(PID:%ld)\n", sys_getpid());
	if(device_may_wakeup(&poClient->dev))
	{
		disable_irq_wake(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
	return 0;
}
#endif	/* CONFIG_PM */

static void OctKey_Connect2InputSys(WorkStruct *poWork)
{
	I2cKbdRec *poKbdRec = container_of(poWork, I2cKbdRec, moCmdQ.work);
	Device *poDev = &poKbdRec->mpoClient->dev;

printk(KERN_DEBUG "[OctKey]Connect2InputSys(PID:%ld)\n", sys_getpid());
	poKbdRec->mpoInDev = OctKey_CreateInputDev(poKbdRec);
	if(poKbdRec->mpoInDev)
	{
		if(input_register_device(poKbdRec->mpoInDev) != 0)
		{
			dev_err(poDev, "Failed to register with input system\n");
			input_free_device(poKbdRec->mpoInDev);
		}
	}
}

static InputDev *OctKey_CreateInputDev(I2cKbdRec *poKbdRec)
{
	Device *poDev = &poKbdRec->mpoClient->dev;
	InputDev *pInDev = input_allocate_device();
	int nI;

printk(KERN_DEBUG "[OctKey]CreateInputDev(PID:%ld)\n", sys_getpid());
	if(pInDev)
	{
		pInDev->name = NB30_KBD_I2C_DEVNAME;
		pInDev->phys = poKbdRec->mcPhysInfo;
		pInDev->id.bustype = BUS_I2C;
		pInDev->id.vendor  = QCVENDOR_ID;
		pInDev->id.product = QCPRODUCT_ID;
		pInDev->id.version = QCVERSION_ID;
		pInDev->open = OctKey_OpenCB;
		pInDev->close = OctKey_CloseCB;
		__set_bit(EV_KEY, pInDev->evbit);
		for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
		{
			poKbdRec->mwPrev[nI] = KEY_RESERVED;
}
		for(nI = 0; nI < KPD_KEY_MAX; nI++)
		{
			if(gwOctKeyTable[nI] != 0)
{
				__set_bit(gwOctKeyTable[nI], pInDev->keybit);
			}
		}
		__set_bit(KEY_END, pInDev->keybit);
		__set_bit(KEY_POWER, pInDev->keybit);
		input_set_drvdata(pInDev, poKbdRec);
}
	else
{
		dev_err(poDev, "Failed to allocate input device for %s\n", NB30_KBD_I2C_DEVNAME);
	}
	return pInDev;
		}

/*+-------------------------------------------------------------------------+*/
/*|	GPIO SETTING															|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_ConfigGPIO(I2cKbdRec *poKbdRec)
{
	if(poKbdRec == NULL)
		return -EINVAL;
	return poKbdRec->mpfPinSetupFunc();
}

/*+-------------------------------------------------------------------------+*/
/*|	GPIO LIBERATING															|*/
/*+-------------------------------------------------------------------------+*/
static int OctKey_ReleaseGPIO(I2cKbdRec *poKbdRec)
{
	if(poKbdRec == NULL)
		return -EINVAL;
	/* GPIO Liberating */
	dev_info(&poKbdRec->mpoClient->dev, "releasing keyboard gpio pins %d\n", poKbdRec->mnIrqPin);
	poKbdRec->mpfPinShutdownFunc();
	return 0;
		}

static int OctKey_OpenCB(InputDev *poInDev)
{
	I2cKbdRec *poKbdRec = input_get_drvdata(poInDev);

printk(KERN_DEBUG "[OctKey]OpenCB(PID:%ld)\n", sys_getpid());
	dev_dbg(&poKbdRec->mpoClient->dev, "ENTRY: input_dev open callback\n");
	return OctKey_Start(poKbdRec);
}

static void OctKey_CloseCB(InputDev *pInDev)
{
	I2cKbdRec *poKbdRec = input_get_drvdata(pInDev);
	Device *poDev = &poKbdRec->mpoClient->dev;

printk(KERN_DEBUG "[OctKey]CloseCB(PID:%ld)\n", sys_getpid());
	dev_dbg(poDev, "ENTRY: close callback\n");
	OctKey_Shutdown(poKbdRec);
}

static void OctKey_Shutdown(I2cKbdRec *poKbdRec)
{
printk(KERN_DEBUG "[OctKey]Shutdown(PID:%ld)\n", sys_getpid());
	/* Active */
	if(poKbdRec->mbIsActive)
	{
		OctKey_Stop(poKbdRec);
		/* Work memory liberating */
		flush_work(&poKbdRec->moIrqWork);
		}
	}

static int OctKey_Start(I2cKbdRec *poKbdRec)
{
	I2cClient *poClient = poKbdRec->mpoClient;
	int nResult = 0;

printk(KERN_DEBUG "[OctKey]Start(PID:%ld)\n", sys_getpid());
	/* Inactive */
	poKbdRec->mbIsActive = 0;
	nResult = OctKey_I2cWriteAny(poClient, goInitData);
	if(nResult < 0)
	{
printk(KERN_DEBUG "[OctKey]Start-->Error\n");
		return -EIO;
}
	/* Set Irq */
	nResult = request_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin), &OctKey_IrqHandler,
						 IRQF_TRIGGER_LOW | IRQF_DISABLED,
					     NB30_KBD_I2C_DEVNAME, poKbdRec);
	if(nResult < 0)
{
		printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", NB30_KBD_I2C_DEVNAME, nResult);
		return -EIO;
	}
	/* Set Active */
	poKbdRec->mbIsActive = 1;
	return 0;
}

static void OctKey_Stop(I2cKbdRec *poKbdRec)
{
printk(KERN_DEBUG "[OctKey]Stop(PID:%ld)\n", sys_getpid());
	/* Inactive */
	poKbdRec->mbIsActive = 0;
	/* Irq release */
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin), poKbdRec);
}

/*+-------------------------------------------------------------------------+*/
/*|	IRQ HANDLER																|*/
/*+-------------------------------------------------------------------------+*/
static irqreturn_t OctKey_IrqHandler(int nIrq, void *pvDevId)
{
	I2cKbdRec *poKbdRec = pvDevId;

//printk(KERN_DEBUG "[OctKey]IrqHandler(PID:%ld,IRQ:%02X)\n", sys_getpid(), nIrq);
	disable_irq_nosync(nIrq);
	schedule_work(&poKbdRec->moIrqWork);
	return IRQ_HANDLED;
}

static void OctKey_Recover(WorkStruct *poWork)
{
	int nResult;
	I2cKbdRec *poKbdRec;

printk(KERN_DEBUG "[OctKey]Recover(PID:%ld)\n", sys_getpid());
	poKbdRec = container_of(poWork, I2cKbdRec, moCmdQ.work);

	dev_info(&poKbdRec->mpoClient->dev, "keyboard recovery requested\n");

	nResult = OctKey_Start(poKbdRec);
	if(nResult != 0)
	{
		dev_err(&poKbdRec->mpoClient->dev, "recovery failed with (nResult=%d)\n", nResult);
	}
}

static void OctKey_NotifyKeyCode(I2cKbdRec *poKbdRec, InputDev *pInDev, uint8_t *pbCode)
{
	long lCode;
	int nI;
	int nJ;
	int nCnt;
	int wCode[KPD_KEYBUFF_SIZE];

	/* Inactive */
	if(!poKbdRec->mbIsActive)
	{
		return;
}
	nCnt = 0;
	/* Loop */
	for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
	{
		/* key push */
		if(pbCode[nI] != KPD_INVALID_KEY)
		{
			wCode[nI] = gwOctKeyTable[pbCode[nI] & 0x7f];
			/* puch checked */
			for(nJ = 0; nJ < KPD_KEYBUFF_SIZE; nJ++)
			{
				if(poKbdRec->mwPrev[nJ] == wCode[nI])
{
					break;
	}
}
			/* New key */
			if(nJ == KPD_KEYBUFF_SIZE)
{
				lCode = (long)wCode[nI];
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[OctKey]Key Press  <%3ld>\n", lCode);
#endif	/* KBD_DEBPRN */
				input_report_key(pInDev, lCode, KPD_KEYPRESS);
				nCnt++;
}
			else
{
				/* deleted */
				poKbdRec->mwPrev[nJ] = KEY_RESERVED;
			}
		}
		else
		{
			wCode[nI] = KEY_RESERVED;
	}
	}
	/* Loop */
	for(nJ = 0; nJ < KPD_KEYBUFF_SIZE; nJ++)
	{
		/* judged key release */
		if(poKbdRec->mwPrev[nJ] != KEY_RESERVED)
		{
			lCode = (long)poKbdRec->mwPrev[nJ];
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[OctKey]Key Release<%3ld>\n", lCode);
#endif	/* KBD_DEBPRN */
			input_report_key(pInDev, lCode, KPD_KEYRELEASE);
			nCnt++;
		}
	}
	if(nCnt != 0)
	{
		input_sync(pInDev);
}
	/* Loop */
	for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
	{
		/* Pressing key record */
		poKbdRec->mwPrev[nI] = wCode[nI];
		}
	}

static void OctKey_FetchKeys(WorkStruct *poWork)
{
	I2cKbdRec *poKbdRec = container_of(poWork, I2cKbdRec, moIrqWork);

//printk(KERN_DEBUG "[OctKey]FetchKeys(PID:%ld)\n", sys_getpid());
	if(OctKey_FetchKeysMain(poKbdRec) != 0)
{
		dev_err(&poKbdRec->mpoClient->dev, "Failed read from keyboard \n");
		OctKey_Stop(poKbdRec);
		INIT_DELAYED_WORK(&poKbdRec->moCmdQ, OctKey_Recover);
		schedule_delayed_work(&poKbdRec->moCmdQ, msecs_to_jiffies(RECOVER_TIMER));
	}
		else
	{
		/* Irq Enable */
		enable_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
}

static int OctKey_FetchKeysMain(I2cKbdRec *poKbdRec)
{
	InputDev *pInDev = poKbdRec->mpoInDev;
	I2cClient *poClient = poKbdRec->mpoClient;
	uint8_t bIRQST;
	uint8_t bKBDMIS;
	uint8_t bKey[KPD_KEYBUFF_SIZE];
	uint8_t bEvt = 0x00;

	/* Read status */
	if(0 != OctKey_I2cRead(poClient, KPD_IRQST, &bIRQST, 1))
	{
printk(KERN_DEBUG "[OctKey]i2c read error\n");
		return -1;
	}
	if(bIRQST & KPD_IRQST_PORIRQ)
	{
//printk(KERN_DEBUG "[OctKey]IRQST %02X\n", bIRQST);
		/* Power-on reset */
		OctKey_I2cWriteOne(poClient, KPD_RSTINTCLR, KPD_RSTINTCLR_IRQCLR);
printk(KERN_DEBUG "[OctKey]KPD_RSTINTCLR(%02X)<<%02X\n", KPD_RSTINTCLR, KPD_RSTINTCLR_IRQCLR);
	}
	else if(bIRQST & KPD_IRQST_KBDIRQ)
	{
		/* Read status */
		if(0 != OctKey_I2cRead(poClient, KPD_KBDMIS, &bKBDMIS, 1))
		{
printk(KERN_DEBUG "[OctKey]i2c read error\n");
			return -1;
}
		if(bKBDMIS & KPD_KBDMIS_MELINT)
{
printk(KERN_DEBUG "[OctKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
			OctKey_I2cWriteOne(poClient, KPD_KBDIC, KPD_KBDIC_KBDIC);
}
		if(bKBDMIS & KPD_KBDMIS_MEVINT)
		{
			if(0 != OctKey_I2cRead(poClient, KPD_EVTCODE, &bEvt, 1))
{
printk(KERN_DEBUG "[OctKey]i2c read error\n");
				return -1;
			}
printk(KERN_DEBUG "[OctKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
	}
		if(bKBDMIS & KPD_KBDMIS_MKLINT)
		{
printk(KERN_DEBUG "[OctKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
			OctKey_I2cWriteOne(poClient, KPD_KBDIC, KPD_KBDIC_EVTIC);
}
		if(bKBDMIS & KPD_KBDMIS_MSINT)
		{
			/* Read key code */
			if(0 != OctKey_I2cRead(poClient, KPD_KEYCODE0, bKey, KPD_KEYBUFF_SIZE))
{
printk(KERN_DEBUG "[OctKey]i2c read error\n");
				return -1;
			}
			/* Notifies */
			OctKey_NotifyKeyCode(poKbdRec, pInDev, bKey);
}
	}
	else
{
printk(KERN_DEBUG "[OctKey]Other IRQ(%02X)\n", bIRQST);
		return -1;
	}
	return 0;
}

