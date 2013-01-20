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
 * This code is based on rpc_server_handset.c.
 * The original copyright and notice are described below.
*/

/* arch/arm/mach-msm/rpc_server_handset.c
 *
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/miscdevice.h>
#if 1
#include <linux/fs.h>
#include <asm/uaccess.h>
#endif

#include <asm/mach-types.h>

#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/rpc_server_handset.h>
#if 1
#include <linux/extdet_headset.h>

#endif

#if 1
#include <mach/msm_i2ckbd.h>
#if 0
#include <mach/msm_i2ctps.h>
#include <sharp/shterm_k.h>
#endif
#endif

#define DRIVER_NAME	"msm-handset"
#if 1
#define DRIVER_SH_PM_NAME	"SH_pm_key"
#define DRIVER_SH_HS_NAME	"SH_headset_key"
#define DRIVER_SH_MT_NAME	"SH_matrix_keypad"
#endif

#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x00010001

#define HS_RPC_PROG 0x30000091

#define HS_RPC_VERS_1 0x00010001
#define HS_RPC_VERS_2 0x00020001

#define HS_PROCESS_CMD_PROC 0x02
#define HS_SUBSCRIBE_SRVC_PROC 0x03
#define HS_REPORT_EVNT_PROC    0x05
#if 1
#define HS_SHEXTDET_API_INITIALIZE_REMOTE_PROC 0x05
#define HS_SHKEYPAD_API_INITIALIZE_REMOTE_PROC 0x06
#define HS_SHEXTDET_SW_DETECT_PROC			   0x07
#endif
#define HS_EVENT_CB_PROC	1
#define HS_EVENT_DATA_VER	1

#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

#define HS_PWR_K		0x6F	/* Power key */
#define HS_END_K		0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_K	0x82
#define HS_HEADSET_SWITCH_K	0x84
#if 1
#define HS_FLIP_K			0x88
#endif
#define HS_HEADSET_SWITCH_2_K	0xF0
#define HS_HEADSET_SWITCH_3_K	0xF1
#define HS_REL_K		0xFF	/* key release */
#if 1
#define HS_VOL_UP_K		0x8F	/* Volume Up Key */
#define HS_VOL_DOWN_K	0x90	/* Volume Down Key */
#define HS_SF1_K		0x93	/* SF1 Key */
#define HS_SF2_K		0x94	/* SF2 Key */
#ifdef CONFIG_LYNCIS_KEYPAD
#define HS_MENU_K       0x5B    /* MENU Key */
#define HS_HOME_K       0xB4    /* HOME Key */
#define HS_BACK_K       0xB5    /* BACK Key */
#define HS_SEARCH_K     0xB6    /* SEARCH Key */
#endif /* CONFIG_LYNCIS_KEYPAD */
#endif

#define KEY(hs_key, input_key) ((hs_key << 24) | input_key)

#define DEBUG 0

enum hs_event {
	HS_EVNT_EXT_PWR = 0,	/* External Power status        */
	HS_EVNT_HSD,		/* Headset Detection            */
	HS_EVNT_HSTD,		/* Headset Type Detection       */
	HS_EVNT_HSSD,		/* Headset Switch Detection     */
	HS_EVNT_KPD,
	HS_EVNT_FLIP,		/* Flip / Clamshell status (open/close) */
	HS_EVNT_CHARGER,	/* Battery is being charged or not */
	HS_EVNT_ENV,		/* Events from runtime environment like DEM */
	HS_EVNT_REM,		/* Events received from HS counterpart on a
				remote processor*/
	HS_EVNT_DIAG,		/* Diag Events  */
	HS_EVNT_LAST,		 /* Should always be the last event type */
	HS_EVNT_MAX		/* Force enum to be an 32-bit number */
};

enum hs_src_state {
	HS_SRC_STATE_UNKWN = 0,
	HS_SRC_STATE_LO,
	HS_SRC_STATE_HI,
};

struct hs_event_data {
	uint32_t	ver;		/* Version number */
	enum hs_event	event_type;     /* Event Type	*/
	enum hs_event	enum_disc;     /* discriminator */
	uint32_t	data_length;	/* length of the next field */
	enum hs_src_state	data;    /* Pointer to data */
	uint32_t	data_size;	/* Elements to be processed in data */
};

enum hs_return_value {
	HS_EKPDLOCKED     = -2,	/* Operation failed because keypad is locked */
	HS_ENOTSUPPORTED  = -1,	/* Functionality not supported */
	HS_FALSE          =  0, /* Inquired condition is not true */
	HS_FAILURE        =  0, /* Requested operation was not successful */
	HS_TRUE           =  1, /* Inquired condition is true */
	HS_SUCCESS        =  1, /* Requested operation was successful */
	HS_MAX_RETURN     =  0x7FFFFFFF/* Force enum to be a 32 bit number */
};

struct hs_key_data {
	uint32_t ver;        /* Version number to track sturcture changes */
	uint32_t code;       /* which key? */
	uint32_t parm;       /* key status. Up/down or pressed/released */
};

enum hs_subs_srvc {
	HS_SUBS_SEND_CMD = 0, /* Subscribe to send commands to HS */
	HS_SUBS_RCV_EVNT,     /* Subscribe to receive Events from HS */
	HS_SUBS_SRVC_MAX
};

enum hs_subs_req {
	HS_SUBS_REGISTER,    /* Subscribe   */
	HS_SUBS_CANCEL,      /* Unsubscribe */
	HS_SUB_STATUS_MAX
};

enum hs_event_class {
	HS_EVNT_CLASS_ALL = 0, /* All HS events */
	HS_EVNT_CLASS_LAST,    /* Should always be the last class type   */
	HS_EVNT_CLASS_MAX
};

enum hs_cmd_class {
	HS_CMD_CLASS_LCD = 0, /* Send LCD related commands              */
	HS_CMD_CLASS_KPD,     /* Send KPD related commands              */
	HS_CMD_CLASS_LAST,    /* Should always be the last class type   */
	HS_CMD_CLASS_MAX
};

/*
 * Receive events or send command
 */
union hs_subs_class {
	enum hs_event_class	evnt;
	enum hs_cmd_class	cmd;
};

struct hs_subs {
	uint32_t                ver;
	enum hs_subs_srvc	srvc;  /* commands or events */
	enum hs_subs_req	req;   /* subscribe or unsubscribe  */
	uint32_t		host_os;
	enum hs_subs_req	disc;  /* discriminator    */
	union hs_subs_class      id;
};

struct hs_event_cb_recv {
	uint32_t cb_id;
	uint32_t hs_key_data_ptr;
	struct hs_key_data key;
};
enum hs_ext_cmd_type {
	HS_EXT_CMD_KPD_SEND_KEY = 0, /* Send Key */
	HS_EXT_CMD_KPD_BKLT_CTRL, /* Keypad backlight intensity	*/
	HS_EXT_CMD_LCD_BKLT_CTRL, /* LCD Backlight intensity */
	HS_EXT_CMD_DIAG_KEYMAP, /* Emulating a Diag key sequence */
	HS_EXT_CMD_DIAG_LOCK, /* Device Lock/Unlock */
	HS_EXT_CMD_GET_EVNT_STATUS, /* Get the status for one of the drivers */
	HS_EXT_CMD_KPD_GET_KEYS_STATUS,/* Get a list of keys status */
	HS_EXT_CMD_KPD_SET_PWR_KEY_RST_THOLD, /* PWR Key HW Reset duration */
	HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD, /* Set pwr key threshold duration */
	HS_EXT_CMD_LAST, /* Should always be the last command type */
	HS_EXT_CMD_MAX = 0x7FFFFFFF /* Force enum to be an 32-bit number */
};

struct hs_cmd_data_type {
	uint32_t hs_cmd_data_type_ptr; /* hs_cmd_data_type ptr length */
	uint32_t ver; /* version */
	enum hs_ext_cmd_type id; /* command id */
	uint32_t handle; /* handle returned from subscribe proc */
	enum hs_ext_cmd_type disc_id1; /* discriminator id */
	uint32_t input_ptr; /* input ptr length */
	uint32_t input_val; /* command specific data */
	uint32_t input_len; /* length of command input */
	enum hs_ext_cmd_type disc_id2; /* discriminator id */
	uint32_t output_len; /* length of output data */
	uint32_t delayed; /* execution context for modem
				true - caller context
				false - hs task context*/
};

static const uint32_t hs_key_map[] = {
	KEY(HS_PWR_K, KEY_POWER),
	KEY(HS_END_K, KEY_END),
	KEY(HS_STEREO_HEADSET_K, SW_HEADPHONE_INSERT),
	KEY(HS_HEADSET_SWITCH_K, KEY_MEDIA),
	KEY(HS_HEADSET_SWITCH_2_K, KEY_VOLUMEUP),
	KEY(HS_HEADSET_SWITCH_3_K, KEY_VOLUMEDOWN),
#if 1
	KEY(HS_FLIP_K, SW_LID),
	KEY(HS_VOL_UP_K, KEY_VOLUMEUP),		/* KEYCODE_VOLUME_UP */
	KEY(HS_VOL_DOWN_K, KEY_VOLUMEDOWN),	/* KEYCODE_VOLUME_DOWN */
	KEY(HS_SF1_K, KEY_CAMERA),			/* KEYCODE_CAMERA */
	KEY(HS_SF2_K, KEY_FOCUS),			/* KEYCODE_FOCUS */
#ifdef CONFIG_LYNCIS_KEYPAD
    KEY(HS_MENU_K, KEY_MENU),           /* KEYCODE_MENU */
    KEY(HS_HOME_K, KEY_HOME),           /* KEYCODE_HOME */
    KEY(HS_BACK_K, KEY_BACK),           /* KEYCODE_BACK */
    KEY(HS_SEARCH_K, KEY_SEARCH),       /* KEYCODE_SEARCH */
#endif /* CONFIG_LYNCIS_KEYPAD */
#endif
	0
};

#if 0
enum {
	NO_DEVICE	= 0,
	MSM_HEADSET	= 1,
};
#else
enum {
	NO_DEVICE			= 0x00,
	MSM_HEADSET_STE		= 0x01,
	MSM_HEADSET_MONO	= 0x02,
	MSM_HEADSET_OTHER	= 0x03,
};
#endif

#if 1
enum {
	MSM_OPEN	= 0x00,
	MSM_CLOSE	= 0x01,
	MSM_OPEN_CHATT = 0x10,
	MSM_CLOSE_CHATT = 0x11,
};
enum {
	MSM_HSSW_RELEASE	= 0x00,
	MSM_HSSW_PRESS		= 0x01,
};
enum {
	HEADPHONE_SW_DETECT_OFF	 = 0,
	HEADPHONE_SW_DETECT_ON	 = 1,
};
#endif

/* Add newer versions at the top of array */
static const unsigned int rpc_vers[] = {
	0x00030001,
	0x00020001,
	0x00010001,
};
/* hs subscription request parameters */
struct hs_subs_rpc_req {
	uint32_t hs_subs_ptr;
	struct hs_subs hs_subs;
	uint32_t hs_cb_id;
	uint32_t hs_handle_ptr;
	uint32_t hs_handle_data;
};

static struct hs_subs_rpc_req *hs_subs_req;

struct msm_handset {
	struct input_dev *ipdev;
	struct switch_dev sdev;
#if 1
	struct switch_dev sdev_hssw;
#endif
	struct msm_handset_platform_data *hs_pdata;
};

static struct msm_rpc_client *rpc_client;
static struct msm_handset *hs;

#if 1
static struct msm_handset *hssw;
static struct msm_handset *pm_key;
static struct msm_handset *mt_key;

static uint8_t Headset_Status = NO_DEVICE;
#endif

#if 1
static struct msm_rpc_endpoint* rpc_svc_p;
#endif

static int hs_find_key(uint32_t hscode)
{
	int i, key;

	key = KEY(hscode, 0);

	for (i = 0; hs_key_map[i] != 0; i++) {
		if ((hs_key_map[i] & 0xff000000) == key)
			return hs_key_map[i] & 0x00ffffff;
	}
	return -1;
}

static void
report_headset_switch(struct input_dev *dev, int key, int value)
{
	struct msm_handset *hs = input_get_drvdata(dev);
#if 0
	input_report_switch(dev, key, value);
	switch_set_state(&hs->sdev, value);
#else
	switch(value)
	{
		case MSM_HEADSET_STE:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] key = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
#endif
			input_report_switch(dev, key, !!value);
			break;
		case NO_DEVICE:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] key = %d, !!value = %d, value = %d\n", __func__, key, !!value, value);
#endif
			input_report_switch(dev, key, !!value);
			break;
		default:
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] non Value [%d]\n", __func__, value);
#endif
			break;
	}
	Headset_Status = value;
	switch_set_state(&hs->sdev, !!value);
#endif
}

#if 1
static void
report_headphone_switch(struct input_dev *dev, int key, int value)
{
#if 0
	struct msm_handset *hs = input_get_drvdata(dev);
#if DEBUG
	printk(KERN_INFO "[msm-handset] [%s] sdev_hssw value = 0x%02x\n", __func__, value);
#endif
	switch_set_state(&hs->sdev_hssw, value);
	input_report_key(dev, key, value);
#else
	struct msm_handset *hssw = input_get_drvdata(dev);
#if DEBUG
	printk(KERN_INFO "[msm-handset] [%s] sdev_hssw value = 0x%02x\n", __func__, value);
#endif
	switch_set_state(&hssw->sdev_hssw, value);
	input_report_key(dev, key, value);
#endif
}
#endif

/*
 * tuple format: (key_code, key_param)
 *
 * old-architecture:
 * key-press = (key_code, 0)
 * key-release = (0xff, key_code)
 *
 * new-architecutre:
 * key-press = (key_code, 0)
 * key-release = (key_code, 0xff)
 */
static void report_hs_key(uint32_t key_code, uint32_t key_parm)
{
	int key, temp_key_code;

	if (key_code == HS_REL_K)
		key = hs_find_key(key_parm);
	else
		key = hs_find_key(key_code);

	temp_key_code = key_code;

	if (key_parm == HS_REL_K)
		key_code = key_parm;

	switch (key) {
	case KEY_POWER:
	case KEY_END:
#if 0
	case KEY_MEDIA:
	case KEY_VOLUMEUP:
	case KEY_VOLUMEDOWN:
#endif
#if 0
		input_report_key(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(pm_key)
		{
			input_report_key(pm_key->ipdev, key, (key_code != HS_REL_K));
			input_sync(pm_key->ipdev);
		}
#endif
		break;
#if 1
	case KEY_MEDIA:
#if 0
		report_headphone_switch(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(hssw)
		{
			report_headphone_switch(hssw->ipdev, key, (key_code != HS_REL_K));
			input_sync(hssw->ipdev);
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] hssw = NULL\n", __func__);
#endif
		}
#endif
		break;
#endif
	case SW_HEADPHONE_INSERT:
#if 0
		report_headset_switch(hs->ipdev, key, (key_code != HS_REL_K));
#else
		if(hs)
		{
			report_headset_switch(hs->ipdev, key, key_parm);
#endif
#if 1
			input_sync(hs->ipdev);
#endif
		}
		else
		{
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] hs = NULL\n", __func__);
#endif
		}
		break;
#if 1
	case SW_LID:
		break;
#endif
#if 1
	case KEY_VOLUMEUP:
	case KEY_VOLUMEDOWN:
	case KEY_CAMERA:
	case KEY_FOCUS:
#ifdef CONFIG_LYNCIS_KEYPAD
    case KEY_MENU:
    case KEY_HOME:
    case KEY_BACK:
    case KEY_SEARCH:
#endif /* CONFIG_LYNCIS_KEYPAD */
		if(mt_key)
		{
			input_report_key(mt_key->ipdev, key, (key_code != HS_REL_K));
			input_sync(mt_key->ipdev);
		}
		else
		{
			printk(KERN_INFO "[msm-handset] [%s] mt_key = NULL\n", __func__);
		}
		break;
#endif
	case -1:
		printk(KERN_ERR "%s: No mapping for remote handset event %d\n",
				 __func__, temp_key_code);
		return;
	}
#if 0
	input_sync(hs->ipdev);
#endif
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code, args->key_parm);

		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int process_subs_srvc_callback(struct hs_event_cb_recv *recv)
{
	if (!recv)
		return -ENODATA;

	report_hs_key(be32_to_cpu(recv->key.code), be32_to_cpu(recv->key.parm));

	return 0;
}

static void process_hs_rpc_request(uint32_t proc, void *data)
{
	if (proc == HS_EVENT_CB_PROC)
		process_subs_srvc_callback(data);
	else
		pr_err("%s: unknown rpc proc %d\n", __func__, proc);
}

#if 0
static int hs_rpc_report_event_arg(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	struct hs_event_rpc_req {
		uint32_t hs_event_data_ptr;
		struct hs_event_data data;
	};

	struct hs_event_rpc_req *req = buffer;

	req->hs_event_data_ptr	= cpu_to_be32(0x1);
	req->data.ver		= cpu_to_be32(HS_EVENT_DATA_VER);
	req->data.event_type	= cpu_to_be32(HS_EVNT_HSD);
	req->data.enum_disc	= cpu_to_be32(HS_EVNT_HSD);
	req->data.data_length	= cpu_to_be32(0x1);
	req->data.data		= cpu_to_be32(*(enum hs_src_state *)data);
	req->data.data_size	= cpu_to_be32(sizeof(enum hs_src_state));

	return sizeof(*req);
}

static int hs_rpc_report_event_res(struct msm_rpc_client *client,
					void *buffer, void *data)
{
	enum hs_return_value result;

	result = be32_to_cpu(*(enum hs_return_value *)buffer);
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	if (result == HS_SUCCESS)
		return 0;

	return 1;
}
#endif

void report_headset_status(bool connected)
{
#if 0
	int rc = -1;
	enum hs_src_state status;

	if (connected == true)
		status = HS_SRC_STATE_HI;
	else
		status = HS_SRC_STATE_LO;

	rc = msm_rpc_client_req(rpc_client, HS_REPORT_EVNT_PROC,
				hs_rpc_report_event_arg, &status,
				hs_rpc_report_event_res, NULL, -1);

	if (rc)
		pr_err("%s: couldn't send rpc client request\n", __func__);
#endif
}
EXPORT_SYMBOL(report_headset_status);

#if 0
static int hs_rpc_pwr_cmd_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	struct hs_cmd_data_type *hs_pwr_cmd = buffer;

	hs_pwr_cmd->hs_cmd_data_type_ptr = cpu_to_be32(0x01);

	hs_pwr_cmd->ver = cpu_to_be32(0x03);
	hs_pwr_cmd->id = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->handle = cpu_to_be32(hs_subs_req->hs_handle_data);
	hs_pwr_cmd->disc_id1 = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->input_ptr = cpu_to_be32(0x01);
	hs_pwr_cmd->input_val = cpu_to_be32(hs->hs_pdata->pwr_key_delay_ms);
	hs_pwr_cmd->input_len = cpu_to_be32(0x01);
	hs_pwr_cmd->disc_id2 = cpu_to_be32(HS_EXT_CMD_KPD_SET_PWR_KEY_THOLD);
	hs_pwr_cmd->output_len = cpu_to_be32(0x00);
	hs_pwr_cmd->delayed = cpu_to_be32(0x00);

	return sizeof(*hs_pwr_cmd);
}

static int hs_rpc_pwr_cmd_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}
#endif

static int hs_rpc_register_subs_arg(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	hs_subs_req = buffer;

	hs_subs_req->hs_subs_ptr	= cpu_to_be32(0x1);
	hs_subs_req->hs_subs.ver	= cpu_to_be32(0x1);
	hs_subs_req->hs_subs.srvc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	hs_subs_req->hs_subs.req	= cpu_to_be32(HS_SUBS_REGISTER);
	hs_subs_req->hs_subs.host_os	= cpu_to_be32(0x4); /* linux */
	hs_subs_req->hs_subs.disc	= cpu_to_be32(HS_SUBS_RCV_EVNT);
	hs_subs_req->hs_subs.id.evnt	= cpu_to_be32(HS_EVNT_CLASS_ALL);

	hs_subs_req->hs_cb_id		= cpu_to_be32(0x1);

	hs_subs_req->hs_handle_ptr	= cpu_to_be32(0x1);
	hs_subs_req->hs_handle_data	= cpu_to_be32(0x0);

	return sizeof(*hs_subs_req);
}

static int hs_rpc_register_subs_res(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	uint32_t result;

	result = be32_to_cpu(*((uint32_t *)buffer));
	pr_debug("%s: request completed: 0x%x\n", __func__, result);

	return 0;
}

static int hs_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int rc = -1;

	struct rpc_request_hdr *hdr = buffer;

	hdr->type = be32_to_cpu(hdr->type);
	hdr->xid = be32_to_cpu(hdr->xid);
	hdr->rpc_vers = be32_to_cpu(hdr->rpc_vers);
	hdr->prog = be32_to_cpu(hdr->prog);
	hdr->vers = be32_to_cpu(hdr->vers);
	hdr->procedure = be32_to_cpu(hdr->procedure);

	process_hs_rpc_request(hdr->procedure,
			    (void *) (hdr + 1));

	msm_rpc_start_accepted_reply(client, hdr->xid,
				     RPC_ACCEPTSTAT_SUCCESS);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static int __init hs_rpc_cb_init(void)
{
	int rc = 0;

	/* version 2 is used in 7x30 */
	rpc_client = msm_rpc_register_client("hs",
			HS_RPC_PROG, HS_RPC_VERS_2, 0, hs_cb_func);

	if (IS_ERR(rpc_client)) {
		pr_err("%s: couldn't open rpc client with version 2 err %ld\n",
			 __func__, PTR_ERR(rpc_client));
		/*version 1 is used in 7x27, 8x50 */
		rpc_client = msm_rpc_register_client("hs",
			HS_RPC_PROG, HS_RPC_VERS_1, 0, hs_cb_func);
	}

	if (IS_ERR(rpc_client)) {
		pr_err("%s: couldn't open rpc client with version 1 err %ld\n",
			 __func__, PTR_ERR(rpc_client));
		return PTR_ERR(rpc_client);
	}
	rc = msm_rpc_client_req(rpc_client, HS_SUBSCRIBE_SRVC_PROC,
				hs_rpc_register_subs_arg, NULL,
				hs_rpc_register_subs_res, NULL, -1);
	if (rc) {
		pr_err("%s: couldn't send rpc client request\n", __func__);
		msm_rpc_unregister_client(rpc_client);
	}

	return rc;
}

static int __devinit hs_rpc_init(void)
{
	int rc;

	rc = hs_rpc_cb_init();
	if (rc)
		pr_err("%s: failed to initialize rpc client\n", __func__);
#if 1
	rpc_svc_p = msm_rpc_connect_compatible(HS_RPC_PROG,HS_RPC_VERS_1,0);

	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
		rpc_svc_p = msm_rpc_connect_compatible(HS_RPC_PROG,HS_RPC_VERS_2,0);
	}
	
	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
		return PTR_ERR(rpc_svc_p);
	}
#endif
	rc = msm_rpc_create_server(&hs_rpc_server);
	if (rc)
		pr_err("%s: failed to create rpc server\n", __func__);

	return rc;
}

static void __devexit hs_rpc_deinit(void)
{
	if (rpc_client)
		msm_rpc_unregister_client(rpc_client);
}

static ssize_t msm_headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs->sdev)) {
#if 0
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET:
		return sprintf(buf, "Headset\n");
#else
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case MSM_HEADSET_MONO:
		return sprintf(buf, "MonoHeadset\n");
	case MSM_HEADSET_OTHER:
	case MSM_HEADSET_STE:
		return sprintf(buf, "Headset\n");
#endif
	}
	return -EINVAL;
}

#if 1
static ssize_t msm_hssw_print_name(struct switch_dev *sdev, char *buf)
{
#if 0
	switch (switch_get_state(&hs->sdev_hssw)) {
	case MSM_HSSW_RELEASE:
		return sprintf(buf, "RELEASE\n");
	case MSM_HSSW_PRESS:
		return sprintf(buf, "PRESS\n");
	}
	return -EINVAL;
#else
	switch (switch_get_state(&hssw->sdev_hssw)) {
	case MSM_HSSW_RELEASE:
		return sprintf(buf, "RELEASE\n");
	case MSM_HSSW_PRESS:
		return sprintf(buf, "PRESS\n");
	}
	return -EINVAL;
#endif
}
#endif

static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct input_dev *ipdev;
	
#if 1
	struct api_remote_req_t1 {
		struct rpc_request_hdr hdr;
	} send_p;
#endif

	hs = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	hs->sdev.name	= "h2w";
	hs->sdev.print_name = msm_headset_print_name;

	rc = switch_dev_register(&hs->sdev);
	if (rc)
		goto err_switch_dev_register;

#if 0	
	hs->sdev_hssw.name	= "headphone_switch";
	hs->sdev_hssw.print_name = msm_hssw_print_name;
	
	rc = switch_dev_register(&hs->sdev_hssw);
	if (rc)
		goto err_switch_dev_register;
#endif

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hs);

	hs->ipdev = ipdev;

	if (pdev->dev.platform_data)
		hs->hs_pdata = pdev->dev.platform_data;

	if (hs->hs_pdata->hs_name)
		ipdev->name = hs->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

#if 0
	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ipdev, EV_KEY, KEY_POWER);
	input_set_capability(ipdev, EV_KEY, KEY_END);
#else
	input_set_capability(ipdev, EV_SW, SW_HEADPHONE_INSERT);
#endif

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);

	rc = hs_rpc_init();
	if (rc) {
		dev_err(&ipdev->dev, "rpc init failure\n");
		goto err_hs_rpc_init;
	}

#if 1
	rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHEXTDET_API_INITIALIZE_REMOTE_PROC,
							&send_p,sizeof(send_p),
							NULL,0,
							5 * HZ);
	if(rc)
		goto err_hs_rpc_init;
#endif

	return 0;

err_hs_rpc_init:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hs->sdev);
#if 0
	switch_dev_unregister(&hs->sdev_hssw);
#endif
err_switch_dev_register:
	kfree(hs);
	hs = NULL;
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hs = platform_get_drvdata(pdev);

	input_unregister_device(hs->ipdev);
	switch_dev_unregister(&hs->sdev);
#if 0
	switch_dev_unregister(&hs->sdev_hssw);
#endif
	kfree(hs);
	hs_rpc_deinit();
	return 0;
}

#if 1
static int __devinit sh_pm_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;
	
	pm_key = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!pm_key)
		return -ENOMEM;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, pm_key);

	pm_key->ipdev = ipdev;

	if (pdev->dev.platform_data)
		pm_key->hs_pdata = pdev->dev.platform_data;

	if (pm_key->hs_pdata->hs_name)
		ipdev->name = pm_key->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_SH_PM_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_POWER);
	input_set_capability(ipdev, EV_KEY, KEY_END);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, pm_key);

	return 0;

err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	kfree(pm_key);
	pm_key = NULL;
	return rc;
}

static int __devexit sh_pm_remove(struct platform_device *pdev)
{
	struct msm_handset *pm_key_data = platform_get_drvdata(pdev);

	input_unregister_device(pm_key_data->ipdev);

	kfree(pm_key_data);
	return 0;
}

static int __devinit sh_hs_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;
	
	hssw = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!hssw)
		return -ENOMEM;

	hssw->sdev_hssw.name	= "headphone_switch";
	hssw->sdev_hssw.print_name = msm_hssw_print_name;
	
	rc = switch_dev_register(&hssw->sdev_hssw);
	if (rc)
		goto err_switch_dev_register;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hssw);

	hssw->ipdev = ipdev;

	if (pdev->dev.platform_data)
		hssw->hs_pdata = pdev->dev.platform_data;

	if (hssw->hs_pdata->hs_name)
		ipdev->name = hssw->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_SH_HS_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_MEDIA);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hssw);

	return 0;

err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	switch_dev_unregister(&hssw->sdev_hssw);
err_switch_dev_register:
	kfree(hssw);
	hssw = NULL;
	return rc;
}

static int __devexit sh_hs_remove(struct platform_device *pdev)
{
	struct msm_handset *hssw_data = platform_get_drvdata(pdev);

	input_unregister_device(hssw_data->ipdev);
	switch_dev_unregister(&hssw_data->sdev_hssw);

	kfree(hssw_data);
	return 0;
}
static int __devinit sh_mt_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;

	struct api_remote_req_t1 {
		struct rpc_request_hdr hdr;
	} send_p;

	mt_key = kzalloc(sizeof(struct msm_handset), GFP_KERNEL);
	if (!mt_key)
		return -ENOMEM;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, mt_key);

	mt_key->ipdev = ipdev;

	if (pdev->dev.platform_data)
		mt_key->hs_pdata = pdev->dev.platform_data;

	if (mt_key->hs_pdata->hs_name)
		ipdev->name = mt_key->hs_pdata->hs_name;
	else
		ipdev->name	= DRIVER_SH_MT_NAME;

	ipdev->id.vendor	= 0x0001;
	ipdev->id.product	= 1;
	ipdev->id.version	= 1;

	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ipdev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ipdev, EV_KEY, KEY_CAMERA);
	input_set_capability(ipdev, EV_KEY, KEY_FOCUS);
#ifdef CONFIG_LYNCIS_KEYPAD
    input_set_capability(ipdev, EV_KEY, KEY_MENU);
    input_set_capability(ipdev, EV_KEY, KEY_HOME);
    input_set_capability(ipdev, EV_KEY, KEY_BACK);
    input_set_capability(ipdev, EV_KEY, KEY_SEARCH);
#endif /* CONFIG_LYNCIS_KEYPAD */

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, mt_key);

	if(IS_ERR(rpc_svc_p)) {
		pr_err("%s: couldn't connect compatible rpc client err %ld\n", __func__,
			 PTR_ERR(rpc_svc_p));
		goto err_mt_rpc_init;
	}

	rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHKEYPAD_API_INITIALIZE_REMOTE_PROC,
							&send_p,sizeof(send_p),
							NULL,0,
							5 * HZ);
	if(rc)
		goto err_mt_rpc_init;

	return 0;

err_mt_rpc_init:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	kfree(mt_key);
	mt_key = NULL;
	return rc;
}

static int __devexit sh_mt_remove(struct platform_device *pdev)
{
	struct msm_handset *mt_key_data = platform_get_drvdata(pdev);

	input_unregister_device(mt_key_data->ipdev);

	kfree(mt_key_data);
	return 0;
}
#endif

static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};
#if 1
static struct platform_driver sh_pm_driver = {
	.probe		= sh_pm_probe,
	.remove		= __devexit_p(sh_pm_remove),
	.driver		= {
		.name	= DRIVER_SH_PM_NAME,
		.owner	= THIS_MODULE,
	},
};
static struct platform_driver sh_hs_driver = {
	.probe		= sh_hs_probe,
	.remove		= __devexit_p(sh_hs_remove),
	.driver		= {
		.name	= DRIVER_SH_HS_NAME,
		.owner	= THIS_MODULE,
	},
};
static struct platform_driver sh_mt_driver = {
	.probe		= sh_mt_probe,
	.remove		= __devexit_p(sh_mt_remove),
	.driver		= {
		.name	= DRIVER_SH_MT_NAME,
		.owner	= THIS_MODULE,
	},
};
#endif

#if 1
static int headset_sw_open(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int headset_sw_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return 0;
}

static int headset_sw_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc;
	struct headset_sw_rpc_start_t {
		struct rpc_request_hdr hdr;
		unsigned long  sw_detect;
	} send_p;
	
	struct headset_sw_rpc_ret_t {
		struct rpc_reply_hdr hdr;
		long ret;
	} recv_p;
	long result = 0;
	
	if((!hssw) || (!hs))
	{
		return -3;
	}
	
	switch (cmd)
	{
		case HEADPHONE_SW_IOCTL_DETECT_START:
			send_p.sw_detect = cpu_to_be32(HEADPHONE_SW_DETECT_ON);
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] sw_detect = 0x%02lx\n", __func__, send_p.sw_detect);
#endif
			break;
		case HEADPHONE_SW_IOCTL_DETECT_STOP:
			send_p.sw_detect = cpu_to_be32(HEADPHONE_SW_DETECT_OFF);
#if DEBUG
			printk(KERN_INFO "[msm-handset] [%s] sw_detect = 0x%02lx\n", __func__, send_p.sw_detect);
#endif
			break;
		default:
			return -2;
			break;
	}
	
	rc = msm_rpc_call_reply(rpc_svc_p,
							HS_SHEXTDET_SW_DETECT_PROC,
							&send_p, sizeof(send_p),
							&recv_p, sizeof(recv_p),
							5 * HZ);
	if (rc < 0) {
#if DEBUG
		printk(KERN_ERR "%s: rpc call failed! error: (%d)", __func__, rc);
#endif
		return -3;
	}
	else
	{
		result = cpu_to_be32(recv_p.ret);
#if DEBUG
		printk(KERN_INFO "[msm-handset] [%s] result = %ld\n", __func__, result);
#endif
		if(result != 0)
		{
			printk(KERN_ERR "[msm-handset] [%s] RPC_SW_DETECT_PROC NG\n", __func__);
			return -1;
		}
	}
	
	return 0;
}

static int headset_diag_open(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int headset_diag_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return 0;
}

static ssize_t headset_diag_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	char data[4];
	memset(data, 0x00, sizeof(data));
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	sprintf(data, "%d\n",Headset_Status);

	if(copy_to_user(buf, &data, sizeof(&data)))
	{
		printk(KERN_ERR "%s: copy_to_user Error\n", __func__);
		return -EFAULT;
	}
	return strlen(data);
}

static struct file_operations headset_sw_fops = {
	.owner = THIS_MODULE,
	.open    = headset_sw_open,
	.release = headset_sw_release,
	.ioctl    = headset_sw_ioctl,
};

static struct miscdevice headset_sw_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "headphone_sw",
	.fops = &headset_sw_fops,
};

static struct file_operations headset_diag_fops = {
	.owner = THIS_MODULE,
	.open    = headset_diag_open,
	.release = headset_diag_release,
	.read    = headset_diag_read,
};

static struct miscdevice headset_diag_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "headset_diag",
	.fops = &headset_diag_fops,
};
#endif

static int __init hs_init(void)
{
	int err = -ENODEV;

#if 0
	return platform_driver_register(&hs_driver);
#else
	int rc;
	if(0 != (rc = platform_driver_register(&sh_pm_driver))) {
		return rc;
	}
	if(0 != (rc = platform_driver_register(&sh_hs_driver))) {
		platform_driver_unregister(&sh_pm_driver);
		return rc;
	}
	if(0 != (rc = platform_driver_register(&hs_driver))) {
		platform_driver_unregister(&sh_pm_driver);
		platform_driver_unregister(&sh_hs_driver);
		return rc;
	}
	if(0 != (rc = platform_driver_register(&sh_mt_driver))) {
		platform_driver_unregister(&sh_pm_driver);
		platform_driver_unregister(&sh_hs_driver);
		platform_driver_unregister(&hs_driver);
		return rc;
	}
	err = misc_register(&headset_diag_device);
	if (err) {
		misc_deregister(&headset_diag_device);
		printk(KERN_ERR
		       "headset_diag_device: register failed\n");
	}
	err = misc_register(&headset_sw_device);
	if (err) {
		misc_deregister(&headset_sw_device);
		printk(KERN_ERR
		       "headset_sw_device: register failed\n");
	}
	
	return rc;
#endif
}
late_initcall(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
#if 1
	platform_driver_unregister(&sh_pm_driver);
	platform_driver_unregister(&sh_hs_driver);
	platform_driver_unregister(&sh_mt_driver);
	misc_deregister(&headset_diag_device);
	misc_deregister(&headset_sw_device);
#endif
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-handset");
