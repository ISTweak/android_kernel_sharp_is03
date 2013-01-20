/* include/sharp/shterm_k.h
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

#ifndef _SHTERM_K_H_
#define _SHTERM_K_H_

#define SHTERM_SUCCESS 0
#define SHTERM_FAILURE 1

#define SHTERM_FLIP_STATE_OPEN 0
#define SHTERM_FLIP_STATE_CLOSE 1

enum {
    SHTERM_INFO_SPEAKER = 0,
    SHTERM_INFO_VIB,
    SHTERM_INFO_CAMERA,
    SHTERM_INFO_LINE,
    SHTERM_INFO_QTV,
    SHTERM_INFO_DTB,
    SHTERM_INFO_LCDPOW,
    SHTERM_INFO_BACKLIGHT,
    SHTERM_INFO_BLUETOOTH,
    SHTERM_INFO_MOBILE_LIGHT,
    SHTERM_INFO_MUSIC,
    SHTERM_INFO_LINE_RINGING,
    SHTERM_INFO_FM_TX,
    SHTERM_INFO_WLAN_TXRX,
    SHTERM_INFO_SPEAKER_LEV,
    SHTERM_INFO_BACKLIGHT_LEV,
    SHTERM_INFO_IR,
    SHTERM_INFO_SD,
    SHTERM_INFO_GBNAND,
    SHTERM_INFO_USB,
    SHTERM_INFO_WIFI,
    SHTERM_INFO_GPS,
    SHTERM_INFO_ACCELE,
    SHTERM_INFO_COMPS,
    SHTERM_MAX
};

/* events */
typedef enum {
    SHBATTLOG_EVENT_NONE,

    SHBATTLOG_EVENT_POWER_ON,

    SHBATTLOG_EVENT_ON_INFO,

    SHBATTLOG_EVENT_POWER_OFF,
    SHBATTLOG_EVENT_QUIT_OFF_CHG,

    SHBATTLOG_EVENT_KEY_OFF,
    SHBATTLOG_EVENT_FATAL_BATT,
    SHBATTLOG_EVENT_BATT_ID_INVALID,

    SHBATTLOG_EVENT_BATT_REPORT_NORM,
    SHBATTLOG_EVENT_BATT_REPORT_CHG,

    SHBATTLOG_EVENT_INDICATER_0,
    SHBATTLOG_EVENT_INDICATER_1,
    SHBATTLOG_EVENT_INDICATER_2,
    SHBATTLOG_EVENT_INDICATER_3,

    SHBATTLOG_EVENT_LOW_BATT_TRUE,
    SHBATTLOG_EVENT_LOW_BATT_FALSE,

    SHBATTLOG_EVENT_CHG_INSERT_CHGR,
    SHBATTLOG_EVENT_CHG_REMOVE_CHGR,
    SHBATTLOG_EVENT_CHG_INSERT_USB,
    SHBATTLOG_EVENT_CHG_REMOVE_USB,

    SHBATTLOG_EVENT_CHG_END,
    SHBATTLOG_EVENT_CHG_START,
    SHBATTLOG_EVENT_CHG_COMP,
    SHBATTLOG_EVENT_CHG_RESTART,
    SHBATTLOG_EVENT_CHG_ERROR,

    SHBATTLOG_EVENT_CHG_IDLE_ST,
    SHBATTLOG_EVENT_CHG_TRICKLE_ST,
    SHBATTLOG_EVENT_CHG_FAST_ST,
    SHBATTLOG_EVENT_CHG_MAINT_ST,
    SHBATTLOG_EVENT_CHG_ADD_FAST_ST,
    SHBATTLOG_EVENT_CHG_HOT_FAST_ST,
    SHBATTLOG_EVENT_CHG_HOT_ADD_FAST_ST,
    SHBATTLOG_EVENT_CHG_HOT_TRICKLE_ST,
    SHBATTLOG_EVENT_CHG_HOT_ADD_TRICKLE_ST,
    SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST,
    SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST,
    SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST,
    SHBATTLOG_EVENT_CHG_ERR_BAT_ID_INVALID_ST,

    SHBATTLOG_EVENT_CHG_HOT_STOP_ST,
    SHBATTLOG_EVENT_CHG_COLD_STOP_ST,
    SHBATTLOG_EVENT_CHG_MAINT_STOP_ST,
    SHBATTLOG_EVENT_CHG_MAINT_HOT_STOP_ST,
    SHBATTLOG_EVENT_CHG_MAINT_COLD_STOP_ST,

    SHBATTLOG_EVENT_BATT_REPORT_DETERIORATED,

    SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST,

    SHBATTLOG_EVENT_CHG_LED_ON,
    SHBATTLOG_EVENT_CHG_LED_BLINK,
    SHBATTLOG_EVENT_CHG_LED_OFF,

    SHBATTLOG_EVENT_FGIC_EX10,
    SHBATTLOG_EVENT_FGIC_EX20,
    SHBATTLOG_EVENT_FGIC_EX30,
    SHBATTLOG_EVENT_FGIC_EX40,
    SHBATTLOG_EVENT_FGIC_EX50,
    SHBATTLOG_EVENT_FGIC_EX60,
    SHBATTLOG_EVENT_FGIC_EX70,
    SHBATTLOG_EVENT_FGIC_EX80,
    SHBATTLOG_EVENT_FGIC_EX90,
    SHBATTLOG_EVENT_FGIC_EX100,

    SHBATTLOG_EVENT_FGIC_NOMAL,
    SHBATTLOG_EVENT_FGIC_INVALID,

    SHBATTLOG_EVENT_OVER_CURRENT1,
    SHBATTLOG_EVENT_OVER_CURRENT2,

    SHBATTLOG_EVENT_OVER_CURR1_DET,
    SHBATTLOG_EVENT_OVER_CURR2_DET,

    SHBATTLOG_EVENT_OVER_CURR3_DET,
    SHBATTLOG_EVENT_OVER_CURR4_DET,

    SHBATTLOG_EVENT_OVER_CURR1_RELEASE,
    SHBATTLOG_EVENT_OVER_CURR2_RELEASE,

    SHBATTLOG_EVENT_HIGH_TEMP,

    SHBATTLOG_EVENT_VOICE_START,
    SHBATTLOG_EVENT_VOICE_END,
    SHBATTLOG_EVENT_DATA_START,
    SHBATTLOG_EVENT_DATA_END,

    SHBATTLOG_EVENT_KERNEL_POWER_OFF,
    SHBATTLOG_EVENT_AARM_REBOOT,
    SHBATTLOG_EVENT_SYS_REBOOT,

#ifdef CONFIG_SH_DETECT_HIGH_TEMP
    SHBATTLOG_EVENT_DETECT_HIGH_TEMP,
    SHBATTLOG_EVENT_DETECT_LOW_TEMP,
    SHBATTLOG_EVENT_EMERGENCY_ON,
    SHBATTLOG_EVENT_EMERGENCY_OFF,
#endif /* CONFIG_SH_DETECT_HIGH_TEMP */

    SHBATTLOG_EVENT_PPP_START,
    SHBATTLOG_EVENT_PPP_END,
    SHBATTLOG_EVENT_DATA_NG1,
    SHBATTLOG_EVENT_DATA_NG2,
    SHBATTLOG_EVENT_DATA_NG3,
    SHBATTLOG_EVENT_DATA_NG4,
    SHBATTLOG_EVENT_DATA_NG5,
    SHBATTLOG_EVENT_DATA_NG6,
    SHBATTLOG_EVENT_DATA_NG7,
    SHBATTLOG_EVENT_DATA_NG8,
    SHBATTLOG_EVENT_DATA_NG9,
    SHBATTLOG_EVENT_DATA_NG10,
    SHBATTLOG_EVENT_DATA_NG11,
    SHBATTLOG_EVENT_DATA_NG12,
    SHBATTLOG_EVENT_DATA_NG13,
    SHBATTLOG_EVENT_DATA_NG14,
    SHBATTLOG_EVENT_DATA_NG15,
    SHBATTLOG_EVENT_DATA_NG16,
    SHBATTLOG_EVENT_DATA_NG17,
    SHBATTLOG_EVENT_DATA_NG18,
    SHBATTLOG_EVENT_DATA_NG19,
    SHBATTLOG_EVENT_DATA_NG20,

    SHBATTLOG_EVENT_QUEUE_FULL,

    SHBATTLOG_EVENT_MAX = 0x7fffffff
} shbattlog_event_num;

typedef struct _shbattlog_info_t {
    int event_num;
    int bat_vol;
    int bat_temp;
    int chg_temp;
    int cam_temp;
    int pmic_temp;
    int pa_temp;
    int avg_cur;
    int avg_vol;
    int chg_vol;
    int chg_cur;
    int latest_cur;
    int acc_cur;
#ifdef CONFIG_SH_DETECT_HIGH_TEMP
    int tmp_cut;
#endif /* CONFIG_SH_DETECT_HIGH_TEMP */
} shbattlog_info_t;

extern int shterm_k_set_info( unsigned long int shterm_info_id, unsigned long int shterm_info_value );

extern int shterm_k_set_event( shbattlog_info_t *info );

extern int shterm_flip_status_set( int state );

#endif /* _SHTERM_K_H_ */
