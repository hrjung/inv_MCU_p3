/*
 * table.c
 *
 *  Created on: 2019. 6. 3.
 *      Author: hrjung
 */

#include "includes.h"
#include "proc_uart.h"

#include "table.h"
#include "crc32.h"

#include "dsp_comm.h"
#include "nvm_queue.h"
#include "drv_nvm.h"
#include "ext_io.h"
#include "error.h"
#include "handler.h"


#define ERRINFO_ITEM_CNT	4


#define	FREQ_MIN_VALUE		10
#define FREQ_MAX_VALUE		900


int32_t table_data[PARAM_TABLE_SIZE]; // parameter table of actual value
int16_t st_ipm_temp_warning=0;
int16_t st_mtr_temp_warning=0;

extern int32_t table_nvm[];
extern int32_t table_dbg[];

extern int16_t state_run_stop; // run_stop status
extern int16_t state_direction;
extern int16_t st_overload;
extern int16_t st_brake;
extern int16_t gear_ratio;

extern int16_t dbg_warn_test;

extern int32_t prev_adc_cmd;

extern COMM_CMD_t test_cmd;

#ifdef SUPPORT_TASK_WATCHDOG
extern uint8_t watchdog_f;
#endif

#ifdef SUPPORT_PASSWORD
int password_enabled=0;
#endif

TABLE_DSP_PARAM_t table_getDspAddr(PARAM_IDX_t index);
uint8_t table_getWriteOnRunning(PARAM_IDX_t index);

STATIC int8_t table_doNothing(PARAM_IDX_t idx, int32_t value, int16_t option);
int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t opt);
int8_t table_setCommandFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);
int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);
int8_t table_setMultiFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setValueMin(PARAM_IDX_t idx, int32_t value, int16_t opt);
STATIC int8_t table_setValueMax(PARAM_IDX_t idx, int32_t value, int16_t opt);
STATIC int8_t table_setValueDir(PARAM_IDX_t idx, int32_t value, int16_t opt);
int8_t table_setCtrlIn(PARAM_IDX_t idx, int32_t value, int16_t option);
#ifdef SUPPORT_PASSWORD
STATIC int8_t table_setPassword(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setLockValue(PARAM_IDX_t idx, int32_t value, int16_t option);
#endif
STATIC int8_t table_setDinValue(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setAinValue(PARAM_IDX_t idx, int32_t value, int16_t opt);
STATIC int8_t table_setAinFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setBaudValue(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setCommValue(PARAM_IDX_t idx, int32_t value, int16_t option);
STATIC int8_t table_setFactoryValue(PARAM_IDX_t index, int32_t value, int16_t option);
STATIC int8_t table_setTimeValue(PARAM_IDX_t index, int32_t value, int16_t option);
int8_t table_setStatusValue(PARAM_IDX_t idx, int32_t value, int16_t option);

extern void MB_UART_init(uint32_t baudrate_index);
extern void MB_initTimer(int32_t b_index);
extern void MB_setSlaveAddress(uint8_t addr);

extern int8_t main_isForceReset(void);
#ifdef SUPPORT_PASSWORD
extern void main_setPassCounterStart(void);
#endif

extern int8_t COMM_getMotorType(int8_t *status);

extern void UTIL_startADC(void);
extern void UTIL_stopADC(void);

extern void EXT_DI_initStopFlag(void);
//extern int8_t HDLR_isFactoryModeEnabled(void);
//extern int HDLR_isStopInProgress(void);
//extern void HDLR_setStopFlag(uint8_t flag);

STATIC Param_t param_table[] =
{   //    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx			param_func
	{ value_type,			0x100,	40100,	0,		100,	800,	1, 	10,		1, 	value_dsp,			table_setCommandFreqValue, },
	{ freq_min_type,		0x104,	40101,	100,	100,	800,	1, 	10, 	0, 	none_dsp,			table_setValueMin, },
	{ freq_max_type,		0x108,	40102,	800,	100,	800,	1, 	10, 	0, 	freq_max_dsp,		table_setValueMax,},
	{ accel_time_type,		0x10C,	40103,	100,	10,		6000,	1, 	10, 	1, 	accel_time_dsp,		table_setValue,},
	{ decel_time_type,		0x110,	40104,	100,	10,		6000,	1, 	10,		1, 	decel_time_dsp,		table_setValue,},
	{ acc_base_set_type,	0x114,	40105,	0,		0,		1,		1, 	1, 		0, 	acc_base_set_dsp, 	table_setValue,	},
	{ dir_cmd_type,			0x118,	40106,	0,		0,		1,		1, 	1, 		1, 	dir_cmd_dsp,		table_setValue,},
	{ dir_domain_type,		0x11C,	40107,	0,		0,		2,		1, 	1, 		0, 	none_dsp,			table_setValueDir,},

	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx			param_func
	{ ctrl_in_type,			0x200,	40200,	0,		0,		4,		1, 	1, 		0, 	none_dsp,			table_setCtrlIn},
	{ energy_save_type,		0x204,	40201,	0,		0,		1,		1, 	1, 		0, 	energy_save_dsp,	table_setValue	},
	{ pwm_freq_type,		0x208,	40202,	0,		0,		3,		1, 	1, 		0, 	pwm_freq_dsp,		table_setValue	},
	{ jmp_enable0_type,		0x20C,	40203,	0,		0,		1,		1, 	1, 		0, 	jmp_enable0_dsp,	table_setValue,	},
	{ jmp_low0_type,		0x210,	40204,	100,	100,	800,	1, 	10, 	0, 	jmp_low0_dsp, 		table_setFreqValue,},
	{ jmp_high0_type,		0x214,	40205,	100,	100,	800,	1, 	10, 	0, 	jmp_high0_dsp, 		table_setFreqValue,},
	{ jmp_enable1_type,		0x218,	40206,	0,		0,		1,		1, 	1, 		0, 	jmp_enable1_dsp,	table_setValue,	},
	{ jmp_low1_type,		0x21C,	40207,	100,	100,	800,	1, 	10, 	0, 	jmp_low1_dsp, 		table_setFreqValue,},
	{ jmp_high1_type,		0x220,	40208,	100,	100,	800,	1, 	10, 	0, 	jmp_high1_dsp, 		table_setFreqValue,},
	{ jmp_enable2_type,		0x224,	40209,	0,		0,		1,		1, 	1, 		0, 	jmp_enable2_dsp,	table_setValue,	},
	{ jmp_low2_type,		0x228,	40210,	100,	100,	800,	1, 	10, 	0, 	jmp_low2_dsp, 		table_setFreqValue,},
	{ jmp_high2_type,		0x22C,	40211,	100,	100,	800,	1, 	10, 	0, 	jmp_high2_dsp, 		table_setFreqValue,},
	{ brake_type_type,		0x230,	40212,	0,		0,		2,		1, 	1, 		1, 	brake_type_dsp,		table_setValue	},
	{ brake_freq_type,		0x234,	40213,	10,		10,		600,	1, 	10, 	0, 	brake_freq_dsp,		table_setFreqValue	},
	{ dci_brk_freq_type,	0x238,	40214,	30,		10,		600,	1, 	10, 	0, 	dci_brk_freq_dsp,	table_setFreqValue	},
	{ dci_brk_hold_type,	0x23C,	40215,	10,		0,		600,	1,	10, 	0, 	dci_brk_hold_dsp,	table_setValue	},
	{ dci_brk_time_type,	0x240,	40216,	50,		0,		600,	1, 	10, 	0, 	dci_brk_time_dsp,	table_setValue	},
	{ dci_brk_rate_type,	0x244,	40217,	500,	0,		2000,	1, 	10, 	0, 	dci_brk_rate_dsp,	table_setValue	},

	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx			param_func
	{ ovl_warn_limit_type,	0x280,	40280,	150,	100,	200,	1,	1, 		1, 	ovl_warn_limit_dsp,	table_setValue	},
	{ ovl_warn_dur_type,	0x284,	40281,	10,		0,		30,		1,	1, 		1, 	ovl_warn_dur_dsp,	table_setValue	},
	{ ovl_enable_type,		0x288,	40282,	1,		0,		1,		1,	1, 		1, 	ovl_enable_dsp,		table_setValue	},
	{ ovl_trip_limit_type,	0x28C,	40283,	180,	100,	200,	1,	1,		1, 	ovl_trip_limit_dsp, table_setValue	},
	{ ovl_trip_dur_type,	0x290,	40284,	30,		0,		60,		1,	1, 		1, 	ovl_trip_dur_dsp,	table_setValue	},
	{ regen_duty_type,		0x294,	40285,	30,		0,		80,		1,	1, 		0, 	regen_duty_dsp,		table_setValue	},
	{ regen_band_type,		0x298,	40286,	10,		0,		80,		1,	1, 		0, 	regen_band_dsp,		table_setValue	},
	{ fan_onoff_type,		0x29C,	40287,	0,		0,		1,		1,	1, 		1, 	fan_onoff_dsp,		table_setValue	},
#ifdef SUPPORT_PASSWORD
	{ password_type,		0x2A0,	40288,	0,		0,		9999,	1,	1, 		1, 	none_dsp,			table_setPassword	},
	{ modify_lock_type,		0x2A4,	40289,	0,		0,		1,		1,	1, 		1, 	none_dsp,			table_setLockValue	},
#endif

	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx		param_func
	{ multi_Din_0_type,		0x300,	40300,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setDinValue	},
	{ multi_Din_1_type,		0x304,	40301,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setDinValue	},
	{ multi_Din_2_type,		0x308,	40302,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setDinValue	},
	{ multi_val_0_type,		0x30C,	40303,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_1_type,		0x310,	40304,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_2_type,		0x314,	40305,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_3_type,		0x318,	40306,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_4_type,		0x31C,	40307,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_5_type,		0x320,	40308,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_6_type,		0x324,	40309,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_val_7_type,		0x328,	40310,	200,	100,	800,	1, 	10, 	1, 	none_dsp,		table_setMultiFreqValue, },
	{ multi_Dout_0_type,	0x32C,	40311,	0,		0,		5,		1,	1, 		1, 	none_dsp,		table_setValue	},
	{ multi_Dout_1_type,	0x330,	40312,	0,		0,		5,		1,	1, 		1, 	none_dsp,		table_setValue	},
	{ v_in_min_type,		0x334,	40313,	10,		0,		100,	1,	10, 	1, 	none_dsp,		table_setAinValue	},
	{ v_in_min_freq_type,	0x338,	40314,	100,	100,	700,	1,	10, 	1, 	none_dsp,		table_setAinFreqValue	},
	{ v_in_max_type,		0x33C,	40315,	100,	0,		100,	1,	10, 	1, 	none_dsp,		table_setAinValue	},
	{ v_in_max_freq_type,	0x340,	40316,	600,	100,	800,	1,	10, 	1, 	none_dsp,		table_setAinFreqValue	},
	{ mb_address_type,		0x344,	40317,	1,		1,		247,	1,	1, 		1, 	none_dsp,		table_setCommValue	},
	{ baudrate_type,		0x348,	40318,	2,		0,		5,		1,	1, 		1, 	none_dsp,		table_setBaudValue	},


	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, DSPcomm		do nothing for read only
	{ model_type,			0x20,	40020,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setFactoryValue},
	{ motor_type_type,		0x24,	40021,	0,		0,		3,		0, 	1, 		0, 	motor_type_dsp,	table_setFactoryValue},
	{ gear_ratio_type,		0x28,	40022,	1,		0,		200,	0, 	1, 		0,	none_dsp,		table_setFactoryValue},
	{ fw_ver_type,			0x2C,	40023,	0,		0,		0,		0,	1, 		0, 	none_dsp,		table_setFactoryValue},


	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, DSPcomm
	{ err_code_1_type,		0x380,	40401,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_status_1_type,	0x384,	40402,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_current_1_type,	0x388,	40403,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},
	{ err_freq_1_type,		0x38C,	40404,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},

	{ err_code_2_type,		0x390,	40405,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_status_2_type,	0x394,	40406,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_current_2_type,	0x398,	40407,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},
	{ err_freq_2_type,		0x39C,	40408,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},

	{ err_code_3_type,		0x3A0,	40409,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_status_3_type,	0x3A4,	40410,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_current_3_type,	0x3A8,	40411,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},
	{ err_freq_2_type,		0x3AC,	40412,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},

	{ err_code_4_type,		0x3B0,	40413,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_status_4_type,	0x3B4,	40414,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_current_4_type,	0x3B8,	40415,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},
	{ err_freq_4_type,		0x3BC,	40416,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},

	{ err_code_5_type,		0x3C0,	40417,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_status_5_type,	0x3C4,	40418,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_doNothing},
	{ err_current_5_type,	0x3C8,	40419,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},
	{ err_freq_5_type,		0x3CC,	40420,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_doNothing},

	{ run_status1_type,		0x80,	40160,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setStatusValue},
	{ run_status2_type,		0x84,	40161,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setStatusValue},
	{ I_rms_type,			0x88,	40162,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ run_freq_type,		0x8C,	40163,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ dc_voltage_type,		0x90,	40164,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
#ifdef SUPPORT_STATUS_TORQUE
	{ torque_value_type,	0x94,	40165,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ torque_percent_type,	0x98,	40166,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
#endif
	{ ipm_temperature_type,	0x9C,	40167,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ mtr_temperature_type,	0xA0,	40168,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ di_status_type,		0xA4,	40169,	0,		0,		0,		0, 	1,		0, 	none_dsp,		table_setStatusValue},
	{ do_status_type,		0xA8,	40170,	0,		0,		0,		0, 	1,		0, 	none_dsp,		table_setStatusValue},
	{ ai_status_type,		0xAC,	40171,	0,		0,		0,		0, 	10,		0, 	none_dsp,		table_setStatusValue},
	{ motor_on_cnt_type,	0xB0,	40172,	0,		0,		0,		0,	1, 		0, 	none_dsp,		table_setTimeValue},
	{ elapsed_hour_type,	0xB4,	40173,	0,		0,		0,		0,	1, 		0, 	none_dsp,		table_setTimeValue},
	{ operating_hour_type,	0xB8,	40174,	0,		0,		0,		0,	1, 		0, 	none_dsp,		table_setTimeValue},

};


/*
 * 		param_func implementation
 */


static int table_checkValidity(PARAM_IDX_t idx, int32_t value)
{
	if(value < param_table[idx].minValue || value > param_table[idx].maxValue)
	{
		kprintf(PORT_DEBUG,"idx=%d value=%d is not valid\r\n", idx, value);
		return 0;
	}

	return 1;
}

static int table_checkFreqValidity(PARAM_IDX_t idx, int32_t value)
{
	if( (value < param_table[idx].minValue && value != 0) // allow set freq value 0
		|| value > param_table[idx].maxValue)
	{
		kprintf(PORT_DEBUG,"idx=%d value=%d is not valid\r\n", idx, value);
		return 0;
	}

	return 1;
}

int table_checkValidityNFC(PARAM_IDX_t idx, int32_t value)
{
	int freq_index[] = {
			value_type,
			multi_val_0_type,
			multi_val_1_type,
			multi_val_2_type,
			multi_val_3_type,

			multi_val_4_type,
			multi_val_5_type,
			multi_val_6_type,
			multi_val_7_type,
			v_in_min_freq_type,
	};
	int i, check_cnt=10;


	if(value < param_table[idx].minValue || value > param_table[idx].maxValue)
	{
		for(i=0; i<check_cnt; i++) // freq value could be 0 freq
		{
			if(idx == freq_index[i])
			{
				if(value == 0) return 1;
			}
		}

		kprintf(PORT_DEBUG,"idx=%d value=%d is not valid\r\n", idx, value);
		return 0;
	}

	return 1;
}

static int8_t table_setValueAPI(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;
	uint16_t buf[3] = {0,0,0};

	if(table_data[idx] == value) return 1; // ignore

	// check status= RUN, writeOnRunning = 0
	if(table_getWriteOnRunning(idx) == 0 && state_run_stop == CMD_RUN)
	{
		kprintf(PORT_DEBUG,"idx=%d cannot write on running\r\n", idx);
		return 0;
	}

	if(option != REQ_FROM_TEST && option != REQ_FROM_NFC)
	{
		// request to update EEPROM
		status = NVMQ_enqueueNfcQ(idx, value);
		if(status == 0) return 0;
	}

	table_data[idx] = value;
	table_nvm[idx] = value;
	//kprintf(PORT_DEBUG,"table_setValueAPI: idx=%d set value=%d\r\n", idx, value);
	if(option > REQ_FROM_DSP) // need DSP comm
	{
		if(table_getDspAddr(idx) != none_dsp)
		{
			test_cmd = SPICMD_PARAM_W;
			COMM_convertValue((uint16_t)idx, buf);
#ifndef SUPPORT_UNIT_TEST
			status = COMM_sendMessage(SPICMD_PARAM_W, buf);
			if(status == 0) { kprintf(PORT_DEBUG, "set idx=%d value to DSP error! \r\n", idx); return 0;}
#endif
		}
	}

	return 1;
}

int8_t table_doNothing(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	// do nothing
	return 0;
}

int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;

	// writable ?
	if(param_table[idx].isRW == 0) return 0; // read only

	// check validity
	if(table_checkValidity(idx, value) == 0) return 0;

	status = table_setValueAPI(idx, value, option);

	return status;
}

int8_t table_setCommandFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;

	// check validity
	if(table_checkFreqValidity(idx, value) == 0) return 0;

//	if(idx == value_type && HDLR_isStopInProgress()) return 0; // not set freq during stopping

	status = table_setValueAPI(idx, value, option);

	return status;
}

int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;

	// check validity
	if(table_checkFreqValidity(idx, value) == 0) return 0;

	status = table_setValueAPI(idx, value, option);

	return status;
}

int8_t table_setMultiFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;

	// check validity
	if(table_checkFreqValidity(idx, value) == 0) return 0;

	status = table_setValueAPI(idx, value, option);
	if(status == 0) return 0;

	if(table_data[ctrl_in_type] == CTRL_IN_Digital)
	{
		status = EXI_DI_setMultiFreqValue();
	}

	return status;
}

int8_t table_setValueMin(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t result;
	int i, range_size=17;
	static 	PARAM_IDX_t r_idx[] = {
			value_type,
			multi_val_0_type,
			multi_val_1_type,
			multi_val_2_type,
			multi_val_3_type,

			multi_val_4_type,
			multi_val_5_type,
			multi_val_6_type,
			multi_val_7_type,
			jmp_low0_type,

			jmp_low1_type,
			jmp_low2_type,
			jmp_high0_type,
			jmp_high1_type,
			jmp_high2_type,

			v_in_min_freq_type,
			v_in_max_freq_type,
	};

	result = table_setValue(idx, value, option);
	if(result)
	{
		for(i=0; i<range_size; i++)
		{
			param_table[r_idx[i]].minValue = value;
		}
	}

	return result;
}

int8_t table_setValueMax(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t result;
	int i, range_size=17;
	static 	PARAM_IDX_t r_idx[] = {
			value_type,
			multi_val_0_type,
			multi_val_1_type,
			multi_val_2_type,
			multi_val_3_type,

			multi_val_4_type,
			multi_val_5_type,
			multi_val_6_type,
			multi_val_7_type,
			jmp_low0_type,

			jmp_low1_type,
			jmp_low2_type,
			jmp_high0_type,
			jmp_high1_type,
			jmp_high2_type,

			v_in_min_freq_type,
			v_in_max_freq_type,
	};

	result = table_setValue(idx, value, option);
	if(result)
	{
		for(i=0; i<range_size; i++)
		{
			param_table[r_idx[i]].maxValue = value;
		}
	}

	return result;
}


int8_t table_setValueDir(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t result;
	int32_t dir_cmd=0;

	result = table_setValue(idx, value, option);
	if(result)
	{
		// update dir_cmd_type valid range
		if(value == 1)
		{
			param_table[dir_cmd_type].minValue = 0;
			param_table[dir_cmd_type].maxValue = 0;
//			table_data[dir_cmd_type] = 0;
//			NVM_readParam(dir_cmd_type, &dir_cmd);
//			if(dir_cmd != table_data[dir_cmd_type])
//				NVM_writeParam(dir_cmd_type, table_data[dir_cmd_type]); // update command value
		}
		else if(value == 2)
		{
			param_table[dir_cmd_type].minValue = 1;
			param_table[dir_cmd_type].maxValue = 1;
//			table_data[dir_cmd_type] = 1;
//			NVM_readParam(dir_cmd_type, &dir_cmd);
//			if(dir_cmd != table_data[dir_cmd_type])
//				NVM_writeParam(dir_cmd_type, table_data[dir_cmd_type]); // update command value
		}
		else
		{
			param_table[dir_cmd_type].minValue = 0;
			param_table[dir_cmd_type].maxValue = 1;
		}
	}

	return result;
}

int8_t table_setCtrlIn(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	status = table_setValue(idx, value, option);
	if(status == 0) return 0;

	if(value == CTRL_IN_Digital	|| value == CTRL_IN_Din_Ain)
	{
		HDLR_setStopFlag(0); // init stop
	}

	if(value == CTRL_IN_Analog_V || value == CTRL_IN_Din_Ain)
	{
		UTIL_startADC();
	}
	else
	{
		UTIL_stopADC();
		prev_adc_cmd = 1; // initialize
	}

	return status;
}

#ifdef SUPPORT_PASSWORD
int table_isLocked(void)
{
	return (table_getValue(modify_lock_type) != (int32_t)0);
}

int table_isPasswordAddrModbus(uint16_t addr)
{
	return (param_table[password_type].mb_addr == addr);
}

int table_isLockAddrModbus(uint16_t addr)
{
	return (param_table[modify_lock_type].mb_addr == addr);
}

void table_clearPassEnable(void)
{
	password_enabled=0;
}

int table_isPassEnabled(void)
{
	if(table_getValue(password_type) == 0) return 1; // password is not set

	if(password_enabled == 1) return 1; // check password is enabled

	return 0;
}

int8_t table_setPassword(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status;

	if(table_isPassEnabled()) // update password
	{
		// check validity
		if(table_checkValidity(idx, value) == 0) return 0;

		status = table_setValueAPI(idx, value, option);
	}
	else
	{
		if(value == (uint16_t)table_getValue(password_type))
		{
			password_enabled = 1;
			main_setPassCounterStart();
			status = 1; // OK
		}
		else
			status = 0; // not correct password
	}
	return status;
}

int8_t table_setLockValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	if(!table_isPassEnabled()) return 0; // check password is enabled

	status = table_setValue(idx, value, option);
	if(status == 0) return 0;

	return status;
}
#endif

int8_t table_setDinValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;
	int index;

	status = table_setValue(idx, value, option);
	if(status == 0) return 0;

	index = (int)(idx - multi_Din_0_type);
	status = EXT_DI_setupMultiFuncDin(index, (DIN_config_t)value, option);

	if(table_data[ctrl_in_type] == CTRL_IN_Digital)
		EXI_DI_getStepValue(EXT_DI_INITIALIZE);

	return status;
}

int8_t table_setAinValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	status = table_setValue(idx, value, option);

	EXT_AI_needReconfig();

	return status;
}

int8_t table_setAinFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	status = table_setFreqValue(idx, value, option);

	EXT_AI_needReconfig();

	return status;
}

int8_t table_setBaudValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	//value = 2; // TODO: NFC app bug
	status = table_setValue(idx, value, option);

	if(status)
	{
		MB_UART_init(value);
		MB_initTimer(value);
	}

	return status;
}

int8_t table_setCommValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	int8_t status=1;

	status = table_setValue(idx, value, option);

	if(status)
		MB_setSlaveAddress((uint8_t)value);

	return status;
}

int8_t table_setFactoryValue(PARAM_IDX_t index, int32_t value, int16_t option)
{
	int8_t status;

	if(!HDLR_isFactoryModeEnabled()) return 0;  // read only in non-factory mode

	if(table_data[index] == value) return 1; // ignore

	if(option != REQ_FROM_TEST)
	{
		// request to update EEPROM
		status = NVMQ_enqueueNfcQ(index, value);
		if(status == 0) return 0;
	}

	table_data[index] = value;
	table_nvm[index] = value;
	//printf("idx=%d set value=%d\r\n", idx, value);

	return 1;
}

int8_t table_setTimeValue(PARAM_IDX_t index, int32_t value, int16_t option)
{
	int8_t status;

	if(option == REQ_FROM_MODBUS) return 1; // ignore, only set by internal

	if(table_data[index] == value) return 1; // ignore

	if(option != REQ_FROM_TEST)
	{
		// request to update EEPROM
		status = NVMQ_enqueueNfcQ(index, value);
		if(status == 0) return 0;
	}

	table_data[index] = value;
	table_nvm[index] = value;
	//printf("idx=%d set value=%d\r\n", idx, value);

	return 1;
}

void table_getStatusFromTable(int32_t *status, float *current, float *freq)
{
	*status = (table_data[run_status1_type]&0x0F);
	*current = (float)table_data[I_rms_type]/10.0;
	*freq = (float)table_data[run_freq_type]/10.0;
}

int8_t table_setStatusValue(PARAM_IDX_t index, int32_t value, int16_t option)
{
	table_data[index] = value;
	table_nvm[index] = value;

	if(index == run_status1_type)
	{
		if((value&0x0F) <= STATE_STOP)	state_run_stop = CMD_STOP;	// stop
		else	state_run_stop = CMD_RUN;		// RUN
		//state_run_stop = (value>>8)&0x01;

		state_direction = (value>>8)&0x01; 	// forward/backward

		NVM_setMotorStatus((int32_t)state_run_stop); // set NFC App flag
	}
	else if(index == run_status2_type)
	{
		st_overload = value&0x01;		// overload on/off
		st_brake = (value>>8)&0x01;  	// external brake on/off
	}
	else if(index == ipm_temperature_type)
	{
		if(value > IPM_TEMPERATURE_WARNNING) // over 90 than warning
			st_ipm_temp_warning = 1;
		else
			st_ipm_temp_warning = 0;
	}
	else if(index == mtr_temperature_type)
	{
		if(value > MOTOR_TEMP_WARNNING) // over 130 degree than warning
			st_mtr_temp_warning = 1;
		else
			st_mtr_temp_warning = 0;
	}

	return 1;
}

int table_isWarning(void)
{
	return(st_overload || st_ipm_temp_warning || st_mtr_temp_warning || dbg_warn_test);
}

void table_setExtStatusValue(void)
{
	int32_t di_val, do_val, ai_val;

	di_val = EXT_getDIValue();
	table_data[di_status_type] = di_val;
	table_nvm[di_status_type] = di_val;

	do_val = EXT_getDOValue();
	table_data[do_status_type] = do_val;
	table_nvm[do_status_type] = do_val;

	ai_val = EXT_getAIValue();
	table_data[ai_status_type] = ai_val;
	table_nvm[ai_status_type] = ai_val;
}

/*
 * 		test only for debug
 *
 */

void test_setTableValue(PARAM_IDX_t idx, int32_t value, int16_t option)
{
	if(option == REQ_FROM_TEST)
		table_data[idx] = value;
}

/*
 * 		table handling function implementation
 */

//int8_t table_isInit(void)
//{
//	return NVM_isInit();
//}

// CRC value for initial value : 0xf2364d63
uint32_t table_calcCRC(void)
{
	int table_length = (int)(baudrate_type+1); // RW data only
	uint8_t* buff = (uint8_t *)table_nvm;
	uint32_t crc32 = update_crc(-1, buff, table_length *sizeof(int32_t));

//	printf("nvm %d, %d, %d %d\r\n", table_nvm[0], table_nvm[1], table_nvm[2], table_nvm[3]);
//	printf("CRC : 0x%x\r\n", crc32);
	return crc32;
}

int8_t table_initializeBlankEEPROM(void)
{
	int i;
	uint8_t status, errflag=0;

	//NVM_clear();
	//printf("initialize blank EEPROM...\r\n");
	// initialize all parameter in table
	for(i=0; i<PARAM_TABLE_SIZE; i++)
	{
		status = NVM_writeParam((PARAM_IDX_t)i, param_table[i].initValue);
		if(status == 0) errflag++;

		table_data[i] = param_table[i].initValue;
		//kprintf("idx=%d: value=%d, nvm=%d\r\n", i, param_table[i].initValue, table_nvm[i]);
#ifdef SUPPORT_TASK_WATCHDOG
		watchdog_f= 0x1F; // WATCHDOG_ALL; to kick watchdog in case of no NFC B/D
#endif
	}

	kprintf(PORT_DEBUG, "0: err=%d\r\n", errflag);

	// init system param
	status = NVM_initSystemParam();
	if(status == 0) errflag++;

#ifdef SUPPORT_PARAMETER_BACKUP
	HDLR_clearBackupFlag();
#endif

	kprintf(PORT_DEBUG, "1: err=%d\r\n", errflag);

	status = NVM_initTime();
	if(status == 0) errflag++;

	kprintf(PORT_DEBUG, "2: err=%d\r\n", errflag);

	//NVM_setInit(); // not use init flag

	NVM_setCRC();

	if(errflag) return 0;

	return 1;
}

int8_t table_loadEEPROM(void)
{
	int i;
	uint8_t status;
	int32_t value;
	uint32_t crc32_calc;

	for(i=0; i<PARAM_TABLE_SIZE; i++)
	{
		status = NVM_readParam((PARAM_IDX_t)i, &value);
		if(status == NVM_NOK)
		{
			kprintf(PORT_DEBUG, " table_loadEEPROM idx=%d read error, value=%d\r\n", i, (int)value);
			return 0;
		}
		else
			table_data[i] = value;

	}

	// check CRC
	crc32_calc = table_calcCRC();
	status = NVM_verifyCRC(crc32_calc);
	kprintf(PORT_DEBUG, "table_loadEEPROM verify status=%d\r\n", status);
	return status;
}

int8_t table_updateRange(void)
{
	int i, range_size=17, index;
	int32_t min_value, max_value, value, dir_cmd=0;
	PARAM_IDX_t r_idx[] = {
			value_type,
			multi_val_0_type,
			multi_val_1_type,
			multi_val_2_type,
			multi_val_3_type,

			multi_val_4_type,
			multi_val_5_type,
			multi_val_6_type,
			multi_val_7_type,
			jmp_low0_type,

			jmp_low1_type,
			jmp_low2_type,
			jmp_high0_type,
			jmp_high1_type,
			jmp_high2_type,

			v_in_min_freq_type,
			v_in_max_freq_type,
	};

	min_value = table_data[freq_min_type];
	max_value = table_data[freq_max_type];
	for(i=0; i<range_size; i++)
	{
		index = r_idx[i];
		param_table[index].minValue = min_value;
		param_table[index].maxValue = max_value;

#if 0 // dont need to re-set min, max value
		value = table_data[index];
		if(value < param_table[index].minValue && value != 0 )
		{
			table_data[index] = param_table[index].minValue;
			kprintf(PORT_DEBUG, "table_updateRange min idx=%d, value=%d\r\n", r_idx[i], table_data[r_idx[i]]);
		}

		if(value > param_table[r_idx[i]].maxValue)
		{
			table_data[index] = param_table[index].maxValue;
			kprintf(PORT_DEBUG, "table_updateRange max idx=%d, value=%d\r\n", r_idx[i], table_data[r_idx[i]]);
		}
#endif
	}

	// update dir range and cmd value
	value = table_data[dir_domain_type];
	//kprintf(PORT_DEBUG, "dir_domain value=%d\r\n", value);
	if(value == DIR_FORWARD_ONLY)
	{
		param_table[dir_cmd_type].minValue = 0; // forward only
		param_table[dir_cmd_type].maxValue = 0;
//		table_data[dir_cmd_type] = 0;
//		NVM_readParam(dir_cmd_type, &dir_cmd);
//		if(dir_cmd != table_data[dir_cmd_type])
//			NVM_writeParam(dir_cmd_type, table_data[dir_cmd_type]); // update command value

	}
	else if(value == DIR_REVERSE_ONLY)
	{
		param_table[dir_cmd_type].minValue = 1; // reverse only
		param_table[dir_cmd_type].maxValue = 1;
//		table_data[dir_cmd_type] = 1;
//		NVM_readParam(dir_cmd_type, &dir_cmd);
//		if(dir_cmd != table_data[dir_cmd_type])
//			NVM_writeParam(dir_cmd_type, table_data[dir_cmd_type]);
	}

	return 1;
}

int8_t table_getMotorType(void)
{
	int8_t status=1;
	int32_t motor_type, value;

	// read motor type from DSP
	motor_type = COMM_getMotorType(&status);
	if(status == 0)
	{
		osDelay(50);
		motor_type = COMM_getMotorType(&status); // try again
		if(status == 0)
		{
			ERR_setErrorState(TRIP_REASON_MCU_COMM_FAIL);
			kprintf(PORT_DEBUG, " getType error status=%d \r\n", status);
			return status;
		}
	}

	value = table_getValue(motor_type_type);
	if(motor_type != value) // if mis-matched, update NVM
	{
		// valid motor type value
		if(motor_type > MOTOR_NONE_TYPE && motor_type < MOTOR_MAX_TYPE)
		{
			status = NVM_writeParam((PARAM_IDX_t)motor_type_type, motor_type);
			table_data[motor_type_type] = motor_type;
		}
		else
			status = 0;
	}

	kprintf(PORT_DEBUG, " read motor type=%d, table_val=%d, status=%d  \r\n", motor_type, value, status);

	return status;
}

int8_t table_updateFwVersion(void)
{
	int8_t status=1;
	int32_t nvm_fw_ver;
	int32_t fw_ver = (int32_t)((VERSION_MAJ<<8) | VERSION_MIN);

	nvm_fw_ver = table_getValue(fw_ver_type);
	if(nvm_fw_ver != fw_ver)
	{
		status = NVM_writeParam((PARAM_IDX_t)fw_ver_type, fw_ver);
		table_data[fw_ver_type] = fw_ver;
		kprintf(PORT_DEBUG, " need update! nvm_fw_ver=%d, fw_ver=%d status=%d \r\n", nvm_fw_ver, fw_ver, status);
	}

	return status;
}

int8_t table_init(void)
{
	int8_t status, errflag=0;
	int i;
	uint16_t buf[3]={0,0,0}; // index + int32 or float

	status = table_updateRange();
	//if(status == 0) return 0;

	if( !main_isForceReset() ) // no mcu force reset, then send DSP
	{
		for(i=0; i<=baudrate_type; i++)
		{
			if(i == dir_cmd_type) // verify valid direction command
			{
				if(!table_isDirectionValid()) continue; // invalid dir command then not send
			}

			// if value is not initial value than send DSP to sync
			if(table_data[i] != param_table[i].initValue)
			{
				if(table_getDspAddr(i) == none_dsp) continue;

				COMM_convertValue((uint16_t)i, buf);

				status = COMM_sendMessage(SPICMD_PARAM_W, buf);
				kprintf(PORT_DEBUG, "DSP COMM : status=%d, idx=%d, value=%d, param=%d\r\n", \
						status, i, (int)table_data[i], param_table[i].initValue);
				if(status == 0) // retry
				{
					status = COMM_sendMessage(SPICMD_PARAM_W, buf);
					if(status == 0) errflag++;
				}
			}
		}
	}

	// sync EEPROM data and table data
	for(i=0; i<PARAM_TABLE_SIZE; i++) table_nvm[i] = table_data[i];

	// clear system flag at init
	if( !main_isForceReset() )
		NVM_initSystemFlagStartup();

	status = NVM_readTime();
	if(status == 0) errflag++;

	if(errflag) return 0;

	return 1;
}

int8_t table_initNVM(void)
{
	int8_t status;
	//int32_t motor_type, value;

	// initialize EEPROM
//	if(table_isInit() == 1) // correctly initialize
	{
		kputs(PORT_DEBUG, "load EEPROM\r\n");
		// load EEPROM, check CRC
		status = table_loadEEPROM(); // EEPROM -> table_data[]
		kprintf(PORT_DEBUG, "1: status=%d\r\n", status);
		if(status)
		{
			status = table_init();
			kprintf(PORT_DEBUG, "2: status=%d\r\n", status);
		}
		else // try again
		{
			status = table_loadEEPROM();
			kprintf(PORT_DEBUG, "retry 1: status=%d\r\n", status);
			if(status)
			{
				status = table_init();
				kprintf(PORT_DEBUG, "retry 2: status=%d\r\n", status);
			}
			else
				return 0; // load EEPROM error -> trip
		}
	}
#if 0
	else
	{
		// blank NVM : initialize as init value
		kputs(PORT_DEBUG, "blank EEPROM\r\n");
		status = table_initializeBlankEEPROM();
	}
#endif

	status = table_getMotorType();

	status = table_updateFwVersion();

	return status;
}

// return 0 for error
// return 1 for int value
//        10 for float value
uint16_t table_getRatio(PARAM_IDX_t tbl_index)
{
	if(tbl_index < 0 || tbl_index >= PARAM_TABLE_SIZE) { kprintf(PORT_DEBUG, "parameter index error %d\r\n", tbl_index); return 0; }

	return param_table[tbl_index].ratio;
}

uint8_t table_getRW(PARAM_IDX_t index)
{
	return param_table[index].isRW;
}

TABLE_DSP_PARAM_t table_getDspAddr(PARAM_IDX_t index)
{
	return param_table[index].dsp_idx;
}

uint16_t table_getAddr(PARAM_IDX_t index)
{
	if(index >= PARAM_TABLE_SIZE) return 0;

	return param_table[index].addr;
}

uint8_t table_getWriteOnRunning(PARAM_IDX_t index)
{
	return param_table[index].isWriterbleOnRunnuning;
}

int32_t table_getValue(PARAM_IDX_t index)
{
	return table_data[index];
}

int32_t table_getInitValue(PARAM_IDX_t index)
{
	return param_table[index].initValue;
}

int8_t table_setValueFactoryMode(PARAM_IDX_t idx, int32_t value)
{
	int8_t status;

	if(table_data[idx] == value) return 1; // ignore

	// request to update EEPROM
	status = NVMQ_enqueueNfcQ(idx, value);
	if(status == 0) return 0;

	table_data[idx] = value;
	table_nvm[idx] = value;

	return 1;
}

int8_t table_runFunc(PARAM_IDX_t idx, int32_t value, int16_t opt)
{
#ifdef SUPPORT_UNIT_TEST
	return param_table[idx].param_func(idx, value, REQ_FROM_TEST);
#else
	return param_table[idx].param_func(idx, value, opt);
#endif
}

int32_t table_getCtrllIn(void)
{
	return table_data[ctrl_in_type];
}


int32_t table_getStatusValue(int16_t index)
{
	if(index >= run_status1_type && index < PARAM_TABLE_SIZE)
		return table_data[index];
	else
		kprintf(PORT_DEBUG, "table_getStatusValue index=%d error\r\n", index);

	return 0;
}

int8_t table_initStatusError(uint16_t index)
{
	if(index >= err_code_1_type && index < PARAM_TABLE_SIZE)
	{
		table_data[index] = 0;
		return 1;
	}
	else
		return 0;
}

int table_isMotorStop(void)
{
	return (state_run_stop == CMD_STOP);
}

int8_t table_setStatusDSP(void)
{
	int8_t nvm_status;
	int8_t errflag=0;

	nvm_status = NVMQ_enqueueNfcQ(run_status1_type, table_data[run_status1_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(run_status2_type, table_data[run_status2_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(I_rms_type, table_data[I_rms_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(run_freq_type, table_data[run_freq_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(dc_voltage_type, table_data[dc_voltage_type]);
	if(nvm_status == 0) errflag++;
#ifdef SUPPORT_STATUS_TORQUE
	nvm_status = NVMQ_enqueueNfcQ(torque_value_type, table_data[torque_value_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(torque_percent_type, table_data[torque_percent_type]);
	if(nvm_status == 0) errflag++;
#endif
	nvm_status = NVMQ_enqueueNfcQ(ipm_temperature_type, table_data[ipm_temperature_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(mtr_temperature_type, table_data[mtr_temperature_type]);
	if(nvm_status == 0) errflag++;

	nvm_status = NVMQ_enqueueNfcQ(di_status_type, table_data[di_status_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(do_status_type, table_data[do_status_type]);
	if(nvm_status == 0) errflag++;
	nvm_status = NVMQ_enqueueNfcQ(ai_status_type, table_data[ai_status_type]);
	if(nvm_status == 0) errflag++;


	if(errflag) return 0;
	else	return 1;

}

int8_t table_updateErrorDSP(uint16_t err_code, uint16_t status, float current, float freq)
{
	int i;
	uint8_t nvm_status;
	uint16_t index;

	// expect power off, not store at NVM
	if(table_isMotorStop() && err_code == TRIP_REASON_VDC_UNDER)
	{
		table_data[err_code_1_type] = (int32_t)err_code; // only let know error code
		kputs(PORT_DEBUG, "VDC_UNDER at power off\r\n");
		return 1;
	}

	for(i=3; i>=0; i--) // push to next pos
	{
		//printf("i=%d\n", i);
		table_data[err_code_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_code_1_type + i*ERRINFO_ITEM_CNT]; // error code
		table_data[err_status_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_status_1_type + i*ERRINFO_ITEM_CNT]; // status, RUN/STOP/ACCEL/DECEL
		table_data[err_current_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_current_1_type + i*ERRINFO_ITEM_CNT]; // current
		table_data[err_freq_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_freq_1_type + i*ERRINFO_ITEM_CNT]; // freq

#if 1
		// update EEPROM
		if(!ERR_isNvmError())
		{
			index = err_code_1_type + (i+1)*ERRINFO_ITEM_CNT;
			nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
			index = err_status_1_type + (i+1)*ERRINFO_ITEM_CNT;
			nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
			index = err_current_1_type + (i+1)*ERRINFO_ITEM_CNT;
			nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
			index = err_freq_1_type + (i+1)*ERRINFO_ITEM_CNT;
			nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
		}
#endif
	}

	// store latest error info
	if(err_code == TRIP_REASON_MCU_ERR)
		table_data[err_code_1_type] = (int32_t)ERR_getErrorState(); // set MCU error code
	else
		table_data[err_code_1_type] = (int32_t)err_code;

	//kprintf(PORT_DEBUG, "DSP error code tbl_err=%d, err_code=%d\r\n", table_data[err_code_1_type], err_code);

	if(ERR_getErrorState() == TRIP_REASON_MCU_COMM_FAIL) // store status value for COMM error
	{
		table_data[err_current_1_type] = table_getStatusValue(run_status1_type)&0xFF;
		table_data[err_current_1_type] = table_getStatusValue(I_rms_type);
		table_data[err_freq_1_type] = table_getStatusValue(run_freq_type);
	}
	else
	{
		table_data[err_status_1_type] = (int32_t)status;
		table_data[err_current_1_type] = (int32_t)(current*(float)param_table[err_current_1_type].ratio);
		table_data[err_freq_1_type] = (int32_t)(freq*(float)param_table[err_freq_1_type].ratio);
	}

#if 1
	// update EEPROM
	if(!ERR_isNvmError())
	{
		nvm_status = NVMQ_enqueueNfcQ(err_code_1_type, table_data[err_code_1_type]);
		nvm_status = NVMQ_enqueueNfcQ(err_status_1_type, table_data[err_status_1_type]);
		nvm_status = NVMQ_enqueueNfcQ(err_current_1_type, table_data[err_current_1_type]);
		nvm_status = NVMQ_enqueueNfcQ(err_freq_1_type, table_data[err_freq_1_type]);
	}
#endif

	return 1;
}

int8_t table_updateCommError(uint16_t err_code)
{
	int i;
	uint8_t nvm_status;
	uint16_t index;
	int32_t err_status, err_current, err_freq;

	for(i=3; i>=0; i--) // push to next pos
	{
		//printf("i=%d\n", i);
		table_data[err_code_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_code_1_type + i*ERRINFO_ITEM_CNT]; // error code
		table_data[err_status_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_status_1_type + i*ERRINFO_ITEM_CNT]; // status, RUN/STOP/ACCEL/DECEL
		table_data[err_current_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_current_1_type + i*ERRINFO_ITEM_CNT]; // current
		table_data[err_freq_1_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_freq_1_type + i*ERRINFO_ITEM_CNT]; // freq

#if 1
		// update EEPROM
		index = err_code_1_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
		index = err_status_1_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
		index = err_current_1_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
		index = err_freq_1_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVMQ_enqueueNfcQ(index, table_data[index]);
#endif
	}

	// store latest status
	err_status = table_getStatusValue(run_status1_type)&0x0F;
	err_current = table_getStatusValue(I_rms_type);
	err_freq = table_getStatusValue(run_freq_type);

	// store latest error info
	table_data[err_code_1_type] = (int32_t)err_code;
	table_data[err_status_1_type] = (int32_t)err_status;
	table_data[err_current_1_type] = err_current;
	table_data[err_freq_1_type] = err_freq;

#if 1
	// update EEPROM
	nvm_status = NVMQ_enqueueNfcQ(err_code_1_type, table_data[err_code_1_type]);
	nvm_status = NVMQ_enqueueNfcQ(err_status_1_type, table_data[err_status_1_type]);
	nvm_status = NVMQ_enqueueNfcQ(err_current_1_type, table_data[err_current_1_type]);
	nvm_status = NVMQ_enqueueNfcQ(err_freq_1_type, table_data[err_freq_1_type]);
#endif

	return 1;
}

/*
 * 	NFC task -> mainHandler task to update table_data
 */
int8_t table_updatebyTableQ(void)
{
	int32_t value;
	int8_t empty, status=1;
	PARAM_IDX_t index;
	int16_t errflag=0;

	do {

		status = NVMQ_dequeueTableQ(&index, &value);
		if(status == 0) {kprintf(PORT_DEBUG,"ERROR TableQ dequeue \r\n"); errflag++;}

		if(table_data[index] != value)
		{
			status = table_runFunc(index, value, REQ_FROM_NFC);
			kprintf(PORT_DEBUG,"table_updatebyTableQ table_data[%d]=%d, status=%d \r\n", index, (int)value, status);
		}

		empty = NVMQ_isEmptyTableQ();
		osDelay(5);
	} while(empty == 0); // not empty

	if(errflag) return 0;

	//NVM_setCRC(); // force CRC

	return 1;
}

void table_initParam(void)
{
	PARAM_IDX_t idx;
	int index;
	int8_t status=0;
//	int32_t dir_domain=0;
//	uint16_t dummy[3] = {0,0,0};

	// initialize DIN for ext_trip or emergency_stop
	for(idx=multi_Din_0_type; idx<=multi_Din_2_type; idx++)
	{
		index = (int)(idx - multi_Din_0_type);
		EXT_DI_setupMultiFuncDin(index, (DIN_config_t)table_data[idx], REQ_FROM_TEST);
	}

	if(table_data[ctrl_in_type] == CTRL_IN_Digital)
		EXI_DI_getStepValue(EXT_DI_INITIALIZE);

	if(table_data[ctrl_in_type] == CTRL_IN_Analog_V	|| table_data[ctrl_in_type] == CTRL_IN_Din_Ain)
	{
		// initialize AIN
		EXT_AI_needReconfig();
		UTIL_startADC(); // start ADC
	}

	// set dir_domain_type
	param_table[dir_domain_type].param_func(dir_domain_type, table_data[dir_domain_type], REQ_FROM_TEST);
//	dir_domain = (int16_t)table_getValue(dir_domain_type);
//	if(dir_domain == DIR_REVERSE_ONLY) // if dir_domain_type = reverse only, then initialize DSP direction reverse
//	{
//		kprintf(PORT_DEBUG, "initial send SPICMD_CTRL_DIR_R\r\n");
//#ifndef SUPPORT_UNIT_TEST
//		// send run to DSP
//		status = COMM_sendMessage(SPICMD_CTRL_DIR_R, dummy);
//		if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO DIR R error! \r\n"); }
//#endif
//	}

	// set gear ratio 1 as default
	gear_ratio = (int16_t)table_getValue(gear_ratio_type);
	if(gear_ratio == 0 || gear_ratio > param_table[gear_ratio_type].maxValue)
	{
		gear_ratio = 1; // set default value for error value
		status = NVM_writeParam((PARAM_IDX_t)gear_ratio_type, gear_ratio);
		kprintf(PORT_DEBUG,"ERROR!! gear_ratio=%d status=%d \r\n", gear_ratio, status);
	}
}

int table_isDirectionValid(void)
{
	int32_t dir_control = table_getValue(dir_domain_type);
	int result = 1;

	switch(dir_control)
	{
	case DIR_FORWARD_ONLY:
		if(table_getValue(dir_cmd_type) == CMD_DIR_R)
			result = 0;
		break;

	case DIR_REVERSE_ONLY:
		if(table_getValue(dir_cmd_type) == CMD_DIR_F)
			result = 0;
		break;
	}

	return result;
}
