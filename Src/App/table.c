/*
 * table.c
 *
 *  Created on: 2019. 6. 3.
 *      Author: hrjung
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "table.h"
#include "crc32.h"
#include "drv_nvm.h"


#ifdef SUPPORT_NFC_OLD
#define ERRINFO_ITEM_CNT	5
#else
#define ERRINFO_ITEM_CNT	4
#endif


STATIC int32_t table_data[PARAM_TABLE_SIZE]; // parameter table of actual value
STATIC int32_t table_status[PARAM_STATUS_SIZE]; // status from DSP via mailbox



static int32_t sysparam_addr[] =
{
		0x10,		//SYSTEM_PARAM_NFC_TAGGED
		0x14,		//SYSTEM_PARAM_CRC_VALUE
		0x18,		//SYSTEM_PARAM_IS_INITIATED
		0x1C,		//SYSTEM_PARAM_HAS_SYSTEM_ERROR
		0x20,		//SYSTEM_PARAM_ENABLE_NFC_WRITER
		0x24,		//SYSTEM_PARAM_NFC_TRYED
		0x28,		//SYSTEM_PARAM_ON_MONITORING
		0x2C,		//SYSTEM_PARAM_IDLE0_RUN1_STOP2
};

int8_t table_setValue(PARAM_IDX_t idx, int32_t value);
STATIC int8_t table_setAinFreqMinValue(PARAM_IDX_t idx, int32_t value);
static int8_t table_setValueMin(PARAM_IDX_t idx, int32_t value);
static int8_t table_setValueMax(PARAM_IDX_t idx, int32_t value);
static int8_t table_setValueDir(PARAM_IDX_t idx, int32_t value);
static int8_t table_setErrInfo(PARAM_IDX_t idx, int32_t value);


static Param_t param_table[] =
{   	//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx		param_func
		{	value_type,			0x100,	40100,	200,	10,		2000,	1, 	10,		1, 	value_dsp,		table_setValue, },
		{	multi_val_0_type,	0x104,	40101,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_1_type,	0x108,	40102,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_2_type,	0x10C,	40103,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_3_type,	0x110,	40104,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_4_type,	0x114,	40105,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_5_type,	0x118,	40106,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_6_type,	0x11C,	40107,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	multi_val_7_type,	0x120,	40108,	200,	10,		2000,	1, 	10, 	1, 	none_dsp,		table_setValue, },
		{	freq_min_type,		0x124,	40109,	10,		10,		2000,	1, 	10, 	0, 	none_dsp,		table_setValueMin, },
		{	freq_max_type,		0x128,	41010,	600,	10,		2000,	1, 	10, 	0, 	freq_max_dsp,	table_setValueMax,},
		{	accel_time_type,	0x12C,	40111,	100,	10,		6000,	1, 	10, 	1, 	accel_time_dsp,	table_setValue,},
		{	decel_time_type,	0x130,	40112,	100,	10,		6000,	1, 	10,		1, 	decel_time_dsp,	table_setValue,},
		{	dir_cmd_type,		0x134,	40113,	0,		0,		1,		1, 	1, 		1, 	dir_cmd_dsp,	table_setValue,},
		{	jmp_enable0_type,	0x138,	40114,	0,		0,		1,		1, 	1, 		0, 	jmp_enable0_dsp,table_setValue,	},
		{	jmp_enable1_type,	0x13C,	40115,	0,		0,		1,		1, 	1, 		0, 	jmp_enable1_dsp,table_setValue,	},
		{	jmp_enable2_type,	0x140,	40116,	0,		0,		1,		1, 	1, 		0, 	jmp_enable2_dsp,table_setValue,	},
		{	jmp_low0_type,		0x144,	40117,	10,		10,		2000,	1, 	10, 	0, 	jmp_low0_dsp, 	table_setValue,},
		{	jmp_low1_type,		0x148,	40118,	10,		10,		2000,	1, 	10, 	0, 	jmp_low1_dsp, 	table_setValue,},
		{	jmp_low2_type,		0x14C,	40119,	10,		10,		2000,	1, 	10, 	0, 	jmp_low2_dsp, 	table_setValue,},
		{	jmp_high0_type,		0x150,	40120,	10,		10,		2000,	1, 	10, 	0, 	jmp_high0_dsp, 	table_setValue,},
		{	jmp_high1_type,		0x154,	40121,	10,		10,		2000,	1, 	10, 	0, 	jmp_high1_dsp, 	table_setValue,},
		{	jmp_high2_type,		0x158,	40122,	10,		10,		2000,	1, 	10, 	0, 	jmp_high2_dsp, 	table_setValue,},
		{	dir_domain_type,	0x15C,	40123,	0,		0,		2,		1, 	1, 		0, 	none_dsp,		table_setValueDir,},
		{	acc_base_set_type,	0x160,	40124,	0,		0,		1,		1, 	1, 		0, 	acc_base_set_dsp, table_setValue,	},


		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx			param_func
		{	ctrl_in_type,		0x200,	40200,	0,		0,		4,		1, 	1, 		0, 	none_dsp,			table_setValue},
		{	energy_save_type,	0x204,	40202,	0,		0,		1,		1, 	1, 		0, 	energy_save_dsp,	table_setValue	},
		{	pwm_freq_type,		0x208,	40203,	0,		0,		3,		1, 	1, 		0, 	pwm_freq_dsp,		table_setValue	},
		{	brake_type_type,	0x20C,	40206,	0,		0,		2,		1, 	1, 		0, 	brake_type_dsp,		table_setValue	},
		{	brake_freq_type,	0x210,	40207,	10,		1,		600,	1, 	10, 	0, 	brake_freq_dsp,		table_setValue	},
		{	dci_brk_freq_type,	0x214,	40208,	30,		1,		600,	1, 	10, 	0, 	dci_brk_freq_dsp,	table_setValue	},
		{	dci_brk_hold_type,	0x218,	40209,	10,		0,		600,	1,	10, 	0, 	dci_brk_hold_dsp,	table_setValue	},
		{	dci_brk_time_type,	0x21C,	40210,	50,		0,		600,	1, 	10, 	0, 	dci_brk_time_dsp,	table_setValue	},
		{	dci_brk_rate_type,	0x220,	40211,	500,	0,		2000,	1, 	10, 	0, 	dci_brk_rate_dsp,	table_setValue	},

		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx			param_func
		{	ovl_warn_limit_type,0x280,	40280,	150,	100,	200,	1,	1, 		1, 	ovl_warn_limit_dsp,	table_setValue	},
		{	ovl_warn_dur_type,	0x284,	40281,	10,		0,		30,		1,	1, 		1, 	ovl_warn_dur_dsp,	table_setValue	},
		{	ovl_enable_type,	0x288,	40282,	1,		0,		1,		1,	1, 		1, 	ovl_enable_dsp,		table_setValue	},
		{	ovl_trip_limit_type,0x28C,	40283,	180,	100,	200,	1,	1,		1, 	ovl_trip_limit_dsp, table_setValue	},
		{	ovl_trip_dur_type,	0x290,	40284,	30,		0,		60,		1,	1, 		1, 	ovl_trip_dur_dsp,	table_setValue	},
		{	regen_duty_type,	0x294,	40285,	30,		0,		80,		1,	1, 		0, 	regen_duty_dsp,		table_setValue	},
		{	regen_band_type,	0x298,	40286,	10,		0,		80,		1,	1, 		0, 	regen_band_dsp,		table_setValue	},
		{	fan_onoff_type,		0x29C,	40287,	0,		0,		1,		1,	1, 		1, 	fan_onoff_dsp,		table_setValue	},

		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, dsp_idx		param_func
		{	multi_Din_0_type,	0x300,	40300,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	multi_Din_1_type,	0x304,	40301,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	multi_Din_2_type,	0x308,	40302,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	multi_Din_3_type,	0x30C,	40303,	0,		0,		8,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	multi_Dout_0_type,	0x310,	40304,	0,		0,		5,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	multi_Dout_1_type,	0x314,	40305,	0,		0,		5,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	v_in_min_type,		0x318,	40306,	0,		0,		100,	1,	10, 	1, 	none_dsp,		table_setValue	},
		{	v_in_min_freq_type,	0x31C,	40307,	0,		0,		2000,	1,	10, 	1, 	none_dsp,		table_setAinFreqMinValue	},
		{	v_in_max_type,		0x320,	40308,	100,	0,		100,	1,	10, 	1, 	none_dsp,		table_setValue	},
		{	v_in_max_freq_type,	0x324,	40309,	2000,	10,		2000,	1,	10, 	1, 	none_dsp,		table_setValue	},
		{	aout_type_type,		0x328,	40310,	0,		0,		3,		1,	1, 		1, 	none_dsp,		table_setValue	},
		{	aout_rate_type,		0x32C,	40311,	100,	10,		200,	1,	1, 		1, 	none_dsp,		table_setValue	},
		{	mb_address_type,	0x330,	40312,	1,		1,		254,	1,	1, 		1, 	none_dsp,		table_setValue	},
		{	baudrate_type,		0x334,	40313,	2,		0,		5,		1,	1, 		1, 	none_dsp,		table_setValue	},


		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, DSPcomm
		{	Rs_type,			0x20,	40040,	0,		0,		0,		0, 	10, 	0, 	none_dsp,	},
		{	Rr_type,			0x24,	40041,	0,		0,		0,		0, 	10, 	0, 	none_dsp,	},
		{	Ls_type,			0x28,	40042,	0,		0,		0,		0, 	10, 	0,	none_dsp,	},
		{	noload_current_type,0x2C,	40043,	0,		0,		0,		0, 	10, 	0, 	none_dsp,	},
		{	rated_current_type,	0x30,	40044,	0,		0,		0,		0, 	10, 	0, 	none_dsp,	},
		{	poles_type,			0x34,	40045,	0,		0,		0,		0, 	1, 		0, 	none_dsp,	},
		{	input_voltage_type,	0x38,	40046,	0,		0,		0,		0, 	1, 		0, 	none_dsp,	},
		{	rated_freq_type,	0x3C,	40047,	0,		0,		0,		0, 	1, 		0, 	none_dsp,	},

		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, DSPcomm
		{	model_type,			0x60,	40080,	0,		0,		0,		0,	1, 		0, 	none_dsp,	},
		{	motor_type_type,	0x64,	40081,	0,		0,		0,		0,	1,		0, 	none_dsp,	},
		{	gear_ratio_type,	0x68,	40082,	0,		0,		0,		0,	1, 		0, 	none_dsp,	},
		{	motor_on_cnt_type,	0x6C,	40083,	0,		0,		0,		0,	1, 		0, 	none_dsp,	},
		{	elapsed_hour_type,	0x70,	40084,	0,		0,		0,		0,	1, 		0, 	none_dsp,	},
		{	operating_hour_type,0x74,	40085,	0,		0,		0,		0,	1, 		0, 	none_dsp,	},


		//    idx,				addr,	modbus, init,	min,	max,	RW,ratio,WRonRun, DSPcomm
		{	err_date_0_type,	0x380,	40400,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_code_0_type,	0x384,	40401,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_status_0_type,	0x388,	40402,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_current_0_type,	0x38C,	40403,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},
		{	err_freq_0_type,	0x390,	40404,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},

		{	err_date_1_type,	0x394,	40405,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_code_1_type,	0x398,	40406,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_status_1_type,	0x39C,	40407,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_current_1_type,	0x3A0,	40408,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},
		{	err_freq_1_type,	0x3A4,	40409,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},

		{	err_date_2_type,	0x3A8,	40410,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_code_2_type,	0x3AC,	40411,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_status_2_type,	0x3B0,	40412,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_current_2_type,	0x3B4,	40413,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},
		{	err_freq_2_type,	0x3B8,	40414,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},

		{	err_date_3_type,	0x3BC,	40415,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_code_3_type,	0x3C0,	40416,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_status_3_type,	0x3C4,	40417,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_current_3_type,	0x3C8,	40418,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},
		{	err_freq_3_type,	0x3CC,	40419,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},

		{	err_date_4_type,	0x3D0,	40420,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_code_4_type,	0x3D4,	40421,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_status_4_type,	0x3D8,	40422,	0,		0,		0,		0, 	1, 		0, 	none_dsp,		table_setErrInfo},
		{	err_current_4_type,	0x3DC,	40423,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},
		{	err_freq_4_type,	0x3E0,	40424,	0,		0,		0,		0, 	10, 	0, 	none_dsp,		table_setErrInfo},


//		{	run_status1_type,	0,	int_type,	40,	0,	0,	0,	0,	0		},
//		{	run_status2_type,	0,	int_type,	41,	0,	0,	0,	0,	0		},
//		{	I_rms_type,	0,	float_type,	42,	0,	0,	0,	0,	0		},
//		{	run_freq_type,	0,	float_type,	43,	0,	0,	0,	0,	0		},
//		{	dc_voltage_type,	0,	float_type,	44,	0,	0,	0,	0,	0		},
//		{	ipm_temperature_type,	0,	float_type,	45,	0,	0,	0,	0,	0		},
//		{	motor_temperature_type,	0,	int_type,	46,	0,	0,	0,	0,	0		},
};


/*
 * 		param_func implementation
 */
int8_t table_setValue(PARAM_IDX_t idx, int32_t value)
{
	if(value < param_table[idx].minValue || value > param_table[idx].maxValue)
	{
		printf("idx=%d value=%d is not valid\n", idx, value);
		return 0; //ignore
	}

	table_data[idx] = value;
	//printf("idx=%d set value=%d\n", idx, value);

	return 1;
}

int8_t table_setAinFreqMinValue(PARAM_IDX_t idx, int32_t value)
{
	if( (value < param_table[idx].minValue && value != 0)
	   || value > param_table[idx].maxValue)
	{
		printf("idx=%d value=%d is not valid\n", idx, value);
		return 0; //ignore
	}

	table_data[idx] = value;
	//printf("idx=%d set value=%d\n", idx, value);

	return 1;
}

int8_t table_setValueMin(PARAM_IDX_t idx, int32_t value)
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

	result = table_setValue(idx, value);
	if(result)
	{
		for(i=0; i<range_size; i++)
		{
			param_table[r_idx[i]].minValue = value;
		}
	}

	return result;
}

int8_t table_setValueMax(PARAM_IDX_t idx, int32_t value)
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

	result = table_setValue(idx, value);
	if(result)
	{
		for(i=0; i<range_size; i++)
		{

			param_table[r_idx[i]].maxValue = value;
		}
	}

	return result;
}

int8_t table_setValueDir(PARAM_IDX_t idx, int32_t value)
{
	int8_t result;

	result = table_setValue(idx, value);
	if(result)
	{
		// update dir_cmd_type valid range
		if(value == 1)
		{
			param_table[dir_cmd_type].minValue = 0;
			param_table[dir_cmd_type].maxValue = 0;
		}
		else if(value == 2)
		{
			param_table[dir_cmd_type].minValue = 1;
			param_table[dir_cmd_type].maxValue = 1;
		}
		else
		{
			param_table[dir_cmd_type].minValue = 0;
			param_table[dir_cmd_type].maxValue = 1;
		}
	}

	return result;
}

int8_t table_setErrInfo(PARAM_IDX_t idx, int32_t value)
{
	// TODO : need implement

	return 1;
}


/*
 * 		table handling function implementation
 */
void table_setInit(void)
{
	NVM_write(sysparam_addr[SYSTEM_PARAM_IS_INITIATED], (int32_t)1);
}

uint32_t table_calcCRC(void)
{
	int table_length = (int)err_code_0_type;
	uint8_t* buff = (uint8_t *)table_data;
	uint32_t crc32 = update_crc(-1, buff, table_length *sizeof(uint32_t));

	return crc32;
}

int8_t table_verifyCRC(uint32_t crc32_calc)
{
	uint8_t status;
	uint32_t crc32_rd;

	status = NVM_read(sysparam_addr[SYSTEM_PARAM_CRC_VALUE], (int32_t *)&crc32_rd);
	if(status != 0) return 0;

	if(crc32_calc != crc32_rd) return 0;

	return 1;
}

int8_t table_setCRC(void)
{
	uint32_t crc32_calc;
	uint8_t status;

	crc32_calc = table_calcCRC();
	status = NVM_write(sysparam_addr[SYSTEM_PARAM_CRC_VALUE], crc32_calc);
	if(status == 0) return 1;

	return 0;
}

int8_t table_initializeEEPROM(void)
{
	int i;

	NVM_clear();
	printf("initialize Table...");
	for(i=0; i<PARAM_TABLE_SIZE; i++)
	{
		NVM_write(param_table[i].addr, param_table[i].initValue);
		table_data[i] = param_table[i].initValue;
	}
	table_setInit();
	table_setCRC();

	return 1;
}

int8_t table_isInit(void)
{
	int32_t isInit;
	uint8_t status;

	status = NVM_read(sysparam_addr[SYSTEM_PARAM_IS_INITIATED], &isInit);
	if(status!=0) return -1;

	return (int8_t)isInit;
}

int8_t table_loadEEPROM(void)
{
	int i;
	uint8_t status;
	int32_t value;
	uint32_t crc32_calc;

	for(i=0; i<PARAM_TABLE_SIZE; i++)
	{
		status = NVM_read(param_table[i].addr, &value);
		if(status != 0)
		{
			printf("idx=%d read error, value=%d\n", i, value);
			return 0;
		}
		else
			table_data[i] = value;
	}

	// verify EEPROM, check CRC
	crc32_calc = table_calcCRC();
	status = table_verifyCRC(crc32_calc);

	return 1;
}

int8_t table_verifyRange(void)
{
	int8_t result=0;
	int i, range_size=7;
	int32_t value;
	PARAM_IDX_t idx[] = {
			freq_min_type,
			freq_max_type,
			dir_domain_type,
			v_in_max_freq_type,
			v_in_min_freq_type,

			v_in_max_type,
			v_in_min_type,
	};
	PARAM_IDX_t curIdx;

	for(i=0; i<range_size; i++)
	{
		curIdx = idx[i];
		value = table_data[curIdx];
		if(value < param_table[curIdx].minValue || value > param_table[curIdx].maxValue)
		{
			printf("range idx=%d value = %d, error! \n",curIdx, value);
			return 0;
		}
		else
		{
			result = (int8_t)(param_table[curIdx].param_func)(curIdx, value);
		}
	}

	return result;
}

int8_t table_init(void)
{
	int8_t status;
	int i;

	status = table_verifyRange();
	if(status == 0) return 0;

	// init table_status[]
	for(i=0; i<PARAM_STATUS_SIZE; i++)
		table_status[i] = (int32_t)0;

	for(i=0; i<PARAM_TABLE_SIZE; i++)
	{
		if(table_data[i] < param_table[i].minValue || table_data[i] > param_table[i].maxValue)
		{
			printf(" idx=%d value = %d, range error! \n",i, table_data[i]);
			return 0;
		}

		// send to DSP
		if(table_data[i] != param_table[i].initValue && (param_table[i].dsp_idx != none_dsp) )
		{
			printf("(idx:value)(%d:%d) need to send DSP\n", i, table_data[i]);
			// TODO: send DSP

		}
	}

	return 1;
}

// return 0 for error
// return 1 for int value
//        10 for float value
uint8_t table_getRatio(PARAM_IDX_t index)
{
	if(index < 0 || index >= PARAM_TABLE_SIZE) { printf("parameter index error %d", index); return 1; }

	return param_table[index].ratio;
}

int32_t table_getValue(PARAM_IDX_t index)
{
	return table_data[index];
}

int32_t table_getCtrllIn(void)
{
	return table_data[ctrl_in_type];
}

int32_t table_getStatusValue(int16_t index)
{
	return table_status[index];
}

void table_setStatusValue(int16_t index, int32_t value)
{
	table_status[index] = value;
}

int8_t table_updateErrorDSP(uint16_t err_code, uint16_t status, float current, float freq)
{
	int i;
//	uint8_t nvm_status;
//	int32_t addr;
#ifdef SUPPORT_NFC_OLD
	int32_t date=0; //TODO : get date info
#endif

	for(i=3; i>=0; i--) // push to next pos
	{
		//printf("i=%d\n", i);
#ifdef SUPPORT_NFC_OLD
		table_data[err_date_0_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_date_0_type + i*ERRINFO_ITEM_CNT]; // date
#endif
		table_data[err_code_0_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_code_0_type + i*ERRINFO_ITEM_CNT]; // error code
		table_data[err_status_0_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_status_0_type + i*ERRINFO_ITEM_CNT]; // status, RUN/STOP/ACCEL/DECEL
		table_data[err_current_0_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_current_0_type + i*ERRINFO_ITEM_CNT]; // current
		table_data[err_freq_0_type + (i+1)*ERRINFO_ITEM_CNT] = table_data[err_freq_0_type + i*ERRINFO_ITEM_CNT]; // freq

#if 0
		// update EEPROM
		addr = err_date_0_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVM_write(addr, table_data[addr]);
		addr = err_code_0_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVM_write(addr, table_data[addr]);
		addr = err_status_0_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVM_write(addr, table_data[addr]);
		addr = err_current_0_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVM_write(addr, table_data[addr]);
		addr = err_freq_0_type + (i+1)*ERRINFO_ITEM_CNT;
		nvm_status = NVM_write(addr, table_data[addr]);
#endif
	}

	// store latest error info
#ifdef SUPPORT_NFC_OLD
	table_data[err_date_0_type] = (int32_t)date;
#endif
	table_data[err_code_0_type] = (int32_t)err_code;
	table_data[err_status_0_type] = (int32_t)status;
	table_data[err_current_0_type] = (int32_t)(current*(float)param_table[err_current_0_type].ratio);
	table_data[err_freq_0_type] = (int32_t)(freq*(float)param_table[err_freq_0_type].ratio);

#if 0
	// update EEPROM
#ifdef SUPPORT_NFC_OLD
	nvm_status = NVM_write(err_date_0_type, table_data[err_date_0_type]);
#endif
	nvm_status = NVM_write(err_code_0_type, table_data[err_code_0_type]);
	nvm_status = NVM_write(err_status_0_type, table_data[err_status_0_type]);
	nvm_status = NVM_write(err_current_0_type, table_data[err_current_0_type]);
	nvm_status = NVM_write(err_freq_0_type, table_data[err_freq_0_type]);
#endif

	return 1;
}


