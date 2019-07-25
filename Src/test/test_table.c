/*
 * test_table.c
 *
 *  Created on: 2019. 6. 29.
 *      Author: hrjung
 */

#include "includes.h"

#ifdef SUPPORT_UNIT_TEST


#include <memory.h>

#include "unity.h"
#include "table.h"
#include "dsp_comm.h"


extern int16_t state_run_stop;
extern Param_t param_table[];
extern COMM_CMD_t test_cmd;

extern void test_setTableValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern TABLE_DSP_PARAM_t table_getDspAddr(PARAM_IDX_t index);
extern int8_t table_runFunc(PARAM_IDX_t idx, int32_t value, int16_t opt);
extern int8_t table_setStatusValue(PARAM_IDX_t index, int32_t value, int16_t option);



void test_initTable(void)
{
	PARAM_IDX_t index;

	for(index=0; index<baudrate_type+1; index++)
	{
		test_setTableValue(index, param_table[index].initValue, REQ_FROM_TEST);
	}
}

/*
 *		test item : table_setValueMin, table_setValueMax
 *
 *			1. set normal value, set OK
 *			2. set out of range value, get NOK
 *			3. set freq value of out of range, get NOK
 *			4. for Min value, set freq 0.0Hz, set OK
 *			5. cannot set freq_max on running, get NOK
 */
void test_setFreqRange(void)
{
	int i;
	int32_t exp_freq, freq, value;
	PARAM_IDX_t index = value_type;
	int8_t status, exp_status;
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

	table_setStatusValue(run_status1_type, 1, REQ_FROM_TEST); // STOP

	index = freq_min_type;
	freq = 50;
	exp_freq = 50;
	exp_status = 1;
	status = table_runFunc(index, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);
	for(i=0; i<17; i++)
		TEST_ASSERT_EQUAL_INT(exp_freq, param_table[r_idx[i]].minValue);

	// range error, no change
	freq = 5;
	exp_freq = 50;
	exp_status = 0;
	status = table_runFunc(index, freq, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);

	// cannot set freq under freq_min
	freq = 30;
	exp_freq = table_getValue(value_type);
	exp_status = 0;
	status = table_runFunc(value_type, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(value_type);
	TEST_ASSERT_EQUAL_INT(exp_freq, value); // not changed

	// set freq=0.0 is OK
	freq = 0;
	exp_freq = table_getValue(value_type);
	exp_status = 1;
	status = table_runFunc(value_type, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);


	index = freq_max_type;
	freq = 800;
	exp_freq = 800;
	exp_status = 1;
	status = table_runFunc(index, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);
	for(i=0; i<17; i++)
		TEST_ASSERT_EQUAL_INT(exp_freq, param_table[r_idx[i]].maxValue);

	// range error, no change
	freq = 2050;
	exp_freq = 800;
	exp_status = 0;
	status = table_runFunc(index, freq, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);

	// cannot set freq over freq_max
	freq = 850;
	exp_freq = table_getValue(value_type);
	exp_status = 0;
	status = table_runFunc(value_type, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(value_type);
	TEST_ASSERT_EQUAL_INT(exp_freq, value); // not changed

	// cannot set on running
	state_run_stop = 1; // RUN
	freq = 600;
	exp_freq = table_getValue(index);
	exp_status = 0;
	status = table_runFunc(index, freq, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);
}


/*
 *		test item : table_setFreqValue
 *
 *			1. set normal value, set OK
 *			2. set 0.0Hz, set OK
 *			3. set value of out of range, get NOK, no change
 *			4. set normal value in run state, set OK
 *			5. set normal value in run state, get NOK for WRonRun=0 parameter
 */
void test_setFreqValue(void)
{
	int32_t exp_freq, freq_l, value;
	PARAM_IDX_t index = value_type;
	int8_t status, exp_status;

	table_setStatusValue(run_status1_type, 1, REQ_FROM_TEST); // STOP
	table_runFunc(freq_max_type, 600, REQ_FROM_TEST);
	table_runFunc(freq_min_type, 10, REQ_FROM_TEST);

	// set OK
	index = value_type;
	freq_l = 300;
	exp_freq = 300;
	exp_status = 1;
	status = table_runFunc(index, freq_l, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);

	// set freq 0.0, OK
	freq_l = 0;
	exp_freq = 0;
	exp_status = 1;
	status = table_runFunc(index, freq_l, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);

	// range error
	freq_l = 800;
	exp_freq = table_getValue(index);
	exp_status = 0;
	status = table_runFunc(index, freq_l, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);


	// set value in run state OK
	table_setStatusValue(run_status1_type, 4, REQ_FROM_TEST); // RUN

	freq_l = 400;
	exp_freq = 400;
	exp_status = 1;
	status = table_runFunc(index, freq_l, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value);

	// set jump freq value in run state NOK
	index = jmp_low1_type;
	freq_l = 300;
	exp_freq = table_getValue(index);;
	exp_status = 0;
	status = table_runFunc(index, freq_l, REQ_FROM_TEST);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	TEST_ASSERT_EQUAL_INT(exp_freq, value); // no change


}

/*
 *		test item : table_setValue
 *
 *			1. set normal value, set OK
 *			2. set value of out of range, get NOK, no change
 *			3. set normal value in run state, set OK
 *			4. try to set read only parameter, get NOK
 *			5. set normal value in run state, get NOK for WRonRun=0 parameter
 */
void test_setValue(void)
{
	int32_t exp_val, set_val, value;
	PARAM_IDX_t index;
	int8_t status, exp_status;

	table_setStatusValue(run_status1_type, 1, REQ_FROM_TEST); // STOP

	// set accel normal value, OK
	index = accel_time_type;
	set_val = 300;
	exp_val = 300;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 6050;
	exp_val = 300;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = 5;
	exp_val = 300;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set dir cmd normal value, OK
	index = dir_cmd_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set jmp enable normal value, OK
	index = jmp_enable1_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set acc_base normal value, OK
	index = acc_base_set_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set ctrl_in normal value, OK
	index = ctrl_in_type;
	set_val = CTRL_IN_Modbus;
	exp_val = CTRL_IN_Modbus;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = CTRL_IN_MAX;
	exp_val = CTRL_IN_Modbus;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = CTRL_IN_NFC-1;
	exp_val = CTRL_IN_Modbus;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set energy save normal value, OK
	index = energy_save_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set pwm_freq normal value, OK
	index = pwm_freq_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 4;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set brake_type normal value, OK
	index = brake_type_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 3;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set dci_brake_hold normal value, OK
	index = dci_brk_hold_type;
	set_val = 20;
	exp_val = 20;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 605;
	exp_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set dci_brake_rate normal value, OK
	index = dci_brk_rate_type;
	set_val = 1000;
	exp_val = 1000;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2005;
	exp_val = 1000;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1000;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set dci_brake_rate normal value, OK
	index = dci_brk_rate_type;
	set_val = 1000;
	exp_val = 1000;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2005;
	exp_val = 1000;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1000;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set ovl_warn_limit normal value, OK
	index = ovl_warn_limit_type;
	set_val = 150;
	exp_val = 150;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 205;
	exp_val = 150;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = 95;
	exp_val = 150;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set ovl_trip_dur normal value, OK
	index = ovl_trip_dur_type;
	set_val = 10;
	exp_val = 10;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 61;
	exp_val = 10;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 10;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set regen_duty normal value, OK
	index = regen_duty_type;
	set_val = 50;
	exp_val = 50;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 81;
	exp_val = 50;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 50;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set fan normal value, OK
	index = fan_onoff_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set fan normal value, OK
	index = fan_onoff_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 2;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set Dout value, OK
	index = multi_Dout_0_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 6;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// set Dout value, OK
	index = multi_Dout_0_type;
	set_val = 1;
	exp_val = 1;
	exp_status = 1;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value);

	// set out of range value, NOK
	set_val = 6;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change

	// set out of range value, NOK
	set_val = -1;
	exp_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);
	value = table_getValue(index);
	TEST_ASSERT_EQUAL_INT(exp_val, value); // no change


	// try set value of read only parameter, NOK
	index = noload_current_type;
	set_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = gear_ratio_type;
	set_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = err_current_0_type;
	set_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);


	test_initTable(); // initialize all table value

	// test wr on running
	table_setStatusValue(run_status1_type, 4, REQ_FROM_TEST); // RUN

	index = jmp_enable2_type;
	set_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = acc_base_set_type;
	set_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = energy_save_type;
	set_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = brake_type_type;
	set_val = 1;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = dci_brk_hold_type;
	set_val = 100;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = dci_brk_time_type;
	set_val = 100;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = dci_brk_rate_type;
	set_val = 700;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = regen_duty_type;
	set_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

	index = regen_band_type;
	set_val = 20;
	exp_status = 0;
	status = table_runFunc(index, set_val, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_status, status);

}

void test_NvmAddr(void)
{
	PARAM_IDX_t index;
	uint16_t exp_addr;

	exp_addr = param_table[value_type].addr;
	for(index=value_type; index<=acc_base_set_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[ctrl_in_type].addr;
	for(index=ctrl_in_type; index<=dci_brk_rate_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[ovl_warn_limit_type].addr;
	for(index=ovl_warn_limit_type; index<=fan_onoff_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[multi_Din_0_type].addr;
	for(index=multi_Din_0_type; index<=baudrate_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[Rs_type].addr;
	for(index=Rs_type; index<=rated_freq_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[model_type].addr;
	for(index=model_type; index<=operating_hour_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[err_date_0_type].addr;
	for(index=err_date_0_type; index<=err_freq_4_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}

	exp_addr = param_table[run_status1_type].addr;
	for(index=run_status1_type; index<=mtr_temperature_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].addr);
		exp_addr += 4;
	}
}

void test_ModbusAddr(void)
{
	PARAM_IDX_t index;
	uint16_t exp_addr;

	exp_addr = param_table[value_type].mb_addr;
	for(index=value_type; index<=acc_base_set_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[ctrl_in_type].mb_addr;
	for(index=ctrl_in_type; index<=dci_brk_rate_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[ovl_warn_limit_type].mb_addr;
	for(index=ovl_warn_limit_type; index<=fan_onoff_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[multi_Din_0_type].mb_addr;
	for(index=multi_Din_0_type; index<=baudrate_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[Rs_type].mb_addr;
	for(index=Rs_type; index<=rated_freq_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[model_type].mb_addr;
	for(index=model_type; index<=operating_hour_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[err_date_0_type].mb_addr;
	for(index=err_date_0_type; index<=err_freq_4_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}

	exp_addr = param_table[run_status1_type].mb_addr;
	for(index=run_status1_type; index<=mtr_temperature_type; index++)
	{
		TEST_ASSERT_EQUAL_INT(exp_addr, param_table[index].mb_addr);
		exp_addr++;
	}
}
#endif
