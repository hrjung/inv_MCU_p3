/*
 * test_ext_di.c
 *
 *  Created on: 2019. 6. 17.
 *      Author: hrjung
 */

#include "includes.h"

#ifdef SUPPORT_UNIT_TEST

#include <memory.h>

#include "unity.h"
#include "table.h"
#include "dsp_comm.h"

#include "ext_io.h"



extern int32_t table_data[];

extern int16_t state_run_stop;
extern int16_t state_direction;
extern int16_t st_overload;
extern int16_t st_brake;

extern uint8_t mdin_value[];
extern DIN_PIN_NUM_t m_din;
extern COMM_CMD_t test_cmd;
extern uint8_t step_cmd;

extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value);
extern void table_setStatusValue(int16_t index, int32_t value);


void test_clear(void)
{
	mdin_value[0] = 0;
	mdin_value[1] = 0;
	mdin_value[2] = 0;

	state_run_stop = 0;
	state_direction = 0;

	st_overload = 0;
	st_brake = 0;

	table_data[multi_Din_0_type] = DIN_unuse;
	table_data[multi_Din_1_type] = DIN_unuse;
	table_data[multi_Din_2_type] = DIN_unuse;
	table_data[multi_Din_3_type] = DIN_unuse;

	m_din.bit_H = EXT_DIN_COUNT;
	m_din.bit_M = EXT_DIN_COUNT;
	m_din.bit_L = EXT_DIN_COUNT;
	m_din.run_pin = EXT_DIN_COUNT;
	m_din.dir_pin = EXT_DIN_COUNT;
	m_din.emergency_pin = EXT_DIN_COUNT;
	m_din.trip_pin = EXT_DIN_COUNT;

}


/*
 * 		test item : EXT_DI_setupMultiFuncDin
 *
 *		1. set valid func_set at index=0, verify pin_num of func_set is OK
 * 		2. set valid func_set at index=1, verify pin_num of func_set is OK
 * 		3. set valid func_set at index=2, verify pin_num of func_set is OK
 * 		4. set valid func_set at index=3, error
 * 		5. set invaild func_setm error
 * 		6. update valid func_set at index=0, correctly updated, prev pin_num of func is cleared
 * 		7. update valid func_set at index=1, correctly updated, prev pin_num of func is cleared
 * 		8. update same func_set at index=2, correctly updated, prev table_data is also cleared
 * 		9. update same func_set at index=1, correctly updated, prev table_data is also cleared
 *
 */
void test_setupMultiFuncDin(void)
{
	uint8_t index;
	DIN_config_t func_set;
	uint8_t exp_pin;
	int result, exp_result;


	// set pin 0 as direction
	index = 0;
	func_set = DIN_direction;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.dir_pin);

	// set pin 1 as direction
	index = 1;
	func_set = DIN_emergency_stop;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.emergency_pin);

	// set pin 2 as freq_L
	index = 2;
	func_set = DIN_freq_low;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.bit_L);

	// out of bit range : ERROR
	index = 3;
	func_set = DIN_direction;
	exp_pin = 0; // dir_pin in 0
	exp_result = 0;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.dir_pin); // no change in dir_pin

	// wrong func_set : ERROR
	index = 1;
	func_set = DIN_config_max;
	exp_pin = 1; // dir_pin in 0
	exp_result = 0;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.emergency_pin); // no change in emergency_pin

	// update other pin 0
	index = 0;
	func_set = DIN_freq_mid;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.bit_M);
	TEST_ASSERT_EQUAL_INT(EXT_DIN_COUNT, m_din.dir_pin); //dir_pin cleared

	index = 1;
	func_set = DIN_run;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.run_pin);
	TEST_ASSERT_EQUAL_INT(EXT_DIN_COUNT, m_din.emergency_pin); // cleared

	// update pin 2 as run
	index = 2;
	func_set = DIN_run;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.run_pin); // update run_pin only
	TEST_ASSERT_EQUAL_INT(DIN_unuse, table_data[multi_Din_0_type+1]); // clear old table_data[+1]

	// update other pin 0 -> 1, clear pin 0
	index = 1;
	func_set = DIN_freq_mid;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.bit_M);
	TEST_ASSERT_EQUAL_INT(DIN_unuse, table_data[multi_Din_0_type+0]); // clear old table_data[+0]
}


/*
 * 		test item : EXT_DI_convertMultiStep
 *
 *		1. set freq_low at index=0, convert step to be 0, 1
 * 		2. set freq_mid at index=0, convert step to be 0, 2
 * 		3. set freq_high at index=0, convert step to be 0, 4
 * 		4. set freq_low, freq_mid at index=0,1,  convert step to be 0, 1, 2, 3
 * 		5. set freq_mid, freq_high at index=1,0, convert step to be 0, 2, 4, 6
 * 		6. set freq_low, freq_high at index=1,0, convert step to be 0, 1, 4, 5
 * 		7. set freq_low, freq_mid, freq_high at index=0, 1, 2, convert step to be 0, 1, 2, 3, 4, 5, 6, 7
 * 		8. set freq_low, freq_mid, freq_high at index=0, 1, 2, convert step to be 0, 1, 2, 3, 4, 5, 6, 7
 *
 */
void test_convertMultiStep(void)
{
	uint8_t l_index, m_index, h_index;
	DIN_config_t func_set;
	int result;
	uint8_t step, exp_step;

	test_clear();

	// use only 1 bit : low
	l_index = 0;
	func_set = DIN_freq_low;
	mdin_value[l_index] = 0;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(l_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use only 1 bit : mid
	m_index = 0;
	func_set = DIN_freq_mid;
	mdin_value[m_index] = 0;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(m_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = 1;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use only 1 bit : high
	h_index = 0;
	func_set = DIN_freq_high;
	mdin_value[h_index] = 0;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(h_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);


	test_clear();

	// use 2 bit : low + mid
	l_index = 0;
	m_index = 1;
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 2 bit : mid + high
	h_index = 0;
	m_index = 1;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 2 bit : low + high
	l_index = 0;
	h_index = 1;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);
	mdin_value[l_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 3 bit : L, M, H
	l_index = 0;
	m_index = 1;
	h_index = 2;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 7;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 3 bit : H, M, L
	l_index = 2;
	m_index = 1;
	h_index = 0;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 7;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 3 bit : M, L, H
	l_index = 1;
	m_index = 0;
	h_index = 2;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 0;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 0;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 0;
	mdin_value[h_index] = 1;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 0;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = 1;
	mdin_value[m_index] = 1;
	mdin_value[h_index] = 1;
	exp_step = 7;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);
}

/*
 * 		test item : EXI_DI_handleDin
 *
 *		1. set emergency_stop at index=0, check STOP_CMD, regardless ctrl_in
 * 		2. set external_trip at index=1, check STOP_CMD, regardless ctrl_in
 * 		3. set runPin at index=2, ctrl_in is not DIN, not send RUN CMD working
 * 		4. set ctrl_in = DIN, set run_pin=1, send RUN_CMD
 * 		5. set run_pin=1 in run state, not send RUN_CMD
 * 		6. set dir_pin at index=1 in run state, send DIR_F or DIR_R according to dir status
 *
 */
void test_handleDin(void)
{
	uint8_t index;
	DIN_config_t func_set;
	uint8_t exp_pin;
	int8_t result, exp_result;
	COMM_CMD_t exp_cmd;
	uint8_t exp_step;


	table_setValue(ctrl_in_type, CTRL_IN_NFC); // not in CTRL_IN_Digital, but work
	table_setStatusValue(run_status1_type, 1); // stop, forward
	state_run_stop = 0;
	state_direction = 0;

	// set pin 0 as emergency_stop
	index = 0;
	func_set = DIN_emergency_stop;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.emergency_pin);
	mdin_value[m_din.emergency_pin] = 0;
	exp_result = 0; // not sending
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	exp_result = 1;
	mdin_value[m_din.emergency_pin] = 1;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// set pin 1 as external_trip
	index = 1;
	func_set = DIN_external_trip;
	mdin_value[m_din.emergency_pin] = 0; // clear emergency
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.trip_pin);
	mdin_value[m_din.trip_pin] = 0;
	exp_result = 0; // not sending
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	exp_result = 1;
	mdin_value[m_din.trip_pin] = 1;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// set pin 2 as run_pin
	index = 2;
	func_set = DIN_run;
	mdin_value[m_din.emergency_pin] = 0;
	mdin_value[m_din.trip_pin] = 0;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.run_pin);
	mdin_value[m_din.run_pin] = 0; //STOP
	exp_result = 0; // not send command
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	mdin_value[m_din.run_pin] = 1; //RUN
	exp_result = 0; // not send command, ctrl_in != DIN
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// set pin 1 as dir_pin
	index = 1;
	func_set = DIN_direction;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.dir_pin);
	mdin_value[m_din.dir_pin] = 0;
	exp_result = 0; // not send command
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	mdin_value[m_din.dir_pin] = 1;
	exp_result = 0; // not send command
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	mdin_value[m_din.emergency_pin] = 0;
	mdin_value[m_din.dir_pin] = 0;
	mdin_value[m_din.run_pin] = 0;
	// set ctrl_in = DIN, STOP, FORWARD
	table_setValue(ctrl_in_type, CTRL_IN_Digital);

	// DIN config : emerg= 0, dir=1, run=2
	exp_result = 0; // not send command, no change
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// run
	mdin_value[m_din.run_pin] = 1;
	exp_cmd = SPICMD_CTRL_RUN;
	exp_result = 0; // send RUN
	result = EXI_DI_handleDin();
	TEST_ASSERT_NOT_EQUAL(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 4); // run
	state_run_stop = CMD_RUN;
	exp_result = 0; // not send command, no change
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	// dir R
	mdin_value[m_din.dir_pin] = 1;
	exp_cmd = SPICMD_CTRL_DIR_R;
	exp_result = 0; // send DIR_R
	result = EXI_DI_handleDin();
	TEST_ASSERT_NOT_EQUAL(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x104); // run + R
	state_direction = CMD_DIR_R;
	exp_result = 0; // not send command, no change
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	// stop
	mdin_value[m_din.run_pin] = 0;
	exp_cmd = SPICMD_CTRL_STOP;
	exp_result = 0; // send STOP
	result = EXI_DI_handleDin();
	TEST_ASSERT_NOT_EQUAL(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x101); // stop + R
	state_run_stop = CMD_STOP;

	// set bit_H = 0, bit_L=1, run=2,
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(0, DIN_freq_high, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	result = EXT_DI_setupMultiFuncDin(1, DIN_freq_low, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	mdin_value[m_din.bit_L] = 0;
	mdin_value[m_din.bit_H] = 0;

	// run
	mdin_value[m_din.run_pin] = 1;
	exp_cmd = SPICMD_CTRL_RUN;
	exp_result = 0; // send RUN
	result = EXI_DI_handleDin();
	TEST_ASSERT_NOT_EQUAL(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x104); // run + R
	state_run_stop = CMD_RUN;

	mdin_value[m_din.bit_H] = 1;
	exp_cmd = SPICMD_PARAM_W;
	exp_result = 1; // send freq
	exp_step = 4;
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);

	mdin_value[m_din.bit_L] = 1;
	exp_cmd = SPICMD_PARAM_W;
	exp_result = 1; // send freq
	exp_step = 5;
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);

	mdin_value[m_din.bit_H] = 0;
	mdin_value[m_din.bit_L] = 0;
	exp_cmd = SPICMD_PARAM_W;
	exp_result = 1; // send freq
	exp_step = 0;
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);

	// stop
	mdin_value[m_din.run_pin] = 0;
	exp_cmd = SPICMD_CTRL_STOP;
	exp_result = 1; // send STOP
	result = EXI_DI_handleDin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x101); // stop + R
	state_run_stop = CMD_STOP;

}

#endif
