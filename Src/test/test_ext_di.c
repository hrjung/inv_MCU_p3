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

extern uint8_t prev_emergency, prev_trip;

extern uint8_t prev_run, prev_dir;

extern uint8_t mdin_value[];
extern DIN_PIN_NUM_t m_din;
extern COMM_CMD_t test_cmd;
extern uint8_t step_cmd;

extern void HDLR_setStopFlag(uint8_t flag);

extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setStatusValue(PARAM_IDX_t index, int32_t value, int16_t option);
extern int8_t table_setValueDir(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setMultiFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);

void test_clear(void)
{
	mdin_value[0] = EXT_DI_INACTIVE;
	mdin_value[1] = EXT_DI_INACTIVE;
	mdin_value[2] = EXT_DI_INACTIVE;

	state_run_stop = 0;
	state_direction = 0;

	st_overload = 0;
	st_brake = 0;

	table_data[multi_Din_0_type] = DIN_unuse;
	table_data[multi_Din_1_type] = DIN_unuse;
	table_data[multi_Din_2_type] = DIN_unuse;

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
	mdin_value[l_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(l_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use only 1 bit : mid
	m_index = 0;
	func_set = DIN_freq_mid;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(m_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = EXT_DI_ACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use only 1 bit : high
	h_index = 0;
	func_set = DIN_freq_high;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	result = EXT_DI_setupMultiFuncDin(h_index, func_set, REQ_FROM_TEST);
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);


	test_clear();

	// use 2 bit : low + mid
	l_index = 0;
	m_index = 1;
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 2 bit : mid + high
	h_index = 0;
	m_index = 1;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(m_index, DIN_freq_mid, REQ_FROM_TEST);
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	// use 2 bit : low + high
	l_index = 0;
	h_index = 1;
	result = EXT_DI_setupMultiFuncDin(h_index, DIN_freq_high, REQ_FROM_TEST);
	result = EXT_DI_setupMultiFuncDin(l_index, DIN_freq_low, REQ_FROM_TEST);
	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
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

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
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

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
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

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 0;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 1;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 2;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_INACTIVE;
	exp_step = 3;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 4;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_INACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 5;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_INACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 6;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);

	mdin_value[l_index] = EXT_DI_ACTIVE;
	mdin_value[m_index] = EXT_DI_ACTIVE;
	mdin_value[h_index] = EXT_DI_ACTIVE;
	exp_step = 7;
	step = EXT_DI_convertMultiStep();
	TEST_ASSERT_EQUAL_INT(exp_step, step);
}

/*
 * 		test item : test_handleDinEmergency
 *
 *		1. set emergency_stop at index=0, check STOP_CMD, regardless ctrl_in
 * 		2. set external_trip at index=1, check STOP_CMD, regardless ctrl_in
 */
void test_handleDinEmergency(void)
{
	uint8_t index;
	DIN_config_t func_set;
	uint8_t exp_pin;
	int8_t result, exp_result;
	COMM_CMD_t exp_cmd;


	table_setValue(ctrl_in_type, CTRL_IN_NFC, REQ_FROM_TEST); // not in CTRL_IN_Digital, but work
	table_setStatusValue(run_status1_type, 1, REQ_FROM_TEST); // stop, forward
	state_run_stop = CMD_STOP;
	state_direction = CMD_DIR_F;

	// set pin 0 as emergency_stop
	index = 0;
	func_set = DIN_emergency_stop;
	prev_emergency = EXT_DI_INACTIVE;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.emergency_pin);
	mdin_value[m_din.emergency_pin] = EXT_DI_INACTIVE;
	exp_result = 1; // do nothing
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// enable emergency stop
	exp_result = 1;
	mdin_value[m_din.emergency_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	exp_result = 0;
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// clear emergency
	mdin_value[m_din.emergency_pin] = EXT_DI_INACTIVE;

	// set pin 1 as external_trip
	index = 1;
	func_set = DIN_external_trip;
	prev_trip = EXT_DI_INACTIVE;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.trip_pin);
	mdin_value[m_din.trip_pin] = EXT_DI_INACTIVE;
	exp_result = 1; // do nothing
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	exp_result = 0;
	mdin_value[m_din.trip_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// clear emergency and ext_trip
	mdin_value[m_din.emergency_pin] = EXT_DI_INACTIVE;
	mdin_value[m_din.trip_pin] = EXT_DI_INACTIVE;

	// set pin 2 as run_pin
	index = 2;
	func_set = DIN_run;
	exp_pin = index;
	exp_result = 1;
	result = EXT_DI_setupMultiFuncDin(index, func_set, REQ_FROM_TEST);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_pin, m_din.run_pin);
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE; //STOP
	exp_result = 1; // do nothing
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	mdin_value[m_din.run_pin] = EXT_DI_ACTIVE; //RUN
	exp_result = 1; // do nothing
	result = EXT_DI_handleEmergency();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
}

/*
 * 		test item : EXT_DI_handleDin
 *
 * 		1. set run_pin=0, do nothing, run_pin=1,  send RUN_CMD,
 * 		2. set dir_pin=1 in run state, send DIR_F
 *
 */
void test_handleDin(void)
{
	int8_t result, exp_result;
	COMM_CMD_t exp_cmd;
	uint8_t exp_step;

	table_setStatusValue(run_status1_type, 1, REQ_FROM_TEST); // stop, forward
	state_run_stop = CMD_STOP;
	state_direction = CMD_DIR_F;

	// initial setting
	// DI_0 : DIN_run
	// DI_1 : DIN_direction
	EXT_DI_setupMultiFuncDin(0, DIN_run, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(1, DIN_direction, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(2, DIN_unuse, REQ_FROM_TEST);

	// initial test set
	test_cmd = SPICMD_TEST_CMD;
	prev_run = EXT_DI_INACTIVE;
	prev_dir = EXT_DI_INACTIVE;
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;

	// set ctrl_in = DIN, STOP, FORWARD
	exp_result = 0; // do nothing
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// run
	mdin_value[m_din.run_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_RUN;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 4, REQ_FROM_TEST); // run
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// dir R
	mdin_value[m_din.dir_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_DIR_R;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x104, REQ_FROM_TEST); // run + R
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// dir F
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_DIR_F;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x4, REQ_FROM_TEST); // run + F
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// stop
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x1, REQ_FROM_TEST); // stop + F
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	table_setValueDir(dir_domain_type, DIR_FORWARD_ONLY, REQ_FROM_TEST); // set forward only

	// dir R not working
	mdin_value[m_din.dir_pin] = EXT_DI_ACTIVE;
	exp_result = 0; // do nothing
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// restore dir F
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	exp_result = 0; // do nothing
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	table_setValueDir(dir_domain_type, DIR_ALL, REQ_FROM_TEST); // set bi-directional

	// dir R
	mdin_value[m_din.dir_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_DIR_R;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x101, REQ_FROM_TEST); // stop + R
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	table_setValueDir(dir_domain_type, DIR_REVERSE_ONLY, REQ_FROM_TEST); // set reverse only

	// dir F not working
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	exp_result = 0; // do nothing
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// restore dir R
	mdin_value[m_din.dir_pin] = EXT_DI_ACTIVE;
	exp_result = 0; // do nothing
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	table_setValueDir(dir_domain_type, DIR_ALL, REQ_FROM_TEST); // set bi-directional

	table_setStatusValue(run_status1_type, 0x101, REQ_FROM_TEST); // stop + R
	exp_result = 0; // no change
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	// DI_0 : bit_H
	// DI_1 : bit_L
	// DI_2 : run-stop
	EXT_DI_setupMultiFuncDin(0, DIN_freq_high, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(1, DIN_freq_low, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(2, DIN_run, REQ_FROM_TEST);
	mdin_value[m_din.bit_L] = EXT_DI_INACTIVE;
	mdin_value[m_din.bit_H] = EXT_DI_INACTIVE;
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;
	table_setMultiFreqValue(multi_val_1_type, 300, REQ_FROM_TEST);
	table_setMultiFreqValue(multi_val_4_type, 400, REQ_FROM_TEST);
	table_setMultiFreqValue(multi_val_5_type, 500, REQ_FROM_TEST);

	// run
	mdin_value[m_din.run_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_RUN;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x104, REQ_FROM_TEST); // run + R

	mdin_value[m_din.bit_H] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_PARAM_W;
	exp_step = 4;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);
	test_cmd = SPICMD_TEST_CMD;

	mdin_value[m_din.bit_L] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_PARAM_W;
	exp_step = 5;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);
	test_cmd = SPICMD_TEST_CMD;

	mdin_value[m_din.bit_H] = EXT_DI_INACTIVE;
	mdin_value[m_din.bit_L] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_PARAM_W;
	exp_step = 0;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);
	test_cmd = SPICMD_TEST_CMD;

	// stop
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	exp_result = 1; // send STOP
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	test_cmd = SPICMD_TEST_CMD;

	// stop is in progress, no step change is applied
	mdin_value[m_din.bit_H] = EXT_DI_INACTIVE;
	mdin_value[m_din.bit_L] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_TEST_CMD;
	exp_step = 0;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);

	table_setStatusValue(run_status1_type, 0x101, REQ_FROM_TEST); // stop + R

	// end of STOP than apply change
	exp_cmd = SPICMD_PARAM_W;
	exp_step = 1;
	result = EXT_DI_handleDin(CTRL_IN_Digital);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_step, step_cmd);
}

#endif
