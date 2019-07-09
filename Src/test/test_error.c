/*
 * test_error.c
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */

#include "includes.h"

#ifdef SUPPORT_UNIT_TEST

#include <memory.h>

#include "unity.h"
#include "table.h"
#include "error.h"

extern int32_t table_data[];

extern int8_t table_updateErrorDSP(uint16_t err_code, uint16_t status, float current, float freq);


static void init_table_data_error(void)
{
	int i;

	for(i=err_code_0_type; i<=err_freq_4_type; i++)
		table_data[i] = (int32_t)0;

	table_data[err_code_0_type] = TRIP_REASON_NONE;
	table_data[err_code_1_type] = TRIP_REASON_NONE;
	table_data[err_code_2_type] = TRIP_REASON_NONE;
	table_data[err_code_3_type] = TRIP_REASON_NONE;
	table_data[err_code_4_type] = TRIP_REASON_NONE;

}


/*
 * 		test item
 *
 * 		1. put 1st error, check value
 * 		2. put 2nd error, check value at [0], 1st error in [1]
 * 		3. put 3rd error, check value at [0], 1st error in [2]
 * 		4. put 4th error, check value at [0], 1st error in [3]
 * 		5. put 5th error, check value at [0], 1st error in [4]
 * 		6. put 6th error, check value at [0], 2nd error in [4]
 *
 */

void test_errorBasic(void)
{
	int16_t err_code, status;
	float current, freq;
	int32_t exp_err, exp_status, exp_cur, exp_freq;

	init_table_data_error();

	// 1st error
	err_code = 1;
	status = 2;
	current = 3.2;
	freq = 30.0;
	exp_err = 1;
	exp_status = 2;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// 2nd error
	err_code = 10;
	status = 4;
	current = 1.8;
	freq = 15.3;
	exp_err = (int32_t)err_code;
	exp_status = (int32_t)status;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// err0 move to err1
	current = 3.2;
	freq = 30.0;

	exp_err = 1;
	exp_status = 2;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_1_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_1_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_1_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_1_type]);

	// 3rd error
	err_code = 15;
	status = 0;
	current = 5.2;
	freq = 54.8;
	exp_err = (int32_t)err_code;
	exp_status = (int32_t)status;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// err1 move to err2
	current = 3.2;
	freq = 30.0;

	exp_err = 1;
	exp_status = 2;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_2_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_2_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_2_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_2_type]);

	// 4th error
	err_code = 15;
	status = 0;
	current = 5.2;
	freq = 54.8;
	exp_err = (int32_t)err_code;
	exp_status = (int32_t)status;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// err2 move to err3
	current = 3.2;
	freq = 30.0;

	exp_err = 1;
	exp_status = 2;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_3_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_3_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_3_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_3_type]);

	// 5th error
	err_code = 39;
	status = 3;
	current = 10.2;
	freq = 150.5;
	exp_err = (int32_t)err_code;
	exp_status = (int32_t)status;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// err3 move to err4
	current = 3.2;
	freq = 30.0;

	exp_err = 1;
	exp_status = 2;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_4_type]);

	// 5th error
	err_code = 39;
	status = 3;
	current = 10.2;
	freq = 150.5;
	exp_err = (int32_t)err_code;
	exp_status = (int32_t)status;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	table_updateErrorDSP(err_code, status, current, freq);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	// err3 move to err4, 2nd error info
	current = 1.8;
	freq = 15.3;

	exp_err = 10;
	exp_status = 4;
	exp_cur = (int32_t)(current*10.0 + 0.05);
	exp_freq = (int32_t)(freq*10.0 + 0.05);
	TEST_ASSERT_EQUAL_INT(exp_err, table_data[err_code_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_status, table_data[err_status_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_cur, table_data[err_current_4_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_4_type]);
}

#endif
