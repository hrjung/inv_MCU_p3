/*
 * test_ext_do.c
 *
 *  Created on: 2019. 6. 17.
 *      Author: hrjung
 */


#include "includes.h"

#ifdef SUPPORT_UNIT_TEST
#include <stdio.h>
#include <string.h>
#include <memory.h>

#include "unity.h"
#include "table.h"

#include "ext_io.h"


extern uint8_t err_state;
//extern int32_t table_data[];

extern uint8_t mdout_value[];

extern void test_clear(void);
extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value);
//extern int32_t table_getStatusValue(int16_t index);
extern void table_setStatusValue(int16_t index, int32_t value);

/*
 * 		test item : EXT_DO_handleDout
 *
 *		1. set DO as run and stop, verify mdout_value, correctly set
 * 		2. set DO as overload and shaftbrake, verify mdout_value, correctly set
 *
 *
 */
void test_handleDout(void)
{
	PARAM_IDX_t index0=multi_Dout_0_type, index1=multi_Dout_1_type;
	int8_t exp_result0, exp_result1;

	test_clear();

	// run/stop status out
	exp_result0 = 0;
	exp_result1 = 1;
#ifdef SUPPORT_NFC_OLD
	table_setStatusValue(run_status1_type, 0); // STOP
#else
	table_setStatusValue(run_status_type, 0); // STOP
#endif
	table_setValue(index0, DOUT_running);
	table_setValue(index1, DOUT_stop);

	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]);
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);

	exp_result0 = 1;
	exp_result1 = 0;
#ifdef SUPPORT_NFC_OLD
	table_setStatusValue(run_status1_type, 1);
#else
	table_setStatusValue(run_status_type, 1); // running
#endif
	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]);
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);

	// overload, shaft brake out
	exp_result0 = 0;
	exp_result1 = 1;
#ifdef SUPPORT_NFC_OLD
	table_setStatusValue(run_status2_type, 0x100);
#else
	table_setStatusValue(overload_alarm_type, 0);
	table_setStatusValue(shaftbrake_status_type, 1);
#endif
	table_setValue(index0, DOUT_overload);
	table_setValue(index1, DOUT_shaftbrake_on);
	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]);
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);

	exp_result0 = 1;
	exp_result1 = 0;
#ifdef SUPPORT_NFC_OLD
	table_setStatusValue(run_status2_type, 1);
#else
	table_setStatusValue(overload_alarm_type, 1);
	table_setStatusValue(shaftbrake_status_type, 0);
#endif
	table_setValue(index0, DOUT_overload);
	table_setValue(index1, DOUT_shaftbrake_on);
	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]);
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);

	// trip at 1
	err_state = 0; // no error
	exp_result1 = 0;
	table_setValue(index1, DOUT_trip_notify);
	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]); //
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);

	err_state = 1; // set error state
	exp_result1 = 1;
	EXT_DO_handleDout();
	TEST_ASSERT_EQUAL_INT(exp_result0, mdout_value[0]); //
	TEST_ASSERT_EQUAL_INT(exp_result1, mdout_value[1]);
}
#endif
