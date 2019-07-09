/*
 * test_ext_ai.c
 *
 *  Created on: 2019. 6. 18.
 *      Author: hrjung
 */

#include "includes.h"

#ifdef SUPPORT_UNIT_TEST

#include <memory.h>

#include "unity.h"
#include "table.h"

#include "ext_io.h"



extern uint16_t adc_value;


extern void test_clear(void);
extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t opt);
extern int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t opt);


extern int8_t EXT_AI_setConfig(void);
extern int32_t EXT_AI_getFreq(uint16_t adc_val);
extern int8_t EXT_AI_handleAin(void);


/*
 * 		test item : EXT_AI_getFreq
 *
 *		set V_min = 0V, V_max = 10V
 *			1. adc_value = 4095, return freq_max
 * 			2. adc_value = 0, return freq_min
 * 		set V_min = 2V, V_max = 8V
 *			3. adc_value = 4095, return freq_max
 *			4. adc_value > 8V, return freq_max
 * 			5. adc_value = 0, return freq_min
 * 			6.  adc_value < 2, return freq_min
 *
 */
void test_getFreq(void)
{
	uint16_t value;
	int32_t exp_freq, freq_l;


	//config :  0 ~ 10V, 0 ~ 60Hz
	table_setAinValue(v_in_min_type, 0, REQ_FROM_TEST);  // 0V
	table_setAinValue(v_in_max_type, 100, REQ_FROM_TEST); // 10V
	table_setAinFreqValue(v_in_min_freq_type, 0, REQ_FROM_TEST); // 0Hz
	table_setAinFreqValue(v_in_max_freq_type, 600, REQ_FROM_TEST); // 60Hz
	EXT_AI_setConfig();

	adc_value = 4095;
	exp_freq = 600;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 0;
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 2048; // half value
	exp_freq = 300;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	//config :  2 ~ 8V, 0 ~ 60Hz
	table_setAinValue(v_in_min_type, 20, REQ_FROM_TEST); // 2V
	table_setAinValue(v_in_max_type, 80, REQ_FROM_TEST); // 8V
	EXT_AI_setConfig();

	adc_value = 4095;
	exp_freq = 600;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 0;
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 3300; // over 8V
	exp_freq = 600;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 800; // under 2V
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 2047; // half value
	exp_freq = 300;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);


	table_setAinFreqValue(v_in_min_freq_type, 200, REQ_FROM_TEST); // 20Hz
	table_setAinFreqValue(v_in_max_freq_type, 500, REQ_FROM_TEST); // 50Hz
	EXT_AI_setConfig();

	adc_value = 4095;
	exp_freq = 500;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 0;
	exp_freq = 200;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 3300; // over 8V
	exp_freq = 500;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 800; // under 2V
	exp_freq = 200;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 2048; // half value
	exp_freq = 350;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);
}


/*
 * 		test item : EXT_AI_handleAin
 *
 *		1. set DO as run and stop, verify mdout_value, correctly set
 * 		2. set DO as overload and shaftbrake, verify mdout_value, correctly set
 *
 *
 */
void test_handleAin(void)
{

	int8_t exp_result0, exp_result1;

	test_clear();

	//config :  0 ~ 10V, 0 ~ 60Hz
	table_setAinValue(v_in_min_type, 0, REQ_FROM_TEST);  // 0V
	table_setAinValue(v_in_max_type, 100, REQ_FROM_TEST); // 10V
	table_setAinFreqValue(v_in_min_freq_type, 0, REQ_FROM_TEST); // 0Hz
	table_setAinFreqValue(v_in_max_freq_type, 600, REQ_FROM_TEST); // 60Hz

	// set CTRL_in as Analog_V
	//table_setValue(ctrl_in_type, CTRL_IN_Analog_V);
	table_setStatusValue(run_status1_type, 0x0); // fwd, stop



}

#endif
