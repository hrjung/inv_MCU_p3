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
#include "dsp_comm.h"

#include "ext_io.h"



extern uint16_t adc_value;
extern int32_t prev_adc_cmd;

extern uint8_t mdin_value[];
extern DIN_PIN_NUM_t m_din;
extern COMM_CMD_t test_cmd;
extern uint8_t step_cmd;

extern int16_t state_run_stop;
extern int16_t state_direction;
extern uint8_t prev_run, prev_dir;

extern float freq_min, freq_max, V_ai_min, V_ai_max;
extern uint8_t isConfigured;

extern void test_clear(void);
extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t opt);
extern int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t opt);

extern int8_t table_setAinValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setAinFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setMultiFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);

extern int8_t table_setStatusValue(PARAM_IDX_t index, int32_t value, int16_t option);

extern int8_t EXT_AI_setConfig(void);
extern int32_t EXT_AI_getFreq(uint16_t adc_val);
extern int8_t EXT_AI_handleAin(void);
extern int8_t EXT_handleDAin(int32_t ctrl_in);

extern void test_clear(void);

void test_ai_clear(void)
{
	freq_min=0.0;
	freq_max=0.0;

	V_ai_min=0.0;
	V_ai_max=0.0;

	isConfigured=0;
}
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

	test_ai_clear();

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

	adc_value = (EXT_AIN_ADC_MAX-EXT_AIN_ADC_MIN)/2 + EXT_AIN_ADC_MIN; // half value
	exp_freq = 301;
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

	adc_value = 690; // under 2V ~= 695.xx
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 1663; // half value 5V ~= 1663
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
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 3300; // over 8V
	exp_freq = 500;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 690; // under 2V
	exp_freq = 0;
	value = adc_value;
	freq_l = EXT_AI_getFreq(value);
	TEST_ASSERT_EQUAL_INT(exp_freq, freq_l);

	adc_value = 1663; // half value
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
	int8_t exp_result, result;
	COMM_CMD_t exp_cmd;
	int32_t exp_freq;

	test_ai_clear();

	//config :  0 ~ 10V, 0 ~ 60Hz
	table_setAinValue(v_in_min_type, 0, REQ_FROM_TEST);  // 0V
	table_setAinValue(v_in_max_type, 100, REQ_FROM_TEST); // 10V
	table_setAinFreqValue(v_in_min_freq_type, 0, REQ_FROM_TEST); // 0Hz
	table_setAinFreqValue(v_in_max_freq_type, 600, REQ_FROM_TEST); // 60Hz

	// set CTRL_in as Analog_V
	//table_setValue(ctrl_in_type, CTRL_IN_Analog_V, REQ_FROM_TEST);
	table_setStatusValue(run_status1_type, STATE_STOP, REQ_FROM_TEST); // fwd, stop

	adc_value = 0; // initial STOP
	exp_result = 1; // do nothing
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	adc_value = 300; // start : run and param_w
	exp_cmd = SPICMD_PARAM_W;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, STATE_RUN, REQ_FROM_TEST); // fwd, stop

	adc_value = (EXT_AIN_ADC_MAX-EXT_AIN_ADC_MIN)/2 + EXT_AIN_ADC_MIN; // half value
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 301;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = EXT_AIN_ADC_MAX+10; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 4095; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = EXT_AIN_ADC_MIN-10; // max
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);


	//config :  2 ~ 8V, 0 ~ 60Hz
	table_setAinValue(v_in_min_type, 20, REQ_FROM_TEST);  // 2V
	table_setAinValue(v_in_max_type, 80, REQ_FROM_TEST); // 8V

	table_setStatusValue(run_status1_type, STATE_STOP, REQ_FROM_TEST); // fwd, stop


	adc_value = 0; // initial STOP
	exp_result = 1; // do nothing
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	adc_value = 750; // over 2V ~= 695.xx
	exp_cmd = SPICMD_PARAM_W; // run and param_w
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, STATE_RUN, REQ_FROM_TEST); // fwd, stop

	adc_value = 1663; // half value 5V ~= 1663
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 300;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 3300; // over 8V
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 4095; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 690; // under 2V ~= 695.xx
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);


	//config :  2 ~ 8V, 20 ~ 50Hz
	table_setAinFreqValue(v_in_min_freq_type, 200, REQ_FROM_TEST); // 20Hz
	table_setAinFreqValue(v_in_max_freq_type, 500, REQ_FROM_TEST); // 50Hz

	table_setStatusValue(run_status1_type, STATE_STOP, REQ_FROM_TEST); // fwd, stop


	adc_value = 0; // initial STOP
	exp_result = 1; // do nothing
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	adc_value = 750; // over 2V ~= 695.xx
	exp_cmd = SPICMD_PARAM_W; // run and param_w
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, STATE_RUN, REQ_FROM_TEST); // fwd, stop

	adc_value = 1663; // half value 5V ~= 1663
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 350;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 3300; // over 8V
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 500;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 4095; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 500;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 690; // under 2V ~= 695.xx
	exp_cmd = SPICMD_CTRL_STOP;
	exp_freq = 0;
	result = EXT_AI_handleAin();
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);
}

/*
 * 		test item : test_handleDAin
 *
 * 		1. normal test, set AI freq value, set DI as run, dir command
 * 		2. same AI setting, test DI multi-step not working
 * 		3. same AI setting, change ADC value -> change freq
 *
 */
void test_handleDAin(void)
{
	int8_t result, exp_result;
	COMM_CMD_t exp_cmd;
	int32_t exp_freq;

	test_ai_clear();
	test_clear();

	// analog input setting
	table_setAinValue(v_in_min_type, 0, REQ_FROM_TEST);  // 0V
	table_setAinValue(v_in_max_type, 100, REQ_FROM_TEST); // 10V
	table_setAinFreqValue(v_in_min_freq_type, 0, REQ_FROM_TEST); // 0Hz
	table_setAinFreqValue(v_in_max_freq_type, 600, REQ_FROM_TEST); // 60Hz

	// initialize status value
	table_setStatusValue(run_status1_type, STATE_STOP, REQ_FROM_TEST); // fwd, stop
	state_run_stop = CMD_STOP;
	state_direction = CMD_DIR_F;

	// initial DI setting
	// DI_1 : DIN_run
	// DI_2 : DIN_direction
	EXT_DI_setupMultiFuncDin(1, DIN_run, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(2, DIN_direction, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(0, DIN_unuse, REQ_FROM_TEST);

	// initial DI test set
	test_cmd = SPICMD_TEST_CMD;
	prev_run = EXT_DI_INACTIVE;
	prev_dir = EXT_DI_INACTIVE;
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;


	adc_value = 0; // initial STOP
	exp_result = 0; // do nothing
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	adc_value = 300; // set initial freq, not start
	exp_cmd = SPICMD_PARAM_W;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// run
	mdin_value[m_din.run_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_RUN;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 4, REQ_FROM_TEST); // run
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// change freq
	adc_value = (EXT_AIN_ADC_MAX-EXT_AIN_ADC_MIN)/2 + EXT_AIN_ADC_MIN; // half value
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 301;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = EXT_AIN_ADC_MAX+10; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = 4095; // max
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 600;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	adc_value = EXT_AIN_ADC_MIN-10; // min freq, not stop
	exp_cmd = SPICMD_PARAM_W;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	// dir R
	mdin_value[m_din.dir_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_DIR_R;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x104, REQ_FROM_TEST); // run + R
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// dir F
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_DIR_F;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x4, REQ_FROM_TEST); // run + F
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	// stop
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x1, REQ_FROM_TEST); // stop + F
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	//2. multi-step command not working
	// initial DI setting
	// DI_1 : DIN_run
	// DI_2 : bit_L
	// DI_0 : bit_M
	EXT_DI_setupMultiFuncDin(1, DIN_run, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(2, DIN_freq_low, REQ_FROM_TEST);
	EXT_DI_setupMultiFuncDin(0, DIN_freq_mid, REQ_FROM_TEST);
	table_setMultiFreqValue(multi_val_1_type, 300, REQ_FROM_TEST);
	table_setMultiFreqValue(multi_val_2_type, 400, REQ_FROM_TEST);
	table_setMultiFreqValue(multi_val_3_type, 500, REQ_FROM_TEST);

	// initial DI test set
	test_cmd = SPICMD_TEST_CMD;
	prev_run = EXT_DI_INACTIVE;
	prev_dir = EXT_DI_INACTIVE;
	mdin_value[m_din.dir_pin] = EXT_DI_INACTIVE;
	mdin_value[m_din.bit_M] = EXT_DI_INACTIVE;
	mdin_value[m_din.bit_L] = EXT_DI_INACTIVE;


	// run
	mdin_value[m_din.run_pin] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_CTRL_RUN;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 4, REQ_FROM_TEST); // run
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	test_cmd = SPICMD_TEST_CMD;

	mdin_value[m_din.bit_L] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_TEST_CMD;
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	mdin_value[m_din.bit_M] = EXT_DI_ACTIVE;
	exp_cmd = SPICMD_TEST_CMD;
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//3. change freq
	adc_value = (EXT_AIN_ADC_MAX-EXT_AIN_ADC_MIN)/2 + EXT_AIN_ADC_MIN; // half value
	exp_cmd = SPICMD_PARAM_W;
	exp_freq = 301;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);
	TEST_ASSERT_EQUAL_INT(exp_freq, prev_adc_cmd);

	// stop
	mdin_value[m_din.run_pin] = EXT_DI_INACTIVE;
	exp_cmd = SPICMD_CTRL_STOP;
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_cmd, test_cmd);

	table_setStatusValue(run_status1_type, 0x1, REQ_FROM_TEST); // stop + F
	exp_result = 0; // no change
	result = EXT_handleDAin(CTRL_IN_Din_Ain);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
}

#endif
