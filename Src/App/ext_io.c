/*
 * ext_io.c
 *
 *  Created on: 2019. 6. 14.
 *      Author: hrjung
 */

#include "includes.h"

#include "proc_uart.h"
#include "table.h"
#include "dsp_comm.h"
#include "error.h"
#include "ext_io.h"
#include "drv_gpio.h"

#define F_CMD_DIFF_HYSTERISIS		(10)


STATIC DIN_PIN_NUM_t m_din = {
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
		EXT_DIN_COUNT,
};

uint8_t mdin_value[EXT_DIN_COUNT]; // actual DI pin value
STATIC uint8_t prev_emergency=0, prev_trip=0, prev_run=0, prev_dir=0, prev_step=0;
STATIC COMM_CMD_t test_cmd=0;
STATIC uint8_t step_cmd=0;

uint8_t mdout_value[EXT_DOUT_COUNT];

uint16_t adc_value=0; // analog input value
float freq_min=0.0, freq_max=0.0, V_ai_min=0.0, V_ai_max=0.0;
STATIC int32_t prev_adc_cmd=0;

uint8_t isConfigured=0; // flag for AIN parameter configured

extern int16_t state_run_stop; // global run/stop status
extern int16_t state_direction; // global forward/reverse direction status
extern int16_t st_overload;
extern int16_t st_brake;

extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);

extern int8_t COMM_setMultiStepFreq(PARAM_IDX_t table_idx, uint16_t *buf);
extern int8_t COMM_setAnalogFreq(int32_t freq, uint16_t *buf);

extern void TM_setStartRunTime(void);

int EXT_DI_isMultiStepValid(void)
{
	int valid=0;

	if(m_din.bit_L != EXT_DIN_COUNT) valid = 1;

	if(m_din.bit_M != EXT_DIN_COUNT) valid = 1;

	if(m_din.bit_H != EXT_DIN_COUNT) valid = 1;

	return valid;
}

// mdin_value should have input DIN value
uint8_t EXT_DI_convertMultiStep(void)
{
	uint8_t low=0, mid=0, high=0;
	uint8_t step=0;

	if(EXT_DI_isMultiStepValid() == 0) // no multi-step setting
	{
		//printf("L:%d, M:%d, H:%d\n", m_din.bit_L, m_din.bit_M, m_din.bit_H);
		return 0; // 0 is not valid multi-step
	}

	if(m_din.bit_L != EXT_DIN_COUNT)
	{
		low = mdin_value[m_din.bit_L]&0x01;
	}

	if(m_din.bit_M != EXT_DIN_COUNT)
	{
		mid = mdin_value[m_din.bit_M]&0x01;
	}

	if(m_din.bit_H != EXT_DIN_COUNT)
	{
		high = mdin_value[m_din.bit_H]&0x01;
	}

	step = ((high<<2)|(mid<<1)|low)&0x07;

	//printf("step = %d, high:mid:low = %d:%d:%d\n", step, high, mid, low);

	return step;
}

// same setting for 2 or more pin is not allowed, last one is only set
void EXT_DI_clearDinSameFunc(int index, DIN_config_t func_set, int16_t option)
{
	int i;
	DIN_config_t func;

	for(i=0; i<EXT_DIN_COUNT; i++)
	{
		if(index == i) continue;

		func = (DIN_config_t)table_getValue(multi_Din_0_type+i);
		if(func_set == func) table_setValue(multi_Din_0_type+i, DIN_unuse, option);
	}
}

// called by table_setValue
void EXT_DI_updateMultiDinPinIndex(uint8_t index, DIN_config_t func_set)
{
	if(func_set == DIN_run) m_din.run_pin = index;
	else if(index == m_din.run_pin) m_din.run_pin = EXT_DIN_COUNT;

	if(func_set == DIN_direction) m_din.dir_pin = index;
	else if(index == m_din.dir_pin) m_din.dir_pin = EXT_DIN_COUNT;

	if(func_set == DIN_freq_low) m_din.bit_L = index;
	else if(index == m_din.bit_L) m_din.bit_L = EXT_DIN_COUNT;

	if(func_set == DIN_freq_mid) m_din.bit_M = index;
	else if(index == m_din.bit_M) m_din.bit_M = EXT_DIN_COUNT;

	if(func_set == DIN_freq_high) m_din.bit_H = index;
	else if(index == m_din.bit_H) m_din.bit_H = EXT_DIN_COUNT;

	if(func_set == DIN_emergency_stop) m_din.emergency_pin = index;
	else if(index == m_din.emergency_pin) m_din.emergency_pin = EXT_DIN_COUNT;

	if(func_set == DIN_external_trip) m_din.trip_pin = index;
	else if(index == m_din.trip_pin) m_din.trip_pin = EXT_DIN_COUNT;
}


// read from table and init multiDin setting, used at table init
int8_t EXT_DI_setupMultiFuncDin(int index, DIN_config_t func_set, int16_t option)
{
	if(index >= EXT_DIN_COUNT) return 0;

	if(func_set >= DIN_config_max) return 0;

	EXT_DI_clearDinSameFunc(index, func_set, option);

	EXT_DI_updateMultiDinPinIndex(index, func_set); // update m_din.pin_num

	return 1;
}


int8_t EXI_DI_handleDin(void)
{
	int32_t ctrl_in;
	int32_t dir_ctrl;
	int32_t freq_step;
	uint8_t step=0;
	int8_t result=0, status;
	uint16_t dummy[3] = {0,0,0};

	if(m_din.emergency_pin != EXT_DIN_COUNT)
	{
		if(mdin_value[m_din.emergency_pin] == 1 && prev_emergency == 0) // send STOP regardless of run_status
		{
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send emergency SPICMD_CTRL_STOP\r\n");
#ifndef SUPPORT_UNIT_TEST
			// send STOP cmd
			status = COMM_sendMessage(test_cmd, dummy);
			if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
#endif
			prev_emergency = mdin_value[m_din.emergency_pin];
			return 1;
		}
		prev_emergency = mdin_value[m_din.emergency_pin];
	}

	if(m_din.trip_pin != EXT_DIN_COUNT)
	{
		if(mdin_value[m_din.trip_pin] == 1 && prev_trip == 0)
		{
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send trip SPICMD_CTRL_STOP\r\n");
#ifndef SUPPORT_UNIT_TEST
			// send STOP cmd
			status = COMM_sendMessage(test_cmd, dummy);
			if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
#endif
			prev_trip = mdin_value[m_din.trip_pin];
			return 1;
		}
		prev_trip = mdin_value[m_din.trip_pin];
	}

	ctrl_in = table_getCtrllIn();
	if(ctrl_in == CTRL_IN_Digital)
	{
		// handle run/stop
		if(m_din.run_pin != EXT_DIN_COUNT)
		{
			if(mdin_value[m_din.run_pin] == 0 && prev_run == 1 && state_run_stop == CMD_RUN)
			{
				// send STOP cmd
				test_cmd = SPICMD_CTRL_STOP;
				kprintf(PORT_DEBUG, "send SPICMD_CTRL_STOP\r\n");
				result++;
#ifndef SUPPORT_UNIT_TEST
				// send stop to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
#endif
				prev_run = 0;
			}
			else if(mdin_value[m_din.run_pin] == 1 && prev_run == 0 && state_run_stop == CMD_STOP)
			{
				// send run cmd
				test_cmd = SPICMD_CTRL_RUN;
				kprintf(PORT_DEBUG, "send SPICMD_CTRL_RUN\r\n");
				result++;
#ifndef SUPPORT_UNIT_TEST
				// send run to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status != COMM_FAILED)
				{
					TM_setStartRunTime(); // set Run start time, count
				}
				else { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
#endif
				prev_run = 1;
			}
			else
				result = 0;

			prev_run = mdin_value[m_din.run_pin];
			//if(result) return 1; // send only one command at 1 time

		}

		// handle direction
		if(m_din.dir_pin != EXT_DIN_COUNT)
		{
			dir_ctrl = table_getValue(dir_domain_type);
			if(mdin_value[m_din.dir_pin] == 0 && prev_dir == 1 && state_direction != CMD_DIR_F) // forward
			{
				// send FWD cmd
				if(dir_ctrl != DIR_REVERSE_ONLY)
				{
					// send forward cmd
					test_cmd = SPICMD_CTRL_DIR_F;
					kprintf(PORT_DEBUG, "send SPICMD_CTRL_DIR_F\r\n");
					result++;
#ifndef SUPPORT_UNIT_TEST
					// send run to DSP
					status = COMM_sendMessage(test_cmd, dummy);
					if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO DIR F error! \r\n"); }
#endif
				}

			}
			else if(mdin_value[m_din.dir_pin] == 1 && prev_dir == 0 && state_direction != CMD_DIR_R)	// reverse
			{
				// send RVS cmd
				if(dir_ctrl != DIR_FORWARD_ONLY)
				{
					// send reverse cmd
					test_cmd = SPICMD_CTRL_DIR_R;
					kprintf(PORT_DEBUG, "send SPICMD_CTRL_DIR_R\r\n");
					result++;
#ifndef SUPPORT_UNIT_TEST
					// send run to DSP
					status = COMM_sendMessage(test_cmd, dummy);
					if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO DIR R error! \r\n"); }
#endif
				}
			}
//			else
//				result = 0;

			prev_dir = mdin_value[m_din.dir_pin];
			//if(result) return 1;

		}

		// handle multi-step freq
		if(EXT_DI_isMultiStepValid())
		{
			step = EXT_DI_convertMultiStep();
			if(step != prev_step)
			{
				test_cmd = SPICMD_PARAM_W;
#ifndef SUPPORT_UNIT_TEST
				freq_step = table_getValue(multi_Din_0_type+step);
				status = table_setFreqValue(value_type, freq_step, REQ_FROM_EXTIO);
				if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO set multistep error! \r\n"); }
#endif
				prev_step = step;
				step_cmd = step; // for test
				result++;
			}
//			else
//				result = 0;
		}

	}

	return result;
}

int EXT_DO_isValid(void)
{
	int valid = 0;

	if(table_getValue(multi_Dout_0_type) != DOUT_unuse) valid = 1;

	if(table_getValue(multi_Dout_1_type) != DOUT_unuse) valid = 1;

	return valid;
}

void EXT_DO_setDoutPin(int do_idx, int32_t DO_config)
{
	int32_t value;

	switch(DO_config)
	{
	case DOUT_running:
		if(state_run_stop)
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;

	case DOUT_stop:
		if(state_run_stop == 0)
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;

	case DOUT_overload:
		if(st_overload)
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;

	case DOUT_shaftbrake_on:
		if(st_brake)
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;

	case DOUT_trip_notify:
		value = (int32_t)ERR_isErrorState();
		if(value)
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;
	}
	//kprintf(PORT_DEBUG, "DO index=%d, config=%d, mout_val=%d\r\n", do_idx, DO_config, mdout_value[do_idx]);
}

// DOUT set GPIO 0 -> ON
// DOUT set GPIO 1 -> OFF
int8_t EXT_DO_handleDout(void)
{
	int32_t do0_config, do1_config;

	do0_config = table_getValue(multi_Dout_0_type);
	if(do0_config != DOUT_unuse)
	{
		EXT_DO_setDoutPin(0, do0_config);
		UTIL_writeDout(0, mdout_value[0]);
	}
	//kprintf(PORT_DEBUG, "DO index=%d, config=%d, mout_val=%d\r\n", 0, do0_config, mdout_value[0]);
	do1_config = table_getValue(multi_Dout_1_type);
	if(do1_config != DOUT_unuse)
	{
		EXT_DO_setDoutPin(1, do1_config);
		UTIL_writeDout(1, mdout_value[1]);
	}
	//kprintf(PORT_DEBUG, "DO index=%d, config=%d, mout_val=%d\r\n", 1, do1_config, mdout_value[1]);
	return 1;
}

void EXT_AI_needReconfig(void)
{
	isConfigured = 0;
}

int8_t EXT_AI_setConfig(void)
{
	uint16_t ratio;
	int32_t value;

	if(isConfigured) return 1; // already configured

	ratio = table_getRatio(v_in_min_freq_type);
	value = table_getValue(v_in_min_freq_type);
	freq_min = (float)value/(float)ratio;

	ratio = table_getRatio(v_in_max_freq_type);
	value = table_getValue(v_in_max_freq_type);
	freq_max = (float)value/(float)ratio;

	ratio = table_getRatio(v_in_min_type);
	value = table_getValue(v_in_min_type);
	V_ai_min = (float)value/(float)ratio;

	ratio = table_getRatio(v_in_max_type);
	value = table_getValue(v_in_max_type);
	V_ai_max = (float)value/(float)ratio;

	//kprintf(PORT_DEBUG, "EXT_AI config f_min=%f, f_max=%f, V_min=%f, V_max=%f\r\n", freq_min, freq_max, V_ai_min, V_ai_max);

	return 1;
}


// 12bit adc value : 0 ~ 4095
uint16_t EXT_AI_readADC(void)
{
	return adc_value;
}

int32_t EXT_AI_getFreq(uint16_t adc_val)
{
	//float V_val = (EXT_AIN_ADC_MIN/EXT_AIN_ADC_MAX)*(float)adc_val;
	float V_val = 0.0031*(float)adc_val - 0.1553;
	float freq_calc;
	int32_t freq_l;


	if(V_val >= V_ai_max) { freq_l = (int32_t)(freq_max*10.0 + 0.05); return freq_l; }

	if(V_val <= V_ai_min) { freq_l = (int32_t)(freq_min*10.0 + 0.05); return freq_l; }

	freq_calc = (V_val - V_ai_min)*(freq_max - freq_min)/(V_ai_max - V_ai_min) + freq_min + 0.05; // add round up

	freq_l = (int32_t)(freq_calc*10.0);

	//kprintf(PORT_DEBUG, "EXT_AI_getFreq calc=%f, feq_l=%d, V_val=%f\r\n", freq_calc, freq_l, V_val);

	return freq_l;
}

int8_t EXT_AI_handleAin(void)
{
	uint16_t value;
	int32_t freq, diff=0;
	uint16_t dummy[3] = {0,0,0};
	int8_t status;

	// read config, can be updated during running
	EXT_AI_setConfig();

	// read AI value as 12bit ADC
	value = EXT_AI_readADC();

	// convert to freq value
	freq = EXT_AI_getFreq(value);

	diff = labs(prev_adc_cmd - freq);
	if(diff > F_CMD_DIFF_HYSTERISIS) // over 1Hz
	{
		if(freq <= 10)
		{
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send SPICMD_CTRL_STOP\r\n");
#ifndef SUPPORT_UNIT_TEST
			// send stop cmd
			status = COMM_sendMessage(test_cmd, dummy);
			if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
#endif
		}
		else
		{
			if(state_run_stop == CMD_STOP)
			{
				test_cmd = SPICMD_CTRL_RUN;
				kprintf(PORT_DEBUG, "send SPICMD_CTRL_RUN freq=%d\r\n");
#ifndef SUPPORT_UNIT_TEST
				// send run to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status == COMM_FAILED) { kprintf(PORT_DEBUG, "ERROR EXTIO RUN error! \r\n"); }
#endif
			}
			else
			{
				test_cmd = SPICMD_PARAM_W;
				kprintf(PORT_DEBUG, "send SPICMD_PARAM_W  freq=%d\r\n", freq);
#ifndef SUPPORT_UNIT_TEST
				status = table_setFreqValue(value_type, freq, REQ_FROM_EXTIO);
				if(status == 0) { kprintf(PORT_DEBUG, "set freq=%d to DSP error! \r\n", freq); return 0;}
#endif
			}

		}

		prev_adc_cmd = freq;
	}

	return 1;
}
