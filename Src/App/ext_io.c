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
#include "handler.h"

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

uint8_t mdin_value[EXT_DIN_COUNT]={EXT_DI_INACTIVE,EXT_DI_INACTIVE,EXT_DI_INACTIVE}; // actual DI pin value
COMM_CMD_t test_cmd=0;
uint8_t prev_emergency=EXT_DI_INACTIVE, prev_trip=EXT_DI_INACTIVE;
uint8_t prev_run=EXT_DI_INACTIVE, prev_dir=EXT_DI_INACTIVE;
uint8_t step_cmd=0;

static int32_t di_freq_val=0;
uint8_t mdout_value[EXT_DOUT_COUNT]={0,0};

uint16_t adc_value=0; // analog input value
float V_adc_val=0;
float freq_min=0.0, freq_max=0.0, V_ai_min=0.0, V_ai_max=0.0;
int32_t prev_adc_cmd=1;

uint8_t isConfigured=0; // flag for AIN parameter configured

extern int8_t ain_ready_flag;

extern int16_t state_run_stop; // global run/stop status
extern int16_t state_direction; // global forward/reverse direction status
extern int16_t st_overload;
extern int16_t st_brake;

extern uint32_t timer_100ms;

extern int32_t table_getValue(PARAM_IDX_t index);
extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t option);


void EXT_DI_printConfig(void)
{
	kprintf(PORT_DEBUG, "\r\n Din config run=%d, dir=%d, trip=%d, emerg=%d", m_din.run_pin, m_din.dir_pin, m_din.trip_pin, m_din.emergency_pin);
	kprintf(PORT_DEBUG, "\r\n Din config bit_L=%d, bit_M=%d, bit_H=%d", m_din.bit_L, m_din.bit_M, m_din.bit_H);
	kprintf(PORT_DEBUG, "\r\n Din mdin %d, %d, %d", \
			(mdin_value[0]==EXT_DI_ACTIVE), (mdin_value[1]==EXT_DI_ACTIVE), (mdin_value[2]==EXT_DI_ACTIVE));
//	kprintf(PORT_DEBUG, "\r\n prev_run=%d, run_stop=%d, step=%d di_freq=%d",
//			prev_run, state_run_stop, step_cmd, di_freq_val);
	//kprintf(PORT_DEBUG, "\r\n prev_emerg=%d, prev_trip=%d", prev_emergency, prev_trip);
}

int32_t EXT_getDIValue(void)
{
	int32_t di_val=0;
	int32_t ctrl_in=0;

	ctrl_in = table_getCtrllIn();
	if(ctrl_in == CTRL_IN_Digital || ctrl_in == CTRL_IN_Din_Ain)
	{
		di_val = (int32_t)((((mdin_value[2]&0x01)<<2) | ((mdin_value[1]&0x01)<<1) | (mdin_value[0]&0x01)));
	}
	else
		di_val = 7;

	return (7-di_val); // low active
}

int32_t EXT_getDOValue(void)
{
	int32_t do_val=0;

	do_val = (int32_t)(((mdout_value[1]&0x01)<<1) | (mdout_value[0]&0x01));

	return do_val;
}

int32_t EXT_getAIValue(void)
{
	int32_t ai_val=0;
	int32_t ctrl_in=0;

	ctrl_in = table_getCtrllIn();
	if(ctrl_in == CTRL_IN_Analog_V || ctrl_in == CTRL_IN_Din_Ain)
	{
		ai_val = (int32_t)(V_adc_val*10.0 + 0.5); // voltage value
	}

	return ai_val;
}


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

	if(m_din.bit_L != EXT_DIN_COUNT)
	{
		low = (uint8_t)(mdin_value[m_din.bit_L] == EXT_DI_ACTIVE);
	}

	if(m_din.bit_M != EXT_DIN_COUNT)
	{
		mid = (uint8_t)(mdin_value[m_din.bit_M] == EXT_DI_ACTIVE);
	}

	if(m_din.bit_H != EXT_DIN_COUNT)
	{
		high = (uint8_t)(mdin_value[m_din.bit_H] == EXT_DI_ACTIVE);
	}

	step = ((high<<2)|(mid<<1)|low)&0x07;

	//printf("step = %d, high:mid:low = %d:%d:%d\n", step, high, mid, low);

	return step; // low active
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

	//kprintf(PORT_DEBUG, "set di_pin %d as %d\r\n", index, func_set);

	return 1;
}

#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
void EXT_DI_sendEmergencyCmd(COMM_CMD_t cmd)
{
	int8_t status=0;
	uint16_t dummy[3] = {0,0,0};

	// send DSP emergency_stop start
	status = COMM_sendMessage(cmd, dummy); // for test only
	if(status == 0)
	{
		status = COMM_sendMessage(cmd, dummy);
	}
}
#endif

int8_t EXT_DI_handleEmergency(void)
{
	if(m_din.emergency_pin != EXT_DIN_COUNT)
	{
		// enable emergency stop
		if(mdin_value[m_din.emergency_pin] == EXT_DI_ACTIVE && prev_emergency == EXT_DI_INACTIVE)
		{
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send emergency SPICMD_CTRL_EMER_IN\r\n");
			ERR_setErrorState(TRIP_REASON_MCU_INPUT); // get external trip
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
			EXT_DI_sendEmergencyCmd(SPICMD_CTRL_EMER_IN); // let DSP go to emergency
#endif
			prev_emergency = mdin_value[m_din.emergency_pin];

			return 0;
		}
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
		// disable emergency stop
		else if(mdin_value[m_din.emergency_pin] == EXT_DI_INACTIVE && prev_emergency == EXT_DI_ACTIVE)
		{
			kprintf(PORT_DEBUG, "restore normal from TRIP \r\n");
			ERR_setErrorState(TRIP_REASON_NONE);
			// send DSP emergency_stop end
			EXT_DI_sendEmergencyCmd(SPICMD_CTRL_EMER_OUT);

			prev_emergency = mdin_value[m_din.emergency_pin];
		}
#else
		prev_emergency = mdin_value[m_din.emergency_pin];
#endif

	}

	if(m_din.trip_pin != EXT_DIN_COUNT)
	{
		if(mdin_value[m_din.trip_pin] == EXT_DI_ACTIVE && prev_trip == EXT_DI_INACTIVE)
		{
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send trip SPICMD_CTRL_EMER_IN\r\n");
			ERR_setErrorState(TRIP_REASON_MCU_INPUT); // get external trip
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
			// send DSP emergency_stop start
			EXT_DI_sendEmergencyCmd(SPICMD_CTRL_EMER_IN); // let DSP go to emergency
#endif
			prev_trip = mdin_value[m_din.trip_pin];

			return 0;
		}
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
		else if(mdin_value[m_din.trip_pin] == EXT_DI_INACTIVE && prev_trip == EXT_DI_ACTIVE)
		{
			kprintf(PORT_DEBUG, "restore normal from TRIP \r\n");
			ERR_setErrorState(TRIP_REASON_NONE);

			// send DSP emergency_stop end
			EXT_DI_sendEmergencyCmd(SPICMD_CTRL_EMER_OUT);

			prev_trip = mdin_value[m_din.trip_pin];
		}
#else
		prev_trip = mdin_value[m_din.trip_pin];
#endif
	}

	return 1;
}

int8_t EXI_DI_setMultiFreqValue(void)
{
	int8_t status=1;
	uint8_t step=0;
	int32_t freq_value=0;

	if(EXT_DI_isMultiStepValid())
	{
		step = EXT_DI_convertMultiStep();
		freq_value = table_getValue(multi_val_0_type+step);
		step_cmd = step; // for test

		if(di_freq_val != freq_value) // if current multi-step is step, then apply this change to di_freq_val
		{
			status = table_setFreqValue(value_type, freq_value, REQ_FROM_EXTIO);
			kprintf(PORT_DEBUG, "table_setMultiFreqValue di_freq_val = %d \r\n", freq_value);
			if(status == 0) { kprintf(PORT_DEBUG, "table_setMultiFreqValue error! \r\n"); }
			else
				di_freq_val = freq_value;
		}
	}

	return status;
}

int32_t EXI_DI_getStepValue(int8_t flag)
{
	int8_t status;
	uint8_t step=0;
	int32_t freq_value=0;

	if(EXT_DI_isMultiStepValid())
	{
		step = EXT_DI_convertMultiStep();
		freq_value = table_getValue(multi_val_0_type+step);
		step_cmd = step; // for test
	}
	else
		freq_value = table_getValue(value_type); // if no multi-step is assigned, use value_type

	if(flag == EXT_DI_INITIALIZE)
	{
		HDLR_setStopFlag(0); // stop state
		di_freq_val = freq_value;
		status = table_setFreqValue(value_type, di_freq_val, REQ_FROM_EXTIO);
		kprintf(PORT_DEBUG, "EXI_DI_getStepValue di_freq_val = %d \r\n", di_freq_val);
		if(status == 0) { kprintf(PORT_DEBUG, "EXT_DI_INITIALIZE set multistep error! \r\n"); }
	}

	return freq_value;
}

void EXT_DI_printStatus(void)
{
	if(m_din.run_pin != EXT_DIN_COUNT)
	{
		kprintf(PORT_DEBUG, "\r\n RUN pin pin=%d, prev_run=%d, run_stop=%d ",
				mdin_value[m_din.run_pin], prev_run, state_run_stop);
	}
	if(m_din.dir_pin != EXT_DIN_COUNT)
	{
		kprintf(PORT_DEBUG, "\r\n DIR pin pin=%d, prev_run=%d, direction=%d ",
				mdin_value[m_din.dir_pin], prev_dir, state_direction);
	}
}

int EXT_isDirectionValid(void)
{
	int result = 1;
	int32_t dir_control = table_getValue(dir_domain_type);

	if(m_din.dir_pin == EXT_DIN_COUNT) return 1; // always valid

	switch(dir_control)
	{
	case DIR_FORWARD_ONLY:
		if(mdin_value[m_din.dir_pin] == EXT_DI_ACTIVE)
			result = 0;
		break;

	case DIR_REVERSE_ONLY:
		if(mdin_value[m_din.dir_pin] == EXT_DI_INACTIVE)
			result = 0;
		break;
	}

	return result;
}

int8_t EXT_DI_handleDin(int32_t ctrl_in)
{
	int32_t dir_ctrl;
	int32_t freq_step;
	uint8_t step=0;
	int8_t status, result=1;
	uint16_t dummy[3] = {0,0,0};
	int dir_check=0;

	if(ctrl_in == CTRL_IN_Digital) // only DI control
	{
		//if(!HDLR_isStopInProgress()) // block multistep command till stop completed
		{
			// handle multi-step freq
			freq_step = EXI_DI_getStepValue(EXT_DI_NORMAL);
			if(di_freq_val != freq_step)
			{
				test_cmd = SPICMD_PARAM_W;
#ifndef SUPPORT_UNIT_TEST
				status = table_setFreqValue(value_type, freq_step, REQ_FROM_EXTIO);
				kprintf(PORT_DEBUG, "send multistep=%d, freq=%d  \r\n", step, freq_step);
				if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO set multistep error! \r\n"); }
				else
#endif
					di_freq_val = freq_step;
			}
		}
	}


	// handle run/stop
	if(m_din.run_pin != EXT_DIN_COUNT)
	{
		if(mdin_value[m_din.run_pin] == EXT_DI_INACTIVE
			&& prev_run == EXT_DI_ACTIVE)
		//	&& state_run_stop == CMD_RUN) //RUN -> STOP
		{
			// send STOP cmd
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send SPICMD_CTRL_STOP run_stop=%d\r\n", state_run_stop);
#ifndef SUPPORT_UNIT_TEST
			// send stop to DSP
			status = COMM_sendMessage(test_cmd, dummy);
			if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }

#endif
			HDLR_setStopFlag(1); // start stop
			prev_run = mdin_value[m_din.run_pin];
		}
		else if(mdin_value[m_din.run_pin] == EXT_DI_ACTIVE
				&& prev_run == EXT_DI_INACTIVE)
		//		&& state_run_stop == CMD_STOP) // STOP -> RUN
		{
			dir_check = EXT_isDirectionValid();
			if(dir_check)
			{
				// send run cmd
				test_cmd = SPICMD_CTRL_RUN;
#ifndef SUPPORT_UNIT_TEST
				// send run to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO RUN error! \r\n"); }
#endif
				HDLR_setStopFlag(0);
				prev_run = mdin_value[m_din.run_pin];
			}
			kprintf(PORT_DEBUG, "send SPICMD_CTRL_RUN run_stop=%d dir_chk=%d\r\n", state_run_stop, dir_check);
		}
		else
			result = 0;

		//prev_run = mdin_value[m_din.run_pin];
	}

	// handle direction
	if(m_din.dir_pin != EXT_DIN_COUNT)
	{
		//kprintf(PORT_DEBUG, "CMD_DIR: dir_pin=%d, prev=%d, direct=%d \r\n", mdin_value[m_din.dir_pin], prev_dir, state_direction);
		dir_ctrl = table_getValue(dir_domain_type);
		if(mdin_value[m_din.dir_pin] == EXT_DI_INACTIVE
			&& prev_dir == EXT_DI_ACTIVE)
		//	&& state_direction != CMD_DIR_F) // reverse -> forward
		{
			// send FWD cmd
			if(dir_ctrl != DIR_REVERSE_ONLY)
			{
				// send forward cmd
				test_cmd = SPICMD_CTRL_DIR_F;
				kprintf(PORT_DEBUG, "send SPICMD_CTRL_DIR_F\r\n");
#ifndef SUPPORT_UNIT_TEST
				// send run to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO DIR F error! \r\n"); }
				else
#endif
					prev_dir = mdin_value[m_din.dir_pin];
			}
			else
				result = 0;

		}
		else if(mdin_value[m_din.dir_pin] == EXT_DI_ACTIVE
				&& prev_dir == EXT_DI_INACTIVE)
		//		&& state_direction != CMD_DIR_R)	// forward -> reverse
		{
			// send RVS cmd
			if(dir_ctrl != DIR_FORWARD_ONLY)
			{
				// send reverse cmd
				test_cmd = SPICMD_CTRL_DIR_R;
				kprintf(PORT_DEBUG, "send SPICMD_CTRL_DIR_R\r\n");
#ifndef SUPPORT_UNIT_TEST
				// send run to DSP
				status = COMM_sendMessage(test_cmd, dummy);
				if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO DIR R error! \r\n"); }
				else
#endif
					prev_dir = mdin_value[m_din.dir_pin];
			}
			else
				result = 0;
		}
		else
			result = 0;
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
		if(!table_isMotorStop())
			mdout_value[do_idx] = 1;
		else
			mdout_value[do_idx] = 0;
		break;

	case DOUT_stop:
		if(table_isMotorStop())
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

// when run this function, MCU reset
void EXT_AI_printConfig(void)
{
	//kprintf(PORT_DEBUG, "\r\n AI config f_min=%f, f_max=%f, V_min=%f, V_max=%f", freq_min, freq_max, V_ai_min, V_ai_max);
	//kprintf(PORT_DEBUG, "\r\n AI config f_min=%f", freq_min);
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
	//float V_val = 0.0031*(float)adc_val - 0.1548;
	float V_val = 0.0031*(float)adc_val - 0.2139; // for P4+ using calibrator
	float freq_calc;
	int32_t freq_l;

	V_adc_val = V_val; //get V value from adc value

	if(adc_val <= 60) return 0; // stop for minimum adc value

	if(V_val >= V_ai_max) { freq_l = (int32_t)(freq_max*10.0 + 0.05); return freq_l; }

	if(V_val <= V_ai_min) { freq_l = (int32_t)(freq_min*10.0 + 0.05); return 0; } // below Vmin then stop

	freq_calc = (V_val - V_ai_min)*(freq_max - freq_min)/(V_ai_max - V_ai_min) + freq_min; // add round up

	freq_l = (int32_t)(freq_calc*10.0);

	//kprintf(PORT_DEBUG, "EXT_AI_getFreq calc=%f, feq_l=%d, V_val=%f\r\n", freq_calc, freq_l, V_val);

	return freq_l;
}

int8_t EXT_AI_handleAin(void)
{
	uint16_t value;
	int32_t freq, diff=0;
	uint16_t dummy[3] = {0,0,0};
	int8_t status=1;
	int dir_check=0;

	if(!ain_ready_flag) return 1;

	// read config, can be updated during running
	EXT_AI_setConfig();

	// read AI value as 12bit ADC
	value = EXT_AI_readADC();

	// convert to freq value
	freq = EXT_AI_getFreq(value);

	diff = labs(prev_adc_cmd - freq);
	if(diff > F_CMD_DIFF_HYSTERISIS) // over 1Hz
	{
		if(freq == 0) // in case of below Vmin
		{
			freq = 0; //for stop
			test_cmd = SPICMD_CTRL_STOP;
			kprintf(PORT_DEBUG, "send SPICMD_CTRL_STOP\r\n");
#ifndef SUPPORT_UNIT_TEST
			// send stop cmd
			status = COMM_sendMessage(test_cmd, dummy);
			if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO STOP error! \r\n"); }
			HDLR_setStopFlag(1);
#endif
		}
		else
		{
			if(table_isMotorStop() || HDLR_isStopInProgress())
			{
				dir_check = table_isDirectionValid();
				if(dir_check)
				{
					HDLR_setStopFlag(0);
					test_cmd = SPICMD_CTRL_RUN;
					kprintf(PORT_DEBUG, "send SPICMD_CTRL_RUN \r\n");
#ifndef SUPPORT_UNIT_TEST
					// send run to DSP
					status = COMM_sendMessage(test_cmd, dummy);
					if(status == 0) { kprintf(PORT_DEBUG, "ERROR EXTIO RUN error! \r\n"); }

					//HDLR_setStartRunTime();
#endif
					test_cmd = SPICMD_PARAM_W;
				}
				kprintf(PORT_DEBUG, " send SPICMD_PARAM_W  freq=%d, dir_chk=%d\r\n", freq, dir_check);
#ifndef SUPPORT_UNIT_TEST
				status = table_setFreqValue(value_type, freq, REQ_FROM_EXTIO);
				if(status == 0) { kprintf(PORT_DEBUG, "set freq=%d to DSP error! \r\n", freq); }
#endif
			}
			else
			{
				test_cmd = SPICMD_PARAM_W;
				kprintf(PORT_DEBUG, "time=%d, send SPICMD_PARAM_W  freq=%d\r\n", timer_100ms, freq);
#ifndef SUPPORT_UNIT_TEST
				status = table_setFreqValue(value_type, freq, REQ_FROM_EXTIO);
				if(status == 0) { kprintf(PORT_DEBUG, "set freq=%d to DSP error! \r\n", freq); }
#endif
			}
		}

		if(status != 0) // if error, not update
			prev_adc_cmd = freq;
	}

	return 1;
}

int8_t EXT_handleDAin(int32_t ctrl_in) // accept both DI, AI as control
{
	uint16_t value;
	int32_t freq, diff=0;
	int8_t status, result=1;

	if(!ain_ready_flag) return 1;

	// read config, can be updated during running
	EXT_AI_setConfig();

	// read AI value as 12bit ADC
	value = EXT_AI_readADC();

	// convert to freq value
	freq = EXT_AI_getFreq(value);

	diff = labs(prev_adc_cmd - freq);
	if(diff >= F_CMD_DIFF_HYSTERISIS || freq == 0) // over 1Hz
	{
		if(freq == 0) freq = (int32_t)(freq_min*10.0 + 0.05); // set minumum freq value

		if(prev_adc_cmd != freq) // send minimum value one time
		{
			test_cmd = SPICMD_PARAM_W;
			kprintf(PORT_DEBUG, "time=%d, send SPICMD_PARAM_W  freq=%d\r\n", timer_100ms, freq);
	#ifndef SUPPORT_UNIT_TEST
			status = table_setFreqValue(value_type, freq, REQ_FROM_EXTIO);
			if(status == 0) { kprintf(PORT_DEBUG, "set freq=%d to DSP error! \r\n", freq); }
			else // if error, not update
	#endif
			prev_adc_cmd = freq;
		}
	}

	result = EXT_DI_handleDin(ctrl_in);

	return result;
}
