/*
 * dsp_comm.c
 *
 *  Created on: 2019. 6. 11.
 *      Author: hrjung
 */
#include "main.h"

#include "includes.h"

#include "proc_uart.h"

#include "table.h"
#include "dsp_comm.h"
#include "drv_dsp_spi.h"
#include "drv_gpio.h"
#include "drv_nvm.h"
#include "error.h"


#define LENGTH_BUFFER 30	// 20 -> 30 for add torque value at status response
static int8_t comm_state;
STATIC uint16_t sendMsg[LENGTH_BUFFER ];
STATIC uint16_t recvMsg[LENGTH_BUFFER ];
STATIC int16_t seq_cnt = 1;

STATIC int16_t read_idx;
STATIC int32_t read_value;
static int8_t NAK_flag = 0;

int16_t state_run_stop = 0;
int16_t state_direction = 0;
int16_t st_overload = 0;
int16_t st_brake = 0;
int16_t gear_ratio = 1;

int16_t comm_err_cnt=0;

//RTC_TimeTypeDef sTime;
//extern RTC_HandleTypeDef hrtc;

//extern int16_t test_run_stop_f;
uint8_t time_cnt=0;

extern int8_t NVM_isNfcMonitoring(void);

extern TABLE_DSP_PARAM_t table_getDspAddr(PARAM_IDX_t index);


const static PARAM_IDX_t DSP_TO_TABLE_IDX[] =
{
		value_type,
		freq_max_type,
		accel_time_type,
		decel_time_type,
		acc_base_set_type,

		dir_cmd_type,
		energy_save_type,
		pwm_freq_type,
		jmp_enable0_type,
		jmp_enable1_type,

		jmp_enable2_type,
		jmp_low0_type,
		jmp_low1_type,
		jmp_low2_type,
		jmp_high0_type,

		jmp_high1_type,
		jmp_high2_type,
		brake_type_type,
		brake_freq_type,
		dci_brk_freq_type,

		dci_brk_hold_type,
		dci_brk_time_type,
		dci_brk_rate_type,
		ovl_warn_limit_type,
		ovl_warn_dur_type,

		ovl_enable_type,
		ovl_trip_limit_type,
		ovl_trip_dur_type,
		regen_duty_type,
		regen_band_type,

		fan_onoff_type,
		motor_type_type,
};

const static char* STR_SPICMD_CTRL_RUN  = "CTRL_RUN";
const static char* STR_SPICMD_CTRL_STOP = "CTRL_STOP";
const static char* STR_SPICMD_CTRL_DIR_F = "CTRL_DIR_F";
const static char* STR_SPICMD_CTRL_DIR_R = "CTRL_DIR_R";
const static char* STR_SPICMD_PARAM_W = "PARAM_W";
const static char* STR_SPICMD_PARAM_R = "PARAM_R";
const static char* STR_SPICMD_REQ_ST = "REQ_ST";
const static char* STR_SPICMD_REQ_ERR = "REQ_ERR";
const static char* STR_SPICMD_RESP_ACK = "RESP_ACK";
const static char* STR_SPICMD_RESP_ST = "RESP_ST";
const static char* STR_SPICMD_RESP_ERR = "RESP_ERR";
const static char* STR_SPICMD_RESP_PARAM = "RESP_PARAM";
const static char* STR_SPICMD_TEST_CMD = "TEST_CMD";
const static char* STR_UNKNOWN_CODE_ERROR = "UNKNOWN_CODE_ERROR";

const char* COMM_getCMDString(COMM_CMD_t cmdCode)
{
	switch(cmdCode)
	{
	case SPICMD_CTRL_RUN:		return STR_SPICMD_CTRL_RUN;
	case SPICMD_CTRL_STOP:		return STR_SPICMD_CTRL_STOP;
	case SPICMD_CTRL_DIR_F:		return STR_SPICMD_CTRL_DIR_F;
	case SPICMD_CTRL_DIR_R:		return STR_SPICMD_CTRL_DIR_R;
	case SPICMD_PARAM_W:		return STR_SPICMD_PARAM_W;
	case SPICMD_PARAM_R:		return STR_SPICMD_PARAM_R;
	case SPICMD_REQ_ST:			return STR_SPICMD_REQ_ST;
	case SPICMD_REQ_ERR:		return STR_SPICMD_REQ_ERR;
	case SPICMD_RESP_ACK:		return STR_SPICMD_RESP_ACK;
	case SPICMD_RESP_ST:		return STR_SPICMD_RESP_ST;
	case SPICMD_RESP_ERR:		return STR_SPICMD_RESP_ERR;
	case SPICMD_RESP_PARAM:		return STR_SPICMD_RESP_PARAM;
	case SPICMD_TEST_CMD:		return STR_SPICMD_TEST_CMD;
	default :					return STR_UNKNOWN_CODE_ERROR;

	}
}

int8_t SPI_isChecksumError(uint16_t* data, int16_t length)
{
	int idx;
	uint16_t sum = 0;
	uint16_t crc = 0;

	for(idx =0; idx< length-1; idx ++)
	{
		sum+=0x0FFFF&data[idx];
	}
	crc = 0x0FFFF&data[length-1];

	return crc != sum;
}

int16_t COMM_getRecvLength(COMM_CMD_t cmd)
{

	switch(cmd)
	{
#ifdef SUPPORT_STATUS_TORQUE
	case SPICMD_REQ_ST: 	return 22; // 18 -> 22 for add torque value at status response
#else
	case SPICMD_REQ_ST: 	return 18;
#endif
	case SPICMD_CTRL_RUN:
	case SPICMD_CTRL_STOP:
	case SPICMD_CTRL_DIR_F:
	case SPICMD_CTRL_DIR_R:
	case SPICMD_TEST_CMD:
	case SPICMD_PARAM_W:	return 7;
	case SPICMD_PARAM_R:	return 9;
	case SPICMD_REQ_ERR:	return 12;

	case SPICMD_RESP_ACK:
	case SPICMD_RESP_PARAM:
	case SPICMD_RESP_ST:
	case SPICMD_RESP_ERR:
	default:				return 0;
	}
}

// MUST check validity of cmd and data before call function
// data[0] : dsp_index
// data[1],[2] : data value
int8_t COMM_generateMessage(COMM_CMD_t cmd, const uint16_t* data)
{
	//printf("make MSG\r\n");
	comm_state=COMM_DEFAULT;

	sendMsg[0] = 0xAAAA;
	sendMsg[1] = 0x5555;
	sendMsg[3] = seq_cnt;
	sendMsg[4] = cmd;

	switch(cmd)
	{
	case SPICMD_CTRL_RUN:
	case SPICMD_CTRL_STOP:
	case SPICMD_CTRL_DIR_F:
	case SPICMD_CTRL_DIR_R:
	case SPICMD_REQ_ST:
	case SPICMD_REQ_ERR:
#ifdef DEBUG_DSP
		kprintf(PORT_DEBUG, "[COMM_DSP] COMM_generateMessage : cmd = %d\r\n", (int)sendMsg[4]);
#endif
		sendMsg[2] = 6;
		comm_state = COMM_SUCCESS;
		break;

	case SPICMD_PARAM_W:
		sendMsg[2] = 9;
		sendMsg[5] = data[0];
		sendMsg[6] = data[1];
		sendMsg[7] = data[2];

#ifdef DEBUG_DSP
		//int* valueInt = (int*)&data[1];
		//float* valueFloat = (float*)&data[1];
		kprintf(PORT_DEBUG, "[COMM_DSP] COMM_generateMessage : WRITE idx = %d, data[0] = %d, data[1] = %d, data[2] = %d\r\n",
				sendMsg[2], sendMsg[5], sendMsg[6], sendMsg[7]);
#endif
		comm_state = COMM_SUCCESS;
		break;

	case SPICMD_TEST_CMD:
		sendMsg[2] = 7;
		sendMsg[5] = data[0]; // test cmd
//		sendMsg[6] = data[1];
//		sendMsg[7] = data[2];

#ifdef DEBUG_DSP
		//int* valueInt = (int*)&data[1];
		//float* valueFloat = (float*)&data[1];
		kprintf(PORT_DEBUG, "[COMM_DSP] COMM_generateMessage : WRITE idx = %d, data[0] = %d, data[1] = %d, data[2] = %d\r\n",
				sendMsg[2], sendMsg[5], sendMsg[6], sendMsg[7]);
#endif
		comm_state = COMM_SUCCESS;
		break;

	case SPICMD_PARAM_R:
		sendMsg[2] = 7;
		sendMsg[5] = data[0];
		comm_state = COMM_SUCCESS;
		break;

	case SPICMD_RESP_ACK:
	case SPICMD_RESP_ST:
	case SPICMD_RESP_ERR:
	case SPICMD_RESP_PARAM:
	default:
		kprintf(PORT_DEBUG, "[COMM_DSP] Wrong type sender message : cmd = %s\r\n", COMM_getCMDString(cmd));
		comm_state = COMM_FAILED;
		break;
	}

	return comm_state;
}

int8_t COMM_sendtoDSP(void)
{
	comm_state=COMM_DEFAULT;

	comm_state = SPI_writeDSP(&sendMsg[0], sendMsg[2]);
	if(comm_state == COMM_FAILED) { kputs(PORT_DEBUG, "[COMM_DSP] SPI write ERROR!\r\n"); return comm_state; }

	return comm_state;
}

int8_t COMM_recvfromDSP(int16_t recv_len)
{
	comm_state=COMM_DEFAULT;

	memset(recvMsg, 0, LENGTH_BUFFER*sizeof(uint16_t));
	comm_state = SPI_readDSP(&recvMsg[0], recv_len);
	if(comm_state == COMM_FAILED) { kputs(PORT_DEBUG, "[COMM_DSP] SPI read ERROR!\r\n"); return comm_state; }

	int8_t isCrcError = SPI_isChecksumError(recvMsg, recv_len);
	if(isCrcError)
	{
		kputs(PORT_DEBUG, "[COMM_DSP] CRC ERROR!");
		return COMM_FAILED;
	}

	return COMM_SUCCESS;
}

int8_t COMM_setMultiStepFreq(PARAM_IDX_t table_idx, uint16_t *buf)
{
	int32_t t_value;
	float value_f;

	buf[0] = value_dsp;

	t_value = table_getValue(table_idx);
	value_f = (float)((float)t_value/10.0);
	memcpy(&buf[1], &value_f, sizeof(float));

	return 1;
}

int8_t COMM_setAnalogFreq(int32_t freq, uint16_t *buf)
{
	float value_f;

	buf[0] = value_dsp;

	value_f = (float)((float)freq/10.0);
	memcpy(&buf[1], &value_f, sizeof(float));

	return 1;
}

// set data for SPICMD_PARAM_W command
// data[0] = index
// data[1], data[2] : int32 or float value
int8_t COMM_convertValue(PARAM_IDX_t table_idx, uint16_t *buf)
{
	uint16_t ratio;
	int8_t ret_value=0;
	int32_t t_value;
	float value_f;

	if(table_idx < value_type || table_idx >= PARAM_TABLE_SIZE) { return 0; }

	ratio = table_getRatio(table_idx);
	t_value = table_getValue(table_idx);

	buf[0] = (uint16_t)table_getDspAddr((PARAM_IDX_t)table_idx);
	if(ratio == 1)
	{
		memcpy(&buf[1], &t_value, sizeof(int32_t));
	}
	else if(ratio == 10)
	{
		value_f = (float)((float)t_value/(float)ratio);
		memcpy(&buf[1], &value_f, sizeof(float));
	}
	else
	{
		ret_value = 0;
		kprintf(PORT_DEBUG,"ERR convert: wrong ratio=%d, index=%d\r\n", ratio, table_idx);
	}

	return ret_value;
}

// return receive value as int32
int32_t COMM_parseValue(int16_t dsp_index, uint16_t *data, int8_t *err)
{
	uint16_t ratio;
	int32_t ret_value=0;
	float value_f;

	*err=0;
	if(dsp_index < value_dsp || dsp_index >= DSP_PARAM_SIZE) { *err=1; return 0; }

	//kprintf(PORT_DEBUG, "COMM_parseValue dsp_index=%d \r\n", dsp_index);
	ratio = table_getRatio(DSP_TO_TABLE_IDX[dsp_index]);
	if(ratio == 1)
	{
		memcpy(&ret_value, &data[0], sizeof(int32_t));
	}
	else if(ratio == 10)
	{
		memcpy(&value_f, &data[0], sizeof(float));
		ret_value = (int32_t)(value_f*(float)ratio);
	}
	else
	{
		*err = 1;
		kprintf(PORT_DEBUG,"ERR parse: wrong ratio=%d, index=%d\r\n", ratio, dsp_index);
	}

	return ret_value;
}


int8_t COMM_getNakFlag(void)
{
	return NAK_flag;
}

uint32_t COMM_getReadValue(void)
{
	return read_value;
}

int8_t COMM_parseMessage(void)
{
#ifdef SUPPORT_NFC_OLD
	int32_t status1=0, status2=0;
#endif

	float i_rms_index = 0.f;
	float run_freq_index = 0.f;
	float dc_voltage_index = 0.f;
	float ipm_temp_index = 0.f;
	int32_t motor_temp_index = 0;
#ifdef SUPPORT_STATUS_TORQUE
	float torque_index = 0.f;
	float torque_percent_index = 0.f;
	static float prev_torque=0.f;
#endif

	comm_state=COMM_DEFAULT;
#ifdef DEBUG_DSP
	kprintf(PORT_DEBUG, "[COMM_DSP] COMM_parseMessage\r\n");
#endif
	if((recvMsg[0] != 0xAAAA) || (recvMsg[1] != 0x5555))
	{
		kprintf(PORT_DEBUG, "[COMM_DSP] ERR: COMM_parseMessage : Broken packet, recvMsg = 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
				recvMsg[0], recvMsg[1], recvMsg[2], recvMsg[3], recvMsg[4], recvMsg[5]);

		return COMM_FAILED;
	}


//	int16_t length = recvMsg[2];
//	int16_t recv_seqNO = recvMsg[3];
	int16_t cmd = recvMsg[4];
	//kprintf(PORT_DEBUG, "COMM_parseMessage cmd=0x%x \r\n", cmd);
	switch(cmd)
	{
	case SPICMD_RESP_ACK:
	{
		if(recvMsg[5] != SPI_ACK) NAK_flag = 1;
		comm_state = COMM_SUCCESS;

		//kprintf(PORT_DEBUG, "SPICMD_RESP_ACK NAK_flag=%d\r\n", NAK_flag);
		break;
	}

	case SPICMD_RESP_ST:
	{
#ifndef TEST_ST_READ
		/*read from DSP*/

//		state_run_stop = recvMsg[5]&0x01; 		//run/stop
//		state_direction = (recvMsg[5]>>8)&0x01; 	// forward/backward
//
//		st_overload = (recvMsg[5+1])&0x01;		// overload on/off
//		st_brake = (recvMsg[5+1]>>8)&0x01;  	// external brake on/off

		status1 = recvMsg[5];
		status2 = recvMsg[6];

		memcpy(&i_rms_index, &recvMsg[5+2], sizeof(float));
		memcpy(&run_freq_index, &recvMsg[5+4], sizeof(float));
		memcpy(&dc_voltage_index, &recvMsg[5+6], sizeof(float));
		memcpy(&ipm_temp_index, &recvMsg[5+8], sizeof(float));
		memcpy(&motor_temp_index, &recvMsg[5+10], sizeof(long));
#ifdef SUPPORT_STATUS_TORQUE
		memcpy(&torque_index, &recvMsg[5+12], sizeof(float));
		memcpy(&torque_percent_index, &recvMsg[5+14], sizeof(float));

		// avoid invalid torque
		if(torque_index < 0.0 || torque_index > 50.0) torque_index = prev_torque;
#endif

#else
		/*for test : count-up values*/

		state_run_stop += 1;
		state_direction += 2;
		i_rms_index += 1.f;
		run_freq_index += 2.f;
		dc_voltage_index += 1.f;
		ipm_temp_index += 2.f;
		motor_temp_index = 1.f;
#endif

		//kprintf(PORT_DEBUG, "SPICMD_RESP_ST run_freq=%d \r\n", (int32_t)run_freq_index);
#ifdef SUPPORT_NFC_OLD
//		status1 = (int32_t)((state_direction<<8) | state_run_stop); // only use lower 16 bit for modbus
//		status2 = (int32_t)((st_brake<<8) | st_overload);
#if 0
		if(test_run_stop_f) // set dummy status value for test
		{
			status1 = 1;//set run state for test
			table_setStatusValue(run_status1_type, (int32_t)status1, REQ_FROM_DSP);
		}
		else
#endif
			table_setStatusValue(run_status1_type, (int32_t)status1, REQ_FROM_DSP);
		table_setStatusValue(run_status2_type, (int32_t)status2, REQ_FROM_DSP);
#else
		table_setStatusValue(run_status_type, (int32_t)state_run_stop);
		table_setStatusValue(dir_status_type, (int32_t)state_direction);
		table_setStatusValue(overload_alarm_type, (int32_t)st_overload);
		table_setStatusValue(shaftbrake_status_type, (int32_t)st_brake);
#endif
		run_freq_index = (10.0*run_freq_index/(float)gear_ratio + 0.05); // apply gear_ratio for freq
		table_setStatusValue(run_freq_type, (int32_t)run_freq_index, REQ_FROM_DSP);
		table_setStatusValue(I_rms_type, (int32_t)(10.0*i_rms_index + 0.05), REQ_FROM_DSP);
		table_setStatusValue(dc_voltage_type, (int32_t)(10.0*dc_voltage_index), REQ_FROM_DSP);
		table_setStatusValue(ipm_temperature_type, (int32_t)(10.0*ipm_temp_index + 0.05), REQ_FROM_DSP);
		table_setStatusValue(mtr_temperature_type, (int32_t)(motor_temp_index), REQ_FROM_DSP);
#ifdef SUPPORT_STATUS_TORQUE
		table_setStatusValue(torque_value_type, (int32_t)(10.0*torque_index + 0.05), REQ_FROM_DSP);
		table_setStatusValue(torque_percent_type, (int32_t)(10.0*torque_percent_index + 0.05), REQ_FROM_DSP);
		prev_torque = torque_index;
#endif
		table_setExtStatusValue();

		//kprintf(PORT_DEBUG, "SPICMD_RESP_ST run_freq=%d freq=%f \r\n", (int32_t)run_freq_index, (int32_t)run_freq_f);
#ifdef SUPPORT_NFC_OLD
		// update EEPROM while NFC tagged
		if(NVM_isNfcMonitoring())
		{
			kprintf(PORT_DEBUG, "update Status to EEPROM\r\n");
			if(table_setStatusDSP() == 0)
				kprintf(PORT_DEBUG, "ERROR in update Status to EEPROM\r\n");

			NVM_clearNfcMonitoring();
			UTIL_setLED(LED_COLOR_G, 0);
		}
#if 0 // for status log
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		time_cnt = sTime.Seconds;
		kprintf(PORT_DEBUG, "cnt=%d : SPICMD_RESP_ST freq=%d \r\n",time_cnt, (int32_t)(10.0*run_freq_index + 0.05));
#endif

#else
		// TODO: update mailbox
#endif


/*
#ifdef DEBUG_DSP
		printf("[COMM_DSP] RESP_ST : len = %d, seqNO= %d,  st_stop[%d], st_reverse[%d]\r\n",
				(int)length, (int)seqNO, (int)state_run_stop, (int)state_direction );
		printf("i_rms[%d] run_freq[%d] dc_volt[%d] ipm_temp[%d] motor_temp[%d]\r\n",
				i_rms_index, run_freq_index, dc_voltage_index, ipm_temp_index, motor_temp_index);
#endif
*/
		comm_state = COMM_SUCCESS;
		break;
	}

	case SPICMD_RESP_ERR:
	{
		uint16_t err_code;
		uint16_t run_state;
		float current, freq;

		err_code = recvMsg[5];
		run_state = recvMsg[5+1];
		memcpy(&current, &recvMsg[5+2], sizeof(float));
		memcpy(&freq, &recvMsg[5+4], sizeof(float));
#ifdef DEBUG_DSP
		kprintf(PORT_DEBUG, "[COMM_DSP] RESP_ERR : err_code[%d], run_state[%d], freq[%d], current[%d]\r\n",
						(int)err_code, (int)run_state, *curr, *freq);
		//printf("[COMM_DSP] RESP_ERR : err_code[%d], run_state[%d]\r\n",(int)err_code, (int)run_state);
#endif

		table_updateErrorDSP(err_code, run_state, current, freq);

		comm_state = COMM_SUCCESS;
		break;
	}

	case SPICMD_RESP_PARAM:
	{
		int8_t index_err=0;

		read_idx = recvMsg[5+0];
		read_value = COMM_parseValue(read_idx, &recvMsg[5+1], &index_err);
		if(index_err)
			comm_state = COMM_FAILED;
		else
			comm_state = COMM_SUCCESS;
#ifdef DEBUG_DSP
		kprintf(PORT_DEBUG, "[COMM_DSP] RESP_PARAM : idx[%d], val[%d]\r\n",
				(int)read_idx, (int)read_value);
#endif

		break;
	}

	case SPICMD_REQ_ST:
	case SPICMD_REQ_ERR:
	case SPICMD_CTRL_RUN:
	case SPICMD_CTRL_STOP:
	case SPICMD_CTRL_DIR_F:
	case SPICMD_CTRL_DIR_R:
	case SPICMD_PARAM_W:
	case SPICMD_PARAM_R:
	default:
		kprintf(PORT_DEBUG, "[COMM_DSP] ERR: Wrong type recv message: cmd = %s\r\n", COMM_getCMDString(cmd));
		comm_state = COMM_FAILED;
		break;
	}

	return comm_state;
}

// other command except SPICMD_PARAM_W command
int8_t COMM_sendCommand(COMM_CMD_t cmd, const uint16_t* data)
{
	int i, rep_cnt=SPI_REPEAT_CNT;
	int8_t result;
	int16_t recv_len;
	int8_t repeat_err=1;

	result = COMM_generateMessage(cmd, data);
	if(result == COMM_FAILED) return COMM_FAILED;

	recv_len = COMM_getRecvLength(cmd);
	if(recv_len == 0) return COMM_FAILED;

	for(i=0; i<rep_cnt; i++)
	{
		result = COMM_sendtoDSP(); repeat_err=2;
		if(result == COMM_FAILED) continue;

		result = COMM_recvfromDSP(recv_len); repeat_err=3;
		if(result == COMM_FAILED) continue;

		repeat_err=4;
		if(recvMsg[3] != seq_cnt) continue;

		result = COMM_parseMessage(); repeat_err=5;
		if(result == COMM_FAILED) continue;

		if(NAK_flag == 1) // DSP did not recognize cmd
		{
			NAK_flag=0; // clear and retry
			continue;
		}

		// all pass OK
		repeat_err=0;
		break;
	}

	if(repeat_err || NAK_flag)
	{
		kprintf(PORT_DEBUG, "ERR: COMM_sendCommand repeat error %d or NAK=%d !!\r\n", repeat_err, NAK_flag);
		//ERR_setErrorState(TRIP_REASON_MCU_COMM_FAIL);
		return COMM_FAILED;
	}

	seq_cnt++;
	return COMM_SUCCESS;
}

// only for SPICMD_PARAM_W command
int8_t COMM_sendParamWrite(const uint16_t* data)
{
	int32_t value;
	int8_t result;

#if 0
	// read parameter before write, in order to avoid writing same value
	result = COMM_sendCommand(SPICMD_PARAM_R, data);
	if(result == COMM_FAILED) return COMM_FAILED;

	//compare value
	value = table_getValue(DSP_TO_TABLE_IDX[read_idx]);
	if(read_value == value) return COMM_SUCCESS; // same value, not sending
#endif
	// send write parameter
	result = COMM_sendCommand(SPICMD_PARAM_W, data);
	if(result == COMM_FAILED) return COMM_FAILED;

	return COMM_SUCCESS;
}

void COMM_handleError(int8_t result)
{
	if(result == COMM_SUCCESS)
	{
		if(comm_err_cnt > 0) comm_err_cnt--;
	}
	else if(result == COMM_FAILED)
	{
		comm_err_cnt++;
		if(comm_err_cnt > COMM_ERR_COUNT_LIMIT)
			ERR_setErrorState(TRIP_REASON_MCU_COMM_FAIL);
	}
}

// TODO: MUST check validity of cmd and data before call function, data should be converted to int32 or float
// data[0] : dsp_index
// data[1],[2] : data value
int8_t COMM_sendMessage(COMM_CMD_t cmd, const uint16_t* data)
{
	int8_t result;

#if 0 // don't need to read DSP to compare same value, just send
	if(cmd == SPICMD_PARAM_W)
		result = COMM_sendParamWrite(data);
	else
#endif
		result = COMM_sendCommand(cmd, data);

	COMM_handleError(result);

	return (result == COMM_SUCCESS);
}

#if 0
// only used at inverter initialize to sync motor parameter
int8_t COMM_sendMotorType(void)
{
	PARAM_IDX_t table_idx;
	int8_t status;
	uint16_t buf[3]={0,0,0};

	// send motor type
	table_idx = motor_type_type;
	COMM_convertValue(table_idx, buf);

	status = COMM_sendMessage(SPICMD_PARAM_W, buf);
	kprintf(PORT_DEBUG, "COMM motor type: status=%d, idx=%d, value=%d\r\n", \
			status, (int)table_idx, (int)table_getValue(table_idx));

	return status;
}
#endif
// read motor type from DSP at startup
int32_t COMM_getMotorType(int8_t *status)
{
	int8_t result = COMM_SUCCESS;
	uint16_t buf[3]={0,0,0};

	buf[0] = (uint16_t)table_getDspAddr((PARAM_IDX_t)motor_type_type);
	result = COMM_sendCommand(SPICMD_PARAM_R, buf);
	if(result == COMM_FAILED) *status = 0;

	return read_value;
}

int8_t COMM_sendTestCmd(uint16_t test_cmd)
{
	int8_t status;
	uint16_t buf[3]={0,0,0};

	buf[0] = test_cmd;
	buf[1] = 0, buf[2] = 0;
	status = COMM_sendMessage(SPICMD_TEST_CMD, buf);
	kprintf(PORT_DEBUG, "SPICMD_TEST_CMD: status=%d, test_cmd=%d \r\n", status, test_cmd);

	return status;
}

// for jig test only
int32_t COM_getReadValue(void)
{
	//int32_t value;
	int8_t result;
	uint16_t buf[3]={0,0,0};

	buf[0] = (uint16_t)table_getDspAddr((PARAM_IDX_t)value_type);
	result = COMM_sendCommand(SPICMD_PARAM_R, buf);
//	if(result == COMM_FAILED) return COMM_FAILED;

	return read_value;
}
