/*
 * dsp_comm.c
 *
 *  Created on: 2019. 6. 11.
 *      Author: hrjung
 */
#include "includes.h"

#include "proc_uart.h"

#include "table.h"
#include "dsp_comm.h"
#include "drv_dsp_spi.h"



#define LENGTH_BUFFER 20
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

extern void printDBG(char *str);

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
	case SPICMD_REQ_ST: 	return 18;
	case SPICMD_CTRL_RUN:
	case SPICMD_CTRL_STOP:
	case SPICMD_CTRL_DIR_F:
	case SPICMD_CTRL_DIR_R:
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
	if(comm_state == COMM_FAILED) { printDBG("[COMM_DSP] SPI write ERROR!"); return comm_state; }

	return comm_state;
}

int8_t COMM_recvfromDSP(int16_t recv_len)
{
	comm_state=COMM_DEFAULT;

	memset(recvMsg, 0, LENGTH_BUFFER*sizeof(uint16_t));
	comm_state = SPI_readDSP(&recvMsg[0], recv_len);
	if(comm_state == COMM_FAILED) { printDBG("[COMM_DSP] SPI read ERROR!"); return comm_state; }

	int8_t isCrcError = SPI_isChecksumError(recvMsg, recv_len);
	if(isCrcError)
	{
		printDBG("[COMM_DSP] CRC ERROR!");
		return COMM_FAILED;
	}

	return COMM_SUCCESS;
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

	t_value = table_getValue(table_idx);

	ratio = table_getRatio(table_idx);
	buf[0] = (uint16_t)table_getDspAddr(table_idx);
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

// return receive value
int32_t COMM_parseValue(int16_t dsp_index, uint16_t *data, int8_t *err)
{
	uint16_t ratio;
	int32_t ret_value=0;
	float value_f;

	*err=0;
	if(dsp_index < value_dsp || dsp_index >= DSP_PARAM_SIZE) { *err=1; return 0; }

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

	comm_state=COMM_DEFAULT;
#ifdef DEBUG_DSP
	kprintf(PORT_DEBUG, "[COMM_DSP] handleMSG\r\n");
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

	switch(cmd)
	{
	case SPICMD_RESP_ACK:
	{
		if(recvMsg[5+0]!=SPI_ACK) NAK_flag = 1;
		comm_state = COMM_SUCCESS;

		//printf("SPICMD_RESP_ACK NAK_flag=%d\n", NAK_flag);
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

#ifdef SUPPORT_NFC_OLD
//		status1 = (int32_t)((state_direction<<8) | state_run_stop); // only use lower 16 bit for modbus
//		status2 = (int32_t)((st_brake<<8) | st_overload);
		table_setStatusValue(run_status1_type, (int32_t)status1);
		table_setStatusValue(run_status2_type, (int32_t)status2);
#else
		table_setStatusValue(run_status_type, (int32_t)state_run_stop);
		table_setStatusValue(dir_status_type, (int32_t)state_direction);
		table_setStatusValue(overload_alarm_type, (int32_t)st_overload);
		table_setStatusValue(shaftbrake_status_type, (int32_t)st_brake);
#endif
		table_setStatusValue(run_freq_type, (int32_t)(10.0*run_freq_index + 0.05) );
		table_setStatusValue(I_rms_type, (int32_t)(10.0*i_rms_index + 0.05) );
		table_setStatusValue(dc_voltage_type, (int32_t)(10.0*dc_voltage_index) );
		table_setStatusValue(ipm_temperature_type, (int32_t)(10.0*ipm_temp_index) );
		table_setStatusValue(mtr_temperature_type, (int32_t)(motor_temp_index) );

#ifdef SUPPORT_NFC_OLD
		// update EEPROM while NFC tagged
		if(NVM_isNfcMonitoring())
		{
			kprintf(PORT_DEBUG, "update Status to EEPROM\n");
			// TODO : update EEPROM

		}

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
		// TODO: update EEPROM
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
		result = COMM_sendtoDSP();
		if(result == COMM_FAILED) continue;

		result = COMM_recvfromDSP(recv_len);
		if(result == COMM_FAILED) continue;

		if(recvMsg[3] != seq_cnt) continue;

		result = COMM_parseMessage();
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

	if(repeat_err)
	{
		printDBG("ERR: COMM_sendCommand repeat error!!");
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

	// read parameter before write, in order to avoid writing same value
	result = COMM_sendCommand(SPICMD_PARAM_R, data);
	if(result == COMM_FAILED) return COMM_FAILED;

	//compare value
	value = table_getValue(DSP_TO_TABLE_IDX[read_idx]);
	if(read_value == value) return COMM_SUCCESS; // same value, not sending

	// send write parameter
	result = COMM_sendCommand(SPICMD_PARAM_W, data);
	if(result == COMM_FAILED) return COMM_FAILED;

	return COMM_SUCCESS;
}

// TODO: MUST check validity of cmd and data before call function, data should be converted to int32 or float
// data[0] : dsp_index
// data[1],[2] : data value
int8_t COMM_sendMessage(COMM_CMD_t cmd, const uint16_t* data)
{
	int8_t result;

	if(cmd == SPICMD_PARAM_W)
		result = COMM_sendParamWrite(data);
	else
		result = COMM_sendCommand(cmd, data);

	return (result == COMM_SUCCESS);
}
