/*
 * handler.c
 *
 *  Created on: 2019. 7. 10.
 *      Author: hrjung
 */

#include "includes.h"
#include "proc_uart.h"

#include "table.h"

#include "dsp_comm.h"
#include "nvm_queue.h"
#include "drv_nvm.h"
#include "nvm_queue.h"
#include "error.h"

#include "drv_gpio.h"

#define RUN_STOP_FLAG_IDLE		0
#define RUN_STOP_FLAG_RUN		1
#define RUN_STOP_FLAG_STOP		2

int8_t mb_run_stop_f=0;

int8_t err_read_flag=0;
int8_t err_state_f=0;

extern int16_t state_run_stop;

extern uint32_t motor_on_cnt;
extern uint32_t motor_run_hour;
extern uint32_t device_on_hour;

extern uint32_t motor_run_start_time;
extern uint32_t device_10min_cnt;

extern void TM_setStartRunTime(void);
extern int8_t NVM_setMotorRunTime(uint32_t run_time);
extern int8_t NVM_setDeviceOnTime(uint32_t on_time);



void HDLR_setRunStopFlagModbus(int8_t flag)
{
	mb_run_stop_f = flag;
}

void HDLR_clearRunStopFlagModbus(void)
{
	mb_run_stop_f = 0;
}

int8_t HDLR_readRunStopFlag(void)
{
	return mb_run_stop_f;
}

int8_t HDLR_handleDspError(void)
{
	uint16_t dummy[] = {0,0,0};
	int8_t status;
	int32_t err_code=0;

	if((UTIL_isDspError() || ERR_isErrorState()) && err_state_f == 0)
	{
		if(ERR_isCommError()) // Comm error cannot get response from DSP
		{
			err_code = TRIP_REASON_MCU_COMM_FAIL;
			kputs(PORT_DEBUG, "HDLR_handleDspError get COMM Error\r\n");
			table_updateCommError(err_code);
		}
		else
		{
			// request DSP error info
			status = COMM_sendMessage(SPICMD_REQ_ERR, dummy);
			if(status == COMM_SUCCESS)
			{
				err_code = table_getValue(err_code_0_type);
			}
			kprintf(PORT_DEBUG, "HDLR_handleDspError send SPICMD_REQ_ERR e=%d \r\n", err_code);
		}
		ERR_setErrorState(err_code);
		UTIL_setLED(LED_COLOR_R, 1); //R LED blinking
		err_state_f = 1;
	}

	return 1;
}

int8_t HDLR_readDspStatus(void)
{
	uint16_t dummy[] = {0,0,0};
	int8_t status;

	 if(ERR_isErrorState() == 0) // no error
	 {
		status = COMM_sendMessage(SPICMD_REQ_ST, dummy);
		if(status == COMM_FAILED)
			kputs(PORT_DEBUG, "HDLR_readDspStatus error!!\r\n");

		//kprintf(PORT_DEBUG, "HDLR_readDspStatus send SPICMD_REQ_ST status=%d\r\n", status);
	 }

	 return 1;
}

int8_t HDLR_handleRunStopFlagNFC(void)
{
	int8_t status;
	int32_t run_stop=0;
	uint16_t dummy[3] = {0,0,0};

	status = NVM_getRunStopFlag(&run_stop);
	if(status == 0) return 0;

	switch(run_stop)
	{
	case RUN_STOP_FLAG_RUN:
		// send run to DSP
		status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			status = NVM_clearRunStopFlag();
			TM_setStartRunTime(); // set Run start time
		}
		kprintf(PORT_DEBUG, "RUN Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_STOP:
		// send stop to DSP
		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			status = NVM_clearRunStopFlag();
		}
		kprintf(PORT_DEBUG, "STOP Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_IDLE:
	default:
		break;
	}

	if(status == 0)
	{
		ERR_setErrorState(TRIP_REASON_MCU_INPUT);
	}

	return status;
}

int8_t HDLR_handleRunStopFlagModbus(void)
{
	int8_t status;
	int32_t run_stop=0;
	uint16_t dummy[3] = {0,0,0};

	run_stop = HDLR_readRunStopFlag();
	switch(run_stop)
	{
	case RUN_STOP_FLAG_RUN:
		// send run to DSP
		status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			HDLR_clearRunStopFlagModbus();
			TM_setStartRunTime(); // set Run start time
		}
		kprintf(PORT_DEBUG, "RUN Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_STOP:
		// send stop to DSP
		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			HDLR_clearRunStopFlagModbus();
		}
		kprintf(PORT_DEBUG, "STOP Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_IDLE:
	default:
		break;
	}

	return status;
}

/*
 *
 * 			table API for NFC task
 *
 */
// table data -> EEPROM
// in case of NFC write failure
int8_t HDLR_restoreNVM(void)
{
	PARAM_IDX_t index=value_type;
	int32_t nvm_value, table_value;
	uint8_t nvm_status;
	int8_t status;
	int16_t errflag=0;

	while(index < baudrate_type) // only writable parameter
	{
		osDelay(5);
		nvm_status = NVM_readParam(index, &nvm_value);
		if(nvm_status == 0)
		{
			kprintf(PORT_DEBUG,"ERROR NVM read error index=%d\n", index); errflag++;
		}
		else
		{
			table_value = table_getValue(index);
			if(nvm_value != table_value)
			{
				nvm_status = NVM_writeParam(index, table_value);
				if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM write error index=%d\n", index); errflag++;}

				kprintf(PORT_DEBUG,"HDLR_restoreNVM index=%d, value=%d, status=%d\r\n", index, table_value, nvm_status);
			}
		}
		index++;
	}

	// restore CRC as well
	status = NVM_setCRC();
	if(status == 0) errflag++;

	if(errflag) return 0;

	return 1;
}

// update EEPROM from table
int8_t HDLR_updateParamNVM(void)
{
	int32_t value, nvm_value;
	int8_t empty, status;
	uint8_t nvm_status;
	uint16_t addr;
	int16_t errflag=0;

	do {

		status = NVMQ_dequeueNfcQ(&addr, &value);
		if(status == 0) {kprintf(PORT_DEBUG,"ERROR NfcQ dequeue \r\n"); errflag++;}

		nvm_status = NVM_read(addr, &nvm_value);
		if(nvm_status == 0)
		{
			kprintf(PORT_DEBUG,"ERROR NVM read error addr=0x%x\r\n", addr); errflag++;
		}
		else
		{
			if(nvm_value != value)
			{
				nvm_status = NVM_write(addr, value);
				if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM write error addr=0x%x\r\n", addr); errflag++;}
			}
		}
		osDelay(5);

		empty = NVMQ_isEmptyNfcQ();
	} while(empty == 0); // not empty

	// update CRC
	status = NVM_setCRC();
	if(status == 0) errflag++;

	if(errflag) return 0;

	return 1;
}

// EEPROM is updated by NFC -> inform to table to update
int8_t HDLR_updatebyNfc(void)
{
	PARAM_IDX_t index=value_type;
	int32_t nvm_value;
	uint8_t nvm_status;
	int8_t status;
	int16_t errflag=0;

	while(index <= baudrate_type) // only writable parameter
	{
		osDelay(5);
		nvm_status = NVM_readParam(index, &nvm_value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM read error index=%d \r\n", index); errflag++;}

		if(nvm_value != table_getValue(index))
		{
			status = NVMQ_enqueueTableQ(index, nvm_value);
			if(status == 0) {kprintf(PORT_DEBUG,"ERROR table enqueue error index=%d \r\n", index); errflag++;}
			kprintf(PORT_DEBUG,"HDLR_updatebyNfc index=%d, value=%d, status=%d \r\n", index, nvm_value, status);
		}
		index++;
	}

//	// update CRC is updated by NFC App
//	status = NVM_setCRC();
//	if(status == 0) errflag++;

	// clear NFC tag flag
	status = NVM_clearNfcStatus();
	if(status == -1) {kprintf(PORT_DEBUG, "ERROR clear tag flag \r\n"); return 0;}

	if(errflag) return 0;
	else	return 1;
}

int8_t HDLR_updateTime(uint32_t cur_time)
{
	int8_t status;
	int16_t errflag=0;

	if(cur_time%6 == 0) // 1 hour
	{
		device_on_hour++;
		status = NVM_setDeviceOnTime(device_on_hour);
		if(status == 0) {kprintf(PORT_DEBUG, "ERROR update On time \r\n"); errflag++;}
	}

	if(state_run_stop) // run status, update run time during run_state, lost all under 1 hour
	{
		if(device_10min_cnt - motor_run_start_time >= 6) // over 1 hour
		{
			motor_run_hour++;
			status = NVM_setMotorRunTime(motor_run_hour);
			if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Run time \r\n"); errflag++;}

			motor_run_start_time = device_10min_cnt;
		}
	}

	if(errflag) return 0;
	else	return 1;
}
