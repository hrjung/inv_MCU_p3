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




int8_t HDLR_handleDspError(void)
{
	uint16_t dummy[] = {0,0,0};
	int8_t status;
	int32_t err_code;
	static int8_t read_flag=0;

	if(UTIL_isDspError() && read_flag == 0)
	{
		// request DSP error info
		status = COMM_sendMessage(SPICMD_REQ_ERR, dummy);
		if(status == COMM_SUCCESS)
		{
			read_flag = 1;
			err_code = table_getValue(err_code_0_type);
			ERR_setErrorState(err_code);
		}
	}

	return 1;
}

int8_t HDLR_handleRunStopFlag(void)
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
			status = MVM_clearRunStopFlag();
		kprintf(PORT_DEBUG, "RUN Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_STOP:
		// send stop to DSP
		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
			status = MVM_clearRunStopFlag();
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
	int16_t errflag=0;

	while(index < PARAM_TABLE_SIZE)
	{
		nvm_status = NVM_readParam(index, &nvm_value);
		if(nvm_status != 0) {kprintf(PORT_DEBUG,"ERROR NVM read error\n"); errflag++;}

		table_value = table_getValue(index);
		if(nvm_value != table_value)
		{
			nvm_status = NVM_writeParam(index, table_value);
			if(nvm_status != 0) {kprintf(PORT_DEBUG,"ERROR NVM write error\n"); errflag++;}
		}
		index++;
	}

	// no need to update CRC

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
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM read error\r\n"); errflag++;}

		if(nvm_value != value)
		{
			nvm_status = NVM_write(addr, value);
			if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM write error\r\n"); errflag++;}
		}

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

	while(index < PARAM_TABLE_SIZE)
	{
		nvm_status = NVM_readParam(index, &nvm_value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM read error index=%d\r\n", index); errflag++;}

		if(nvm_value != table_getValue(index))
		{
			status = NVMQ_enqueueTableQ(index, nvm_value);
			if(status == 0) {kprintf(PORT_DEBUG,"ERROR table enqueue error index=%d\r\n", index); errflag++;}
		}
	}

	// update CRC
	status = NVM_setCRC();
	if(status == 0) errflag++;

	// clear NFC tag flag
	status = NVM_clearNfcStatus();
	if(status == -1) {kprintf(PORT_DEBUG, "ERROR clear tag flag \r\n"); return 0;}

	if(errflag) return 0;
	else	return 1;
}
