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
int8_t mb_factory_mode_f=0; // flag for factory mode enabled/disabled

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

#ifdef SUPPORT_PARAMETER_BACKUP
extern uint16_t table_getAddr(PARAM_IDX_t index);
#endif

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

void HDLR_setFactoryModeFlagModbus(int8_t flag)
{
	mb_factory_mode_f = flag;
}

void HDLR_clearFactoryModeFlagModbus(void)
{
	mb_factory_mode_f = 0;
}

int8_t HDLR_isFactoryModeEnabled(void)
{
	return (mb_factory_mode_f == 1);
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
		//UTIL_setLED(LED_COLOR_R, 1); //R LED blinking
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
	static int32_t prev_run_stop=0;
	uint16_t dummy[3] = {0,0,0};

	status = NVM_getRunStopFlag(&run_stop);
	if(status == 0) return 0;

	if(prev_run_stop == run_stop) return 0; //no change

	switch(run_stop)
	{
	case RUN_STOP_FLAG_RUN:
		// send run to DSP
		status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			//status = NVM_clearRunStopFlag();
			prev_run_stop = run_stop;
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
			//status = NVM_clearRunStopFlag();
			prev_run_stop = run_stop;
		}
		kprintf(PORT_DEBUG, "STOP Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_IDLE:
	default:
		break;
	}

	if(status == 0)
	{
		//ERR_setErrorState(TRIP_REASON_MCU_INPUT);
		kprintf(PORT_DEBUG, "ERROR!!  RUN/STOP Flag, status=%d\r\n", status);
	}

	return status;
}

int8_t HDLR_handleRunStopFlagModbus(void)
{
	int8_t status=0;
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

	// clear NFC tag flag
	status = NVM_clearNfcStatus();
	if(status == -1) {errflag++; kprintf(PORT_DEBUG, "ERROR clear tag flag \r\n");}

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

#ifdef SUPPORT_PARAMETER_BACKUP
int8_t mb_backup_mode_f=0;

void HDLR_setBackupFlagModbus(int8_t flag)
{
	mb_backup_mode_f = flag;
}

void HDLR_clearBackupFlagModbus(void)
{
	mb_backup_mode_f = 0;
}

int8_t HDLR_getBackupFlag(void)
{
	return mb_backup_mode_f;
}

int8_t HDLR_isBackupEnabled(void)
{
	return (mb_backup_mode_f != 0);
}

int8_t HDLR_setBackAvailableFlag(int32_t flag)
{
	int8_t nvm_status;
	int32_t addr=0;

	addr = NVM_BACKUP_FLAG_ADDR;
	nvm_status = NVM_read(addr, &flag);
	if(nvm_status == 0) return 0;

	return 1;
}

int HDLR_isBackupAvailable(void)
{
	int8_t nvm_status;
	int32_t addr=0, value=0;

	addr = NVM_BACKUP_FLAG_ADDR;
	nvm_status = NVM_read(addr, &value);
	if(nvm_status == 0) return 0;

	return (value == NVM_BACKUP_AVAILABLE_F);
}

// store table parameter to backup area in EEPROM
int8_t HDLR_backupParameter(void)
{
	PARAM_IDX_t idx;
	int8_t nvm_status;
	int32_t addr=0, value=0;
	int16_t errflag=0;

	for(idx=value_type; idx <= baudrate_type; idx++)
	{
		addr = NVM_BACKUP_START_ADDR + idx*4;
		value = table_getValue(idx);
		nvm_status = NVM_write(addr, value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM backup error idx=0x%x\r\n", idx); errflag++;}

		osDelay(5);
	}
	kprintf(PORT_DEBUG,"NVM backup finished err=%d\r\n", errflag);

	nvm_status = HDLR_setBackAvailableFlag(NVM_BACKUP_AVAILABLE_F);
	if(nvm_status == 0) {kprintf(PORT_DEBUG,"set NVM backup flag error\r\n"); errflag++;}

	if(errflag) return 0;
	else	return 1;
}

// read parameters in backup area, write to parameter table
int8_t HDLR_restoreParameter(void)
{
	PARAM_IDX_t idx;
	int8_t nvm_status;
	int32_t s_addr=0, t_addr=0, value=0;
	int16_t errflag=0;

	for(idx=value_type; idx <= baudrate_type; idx++)
	{
		s_addr = NVM_BACKUP_START_ADDR + idx*4;
		nvm_status = NVM_read(s_addr, &value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR backup NVM read error idx=0x%x\r\n", idx); errflag++;}

		t_addr = table_getAddr(idx);
		nvm_status = NVM_write(t_addr, value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR backup NVM write error idx=0x%x\r\n", idx); errflag++;}

		osDelay(3);
	}

	if(errflag) return 0;
	else	return 1;
}
#endif
