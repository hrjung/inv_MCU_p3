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
#include "handler.h"

#include "drv_gpio.h"


int8_t mb_run_stop_f=0;
int8_t mb_factory_mode_f=0; // flag for factory mode enabled/disabled

int8_t err_read_flag=0;
int8_t err_state_f=0;

uint32_t motor_run_cnt=0;
uint32_t motor_run_start_time=0;

uint8_t stopping_enabled = 0; // set 1 from sending STOP command till state updated to STOP

extern int16_t state_run_stop;

extern uint32_t motor_on_cnt;
extern uint32_t motor_run_hour;
extern uint32_t device_on_hour;

extern uint32_t motor_run_start_time;
extern uint32_t device_min_cnt;
extern uint32_t run_minutes;
extern uint32_t dev_start_time;

extern uint8_t param_init_requested_f;

void HDLR_saveMotorRunTime(void);

extern int8_t NVM_setMotorRunTime(uint32_t run_time);
extern int8_t NVM_setDeviceOnTime(uint32_t on_time);
extern int8_t NVM_readRunStopSysFlag(void);

extern int32_t table_getInitValue(PARAM_IDX_t index);
extern int8_t table_initStatusError(uint16_t index);
extern int table_checkValidityNFC(PARAM_IDX_t idx, int32_t value);

extern int8_t table_getMotorType(void);
extern int8_t table_updateFwVersion(void);

extern TABLE_DSP_PARAM_t table_getDspAddr(PARAM_IDX_t index);
extern uint16_t table_getAddr(PARAM_IDX_t index);

#ifdef SUPPORT_TASK_WATCHDOG
extern void main_kickWatchdogNFC(void);
#endif

#ifdef SUPPORT_FORCE_RESET
extern int8_t main_SwReset(int flag);
#endif

#ifdef SUPPORT_PARAMETER_BACKUP
extern uint16_t table_getAddr(PARAM_IDX_t index);
#endif

void HDLR_setStopFlag(uint8_t flag)
{
	stopping_enabled = flag;
}

int HDLR_isStopInProgress(void)
{
	return (stopping_enabled && state_run_stop == CMD_RUN);
}

int HDLR_setRunStopFlagModbus(int8_t flag)
{
	int result = 1;
	if(table_isDirectionValid())
		mb_run_stop_f = flag;
	else
		result = 0;

	return result;
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

void HDLR_setStartRunTime(void)
{
	int8_t status;

	//if(state_run_stop == CMD_RUN) return; // already run state

	motor_run_start_time = device_min_cnt - run_minutes;

	motor_run_cnt++;
	status = NVM_setMotorRunCount(motor_run_cnt);
	if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Run Count \r\n"); }
	NVMQ_enqueueTableQ(motor_on_cnt_type, (int32_t)motor_run_cnt);

	kprintf(PORT_DEBUG, "start Run time %d \r\n", device_min_cnt);
}

int8_t HDLR_handleDspError(void)
{
	uint16_t dummy[] = {0,0,0};
	int8_t status;
	int32_t err_code=0, run_stop=0;

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
#ifndef SUPPORT_UNIT_TEST
			// request DSP error info
			status = COMM_sendMessage(SPICMD_REQ_ERR, dummy);
			if(status == COMM_SUCCESS)
			{
				err_code = table_getValue(err_code_1_type);
			}
			else
			{
				osDelay(50);
				status = COMM_sendMessage(SPICMD_REQ_ERR, dummy); // try again
				if(status == COMM_SUCCESS)
				{
					err_code = table_getValue(err_code_1_type);
				}
			}
#endif

			kprintf(PORT_DEBUG, "HDLR_handleDspError send SPICMD_REQ_ERR e=%d \r\n", err_code);
		}
		ERR_setErrorState(err_code);
		//UTIL_setLED(LED_COLOR_R, 1); //R LED blinking

		err_state_f = 1; // only one error accepted

		status = NVM_getRunStopFlag(&run_stop);
		if(run_stop != 0 || status == 0)
			NVM_clearRunStopFlag(); // clear run/stop flag at trip condition
	}

	return 1;
}

int8_t HDLR_readDspStatus(void)
{
	uint16_t dummy[] = {0,0,0};
	int8_t status;

	 if(!ERR_isCommError()) // no comm error
	 {
#ifndef SUPPORT_UNIT_TEST
		status = COMM_sendMessage(SPICMD_REQ_ST, dummy);
		if(status == COMM_FAILED)
			kputs(PORT_DEBUG, "HDLR_readDspStatus error!!\r\n");
#endif

		//kprintf(PORT_DEBUG, "HDLR_readDspStatus send SPICMD_REQ_ST status=%d\r\n", status);
	 }

	 return 1;
}

int8_t HDLR_restoreRunStopFlagNFC(void)
{
	int8_t status;
	int32_t run_stop=0;

	status = NVM_getRunStopFlag(&run_stop);
	if(status == 0) return 0;

	if(run_stop != RUN_STOP_FLAG_IDLE)
	{
		NVM_clearRunStopFlag();
	}

	return 1;
}

int8_t HDLR_handleRunStopFlagNFC(void)
{
	int8_t status=1;
	int32_t run_stop=0;
	static int32_t prev_run_stop=0;
	uint16_t dummy[3] = {0,0,0};
	int dir_check=0;

	if(table_isDirectionValid() == 0) return 0;

	run_stop = NVM_readRunStopSysFlag();

	if(prev_run_stop == run_stop) return 0; //no change

	switch(run_stop)
	{
	case RUN_STOP_FLAG_RUN:
		// send run to DSP
#ifndef SUPPORT_UNIT_TEST

		dir_check = table_isDirectionValid();
		if(dir_check)
		{
			HDLR_setStopFlag(0); // clear stop
			status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
			// clear flag to idle
			if(status != COMM_FAILED)
			{
				NVM_clearRunStopFlag();
				prev_run_stop = run_stop;
				//HDLR_setStartRunTime(); // set Run start time
			}
		}
#endif
		kprintf(PORT_DEBUG, "RUN Flag, send to DSP status=%d dir_chk=%d\r\n", status, dir_check);

		break;

	case RUN_STOP_FLAG_STOP:
		// send stop to DSP
		HDLR_setStopFlag(1); // start stop
#ifndef SUPPORT_UNIT_TEST
		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
		{
			NVM_clearRunStopFlag();
			prev_run_stop = run_stop;
		}
#endif
		kprintf(PORT_DEBUG, "STOP Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_IDLE:
	default:
		break;
	}

//	if(status == 0)
//	{
//		ERR_setErrorState(TRIP_REASON_MCU_INPUT);
//		kprintf(PORT_DEBUG, "ERROR!!  RUN/STOP Flag, status=%d\r\n", status);
//	}

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
		HDLR_setStopFlag(0); // clear stop
		// send run to DSP
#ifndef SUPPORT_UNIT_TEST
		status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
#endif
		{
			HDLR_clearRunStopFlagModbus();
			//HDLR_setStartRunTime(); // set Run start time
		}
		kprintf(PORT_DEBUG, "RUN Flag, send to DSP status=%d\r\n", status);
		break;

	case RUN_STOP_FLAG_STOP:
		HDLR_setStopFlag(1); // start stop
		// send stop to DSP
#ifndef SUPPORT_UNIT_TEST
		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
		// clear flag to idle
		if(status != COMM_FAILED)
#endif
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
	int16_t errflag=0;

	while(index <= baudrate_type) // only writable parameter
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
#ifdef SUPPORT_TASK_WATCHDOG
		main_kickWatchdogNFC();
#endif
	}

	// restore CRC as well
	NVM_setCRC();

	// clear NFC tag flag
	NVM_clearNfcStatus();

	if(errflag) return 0;

	return 1;
}

// update EEPROM from table
int8_t HDLR_updateParamNVM(void)
{
	int32_t value, nvm_value;
	int8_t empty, status;
	uint8_t nvm_status=0;
	uint16_t addr;
	int16_t errflag=0;
	int8_t crc_flag = 0;

	do {

		status = NVMQ_dequeueNfcQ(&addr, &value);
		if(status == 0) {kprintf(PORT_DEBUG,"ERROR NfcQ dequeue \r\n"); errflag++;}

		nvm_status = NVM_read(addr, &nvm_value);
		if(nvm_status == 0 || nvm_value != value)
		{
			//kprintf(PORT_DEBUG,"ERROR NVM read error addr=0x%x\r\n", addr); errflag++;
			if(addr <= table_getAddr(baudrate_type)) crc_flag=1;
			nvm_status = NVM_write(addr, value);
			if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM write error addr=0x%x\r\n", addr); errflag++;}
		}
		osDelay(5);
#ifdef SUPPORT_TASK_WATCHDOG
		main_kickWatchdogNFC();
#endif

		empty = NVMQ_isEmptyNfcQ();
	} while(empty == 0); // not empty

	// update CRC
	if(crc_flag)
		NVM_setCRC();

	if(errflag) return 0;

	return 1;
}

void HDLR_retryUpdate(int *fail_list, int count)
{
	int i, read_fail[baudrate_type+1], fail_cnt=0, retry_index;
	int32_t nvm_value;
	uint8_t nvm_status=0;
	int8_t status=1;
	int valid=0;

	fail_cnt=0;
	for(i=0; i<=baudrate_type; i++) read_fail[i] = 0;

	for(i=0; i<count; i++)
	{
		osDelay(5);
		retry_index = fail_list[i];
		nvm_status = NVM_readParam((PARAM_IDX_t)retry_index, &nvm_value);
		if(nvm_status == 0)
		{
			kprintf(PORT_DEBUG,"Retry ERROR NVM read error index=%d \r\n", i);
			read_fail[fail_cnt++] = retry_index;
		}
		else
		{
			valid = table_checkValidityNFC(retry_index, nvm_value); // valid range check
			if(valid == 0) // wrong value than restore NVM
			{
				status = NVM_writeParam(retry_index, table_getValue(retry_index));
				if(status == 0) {kprintf(PORT_DEBUG,"retry ERROR NVM restore error index=%d\n", retry_index);}
			}
			else // correct value to update table
			{
				if(nvm_value != table_getValue(retry_index))
				{
					status = NVMQ_enqueueTableQ(retry_index, nvm_value);
					if(status == 0) {kprintf(PORT_DEBUG,"retry ERROR table enqueue error index=%d \r\n", retry_index); }
					kprintf(PORT_DEBUG,"HDLR_retryUpdate index=%d, value=%d, status=%d \r\n", retry_index, nvm_value, status);
				}
			}
		}
	}

	kprintf(PORT_DEBUG,"HDLR_retryUpdate() count=%d, fail_cnt=%d \r\n", count, fail_cnt);
}

// EEPROM is updated by NFC -> inform to table to update
int8_t HDLR_updatebyNfc(void)
{
	PARAM_IDX_t index=value_type;
	int i, read_fail[55], fail_cnt=0;
	int32_t nvm_value;
	uint8_t nvm_status;
	int8_t status;
	int16_t errflag=0;
	int valid=0;

	fail_cnt=0;
	for(i=0; i<=baudrate_type; i++) read_fail[i] = 0;

	while(index <= baudrate_type) // only writable parameter
	{
		osDelay(5);
		nvm_status = NVM_readParam(index, &nvm_value);
		if(nvm_status == 0)
		{
			kprintf(PORT_DEBUG,"ERROR NVM read error index=%d \r\n", index); errflag++;
			read_fail[fail_cnt++] = (int)index;
		}
		else
		{
			valid = table_checkValidityNFC(index, nvm_value); // valid range check
			if(valid == 0) // wrong value than restore NVM
			{
				status = NVM_writeParam(index, table_getValue(index));
				if(status == 0) {kprintf(PORT_DEBUG,"ERROR NVM restore error index=%d\n", index); errflag++;}
			}
			else // correct value to update table
			{
				if(nvm_value != table_getValue(index))
				{
					status = NVMQ_enqueueTableQ(index, nvm_value);
					if(status == 0) {kprintf(PORT_DEBUG,"ERROR table enqueue error index=%d \r\n", index); errflag++;}

					kprintf(PORT_DEBUG,"HDLR_updatebyNfc updated index=%d, value=%d, valid=%d \r\n", index, nvm_value, valid);
				}
			}
		}
		index++;
	}

	if(fail_cnt > 0) // read error then retry
		HDLR_retryUpdate(read_fail, fail_cnt);

	kprintf(PORT_DEBUG, "HDLR_updatebyNfc\r\n");

//	CRC is updated by NFC App or NFC task
//	NVM_setCRC();

	// clear NFC tag flag
	NVM_clearNfcStatus();

	if(errflag) return 0;
	else	return 1;
}

#ifdef SUPPORT_INIT_PARAM

uint8_t HDLR_isNeedInitialize(void)
{

	if(param_init_requested_f == 0 && NVM_isInitNvmNfc() == 0) return NVM_INIT_PARAM_NONE;

	if(param_init_requested_f) return param_init_requested_f;

	if(NVM_isInitNvmNfc())
	{
		if(table_isMotorStop())
		{
			return NVM_getInitSysParam();
		}
		else
		{
			NVM_clearInitParamCmd();
		}
	}

	return NVM_INIT_PARAM_NONE;
}

int8_t HDLR_initNVM(NVM_INIT_t init_type)
{
	PARAM_IDX_t index=value_type;
	int32_t nvm_value, init_value;
	uint8_t nvm_status;
	int8_t status;
	int16_t errflag=0;
	uint16_t buf[3]={0,0,0};

	if(init_type == NVM_INIT_PARAM_ALL || init_type == NVM_INIT_PARAM_CONFIG)
	{
		while(index <= baudrate_type) // only writable parameter
		{
			osDelay(5);
			nvm_status = NVM_readParam(index, &nvm_value);
			if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM read error index=%d \r\n", index); errflag++;}

			init_value = table_getInitValue(index);
			if(nvm_value != init_value)
			{
				nvm_status = NVM_writeParam(index, init_value);
				if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR NVM write error index=0x%x\r\n", index); errflag++;}

				status = NVMQ_enqueueTableQ(index, init_value);
				if(status == 0) {kprintf(PORT_DEBUG,"ERROR table enqueue error index=%d \r\n", index); errflag++;}
				//kprintf(PORT_DEBUG,"HDLR_initNVM index=%d, value=%d, status=%d \r\n", index, nvm_value, status);

				// send default value to DSP
				if(table_getDspAddr(index) == none_dsp) continue;

				COMM_convertValue((uint16_t)index, buf);

				status = COMM_sendMessage(SPICMD_PARAM_W, buf);
				kprintf(PORT_DEBUG, "HDLR_initNVM() DSP COMM : status=%d, idx=%d, value=%d, param=%d\r\n", \
						status, index, (int)nvm_value, init_value);
				if(status == 0) // retry
				{
					status = COMM_sendMessage(SPICMD_PARAM_W, buf);
					if(status == 0) errflag++;
				}
			}
			index++;

#ifdef SUPPORT_TASK_WATCHDOG
			main_kickWatchdogNFC();
#endif
		}
		kprintf(PORT_DEBUG, "1: err=%d\r\n", errflag);

		// initialize gear ratio
		init_value = table_getInitValue(gear_ratio_type);
		status = NVM_writeParam((PARAM_IDX_t)gear_ratio_type, init_value);
		if(status == 0) errflag++;
	}

	init_value = table_getInitValue(motor_type_type);
	status = NVM_writeParam((PARAM_IDX_t)motor_type_type, init_value);
	if(status == 0) errflag++;

	if(init_type == NVM_INIT_PARAM_ALL || init_type == NVM_INIT_PARAM_ERROR)
	{
		// initialize error, status
		for(index=err_code_1_type; index<PARAM_TABLE_SIZE; index++)
		{
			init_value = table_getInitValue(index);
			status = NVM_writeParam((PARAM_IDX_t)index, init_value);
			if(status == 0) errflag++;

			table_initStatusError(index); // clear table value
			//kprintf("idx=%d: value=%d, nvm=%d\r\n", i, param_table[i].initValue, table_nvm[i]);

#ifdef SUPPORT_TASK_WATCHDOG
			main_kickWatchdogNFC();
#endif
		}
		kprintf(PORT_DEBUG, "2: err=%d\r\n", errflag);
	}

	if(init_type == NVM_INIT_PARAM_ALL || init_type == NVM_INIT_PARAM_TIME)
	{
		status = NVM_initTime();
		if(status == 0) errflag++;

#ifdef SUPPORT_TASK_WATCHDOG
		main_kickWatchdogNFC();
#endif
		kprintf(PORT_DEBUG, "3: err=%d\r\n", errflag);
	}

	// init system param
	status = NVM_initSystemParam();
	if(status == 0) errflag++;

#ifdef SUPPORT_TASK_WATCHDOG
	main_kickWatchdogNFC();
#endif

	//NVM_setInit(); // not use init flag

	NVM_setCRC();

	status = table_getMotorType(); //initialize motor_type

	status = table_updateFwVersion(); // initialize FW version

	kprintf(PORT_DEBUG, "HDLR_initNVM() Done \r\n");

	if(errflag) return 0;
	else	return 1;
}
#endif

#if 0
void HDLR_saveMotorRunTime(void)
{
	if(state_run_stop == CMD_RUN)
	{
		run_minutes = (device_min_cnt - motor_run_start_time)%60;
		NVM_setMotorRunTimeMinute(run_minutes);
	}
}
#endif

int8_t HDLR_updateTime(uint32_t cur_time)
{
	int8_t status;
	int16_t errflag=0;
	static int16_t prev_state_run_stop=CMD_STOP;

	// process On time
	if(cur_time%60 == 0) // && cur_time != dev_start_time) // 1 hour
	{
		device_on_hour++;
		status = NVM_setDeviceOnTime(device_on_hour);
		if(status == 0) {kprintf(PORT_DEBUG, "ERROR update On time \r\n"); errflag++;}
		NVMQ_enqueueTableQ(operating_hour_type, (int32_t)device_on_hour);

#if 0
		status = NVM_setMotorDevCounter(cur_time);
		if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Dev time \r\n"); errflag++;}
#endif

		kprintf(PORT_DEBUG, "1 hour time %d\r\n", cur_time);
	}

	// process Run time
	if(state_run_stop == CMD_RUN)
	{
		if(prev_state_run_stop == CMD_STOP)
		{
			HDLR_setStartRunTime();
		}

		if(cur_time - motor_run_start_time >= 60) // update 1 hour
		{
			motor_run_hour++;
			status = NVM_setMotorRunTime(motor_run_hour);
			if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Run time \r\n"); errflag++;}
			NVMQ_enqueueTableQ(elapsed_hour_type, (int32_t)motor_run_hour);

			motor_run_start_time = cur_time;
			kprintf(PORT_DEBUG, "motor run 1 more hour %d\r\n", cur_time);

#if 0
			status = NVM_setMotorDevCounter(cur_time);
			if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Dev time at STOP\r\n"); errflag++;}
#endif
		}

		prev_state_run_stop = state_run_stop;
	}
	else if(state_run_stop == CMD_STOP)
	{
		if(prev_state_run_stop == CMD_RUN) // run -> stop
		{
#if 0
			status = NVM_setMotorDevCounter(cur_time);
			if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Dev time \r\n"); errflag++;}
#endif
			run_minutes = (cur_time - motor_run_start_time)%60;
#if 0
			status = NVM_setMotorRunTimeMinute(run_minutes);
			if(status == 0) {kprintf(PORT_DEBUG, "ERROR update Run time minute\r\n"); errflag++;}
#endif

			kprintf(PORT_DEBUG, "stop run time %d, minutes=%d\r\n", cur_time, run_minutes);
		}

		prev_state_run_stop = state_run_stop;
	}


	if(errflag) return 0;
	else	return 1;
}

int8_t HDLR_updateSysParam(int index)
{
	int8_t status=0;
	uint16_t addr;
	int32_t value;

	addr = NVM_getSystemParamAddr(index);
	value = NVM_getSystemParamValue(index);
	status = NVM_write(addr, value);
	if(status == 1) // write OK
		NVM_clearSysParamUpdateFlag(index);
	else
	{
		NVM_increaseSysParamRetryCnt(index);
		if(NVM_getSysParamRetryCnt(index) > NVM_SYS_PARAM_UPDATE_RETRY_MAX)
		{
			//ERR_setErrorState(TRIP_REASON_MCU_SETVALUE);
#ifdef SUPPORT_FORCE_RESET
			main_SwReset(1); // force reset
#endif
		}
	}

	kprintf(PORT_DEBUG, "HDLR_updateSysParam() index=%d status=%d\r\n", index, status);

	return status;
}

#ifdef SUPPORT_PARAMETER_BACKUP
int8_t mb_backup_mode_f=0;

void HDLR_setBackupFlagModbus(int8_t flag)
{
	mb_backup_mode_f = flag;
}

void HDLR_clearBackupFlagModbus(void)
{
	if(mb_backup_mode_f != 0)
	{
		mb_backup_mode_f = 0;
		return;
	}
	else
	{
		NVM_clearBackupCmd();
	}
}

int8_t HDLR_getBackupFlag(void)
{
	int8_t bk_cmd=0;

	if(mb_backup_mode_f != 0)
		return mb_backup_mode_f;
	else
		bk_cmd = NVM_getBackupCmdNfc();

	return bk_cmd;
}

int8_t HDLR_isBackupEnabled(void)
{
	return (mb_backup_mode_f != 0 || NVM_isBackupCmd() != 0);
}

int8_t HDLR_clearBackupFlag(void)
{
	return NVM_setBackupAvailableFlag((int32_t)0);
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

	nvm_status = NVM_setBackupAvailableFlag(NVM_BACKUP_AVAILABLE_F);
	if(nvm_status == 0) {kprintf(PORT_DEBUG,"set NVM backup flag error\r\n"); errflag++;}

	kprintf(PORT_DEBUG,"NVM backup finished err=%d\r\n", errflag);

	if(errflag) return 0;
	else	return 1;
}

// read parameters in backup area, write to parameter table
int8_t HDLR_restoreParameter(void)
{
	PARAM_IDX_t idx;
	int8_t nvm_status;
	int32_t s_addr=0, value=0;
	int16_t errflag=0;

	for(idx=value_type; idx <= baudrate_type; idx++)
	{
		s_addr = NVM_BACKUP_START_ADDR + idx*4;
		nvm_status = NVM_read(s_addr, &value);
		if(nvm_status == 0)
		{
			kprintf(PORT_DEBUG,"ERROR backup NVM read error idx=0x%x\r\n", idx);
			errflag++;
			continue;
		}

		if(value == table_getValue(idx)) continue; // skip same value

		nvm_status = NVM_writeParam(idx, value);
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR backup NVM write error idx=0x%x\r\n", idx); errflag++;}

		nvm_status = NVMQ_enqueueTableQ(idx, value); // let table updated
		if(nvm_status == 0) {kprintf(PORT_DEBUG,"ERROR backup NVM enqueue error idx=0x%x\r\n", idx); errflag++;}

		osDelay(3);
	}

	if(errflag) return 0;
	else	return 1;
}
#endif
