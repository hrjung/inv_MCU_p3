/*
 * drv_nvm.c
 *
 *  Created on: 2019. 6. 3.
 *   mock implementation of NFC EEPROM driver
 *      Author: hrjung
 */

#include "includes.h"

#include "main.h"
#include "cmsis_os.h"

#include "proc_uart.h"
#include "table.h"

#include "drv_ST25DV.h"
#include "drv_nvm.h"


//#define INTERNAL_DEV_COUNTER		0x5A0
//#define INTERNAL_RUN_REMAIN_TIME	0x5A4

//#define TABLE_SIZE_MAX	250
#if 0
#if 0
static int32_t sysparam_addr[] =
{
		0x10,		//SYSTEM_PARAM_NFC_TAGGED
		0x14,		//SYSTEM_PARAM_CRC_VALUE
		0x18,		//SYSTEM_PARAM_IS_INITIATED
		0x1C,		//SYSTEM_PARAM_HAS_SYSTEM_ERROR
		0x20,		//SYSTEM_PARAM_ENABLE_NFC_WRITER
		0x24,		//SYSTEM_PARAM_NFC_TRYED
		0x28,		//SYSTEM_PARAM_ON_MONITORING
		0x2C,		//SYSTEM_PARAM_IDLE0_RUN1_STOP2
};
#else
static int32_t sysparam_addr[] =
{
		400,		//SYSTEM_PARAM_NFC_TAGGED
		404,		//SYSTEM_PARAM_CRC_VALUE
		408,		//SYSTEM_PARAM_IS_INITIATED
		412,		//SYSTEM_PARAM_HAS_SYSTEM_ERROR
		416,		//SYSTEM_PARAM_ENABLE_NFC_WRITER
		420,		//SYSTEM_PARAM_NFC_TRYED
		424,		//SYSTEM_PARAM_ON_MONITORING
		428,		//SYSTEM_PARAM_IDLE0_RUN1_STOP2
		432,		//SYSTEM_PARAM_BACKUP_CMD
};
#endif
#endif

#ifndef SUPPORT_DRIVER_HW
static int32_t nvm_table[1024];
#endif
int32_t table_nvm[PARAM_TABLE_SIZE];


STATIC Param_sys_t sys_table[] =
{		// SYSTEM_PARAM_t					addr	need_update		done
		{ SYSTEM_PARAM_NFC_TAGGED, 			400, 	NO_CHANGE, 		0},
		{ SYSTEM_PARAM_CRC_VALUE, 			404,	NO_CHANGE,		0},
		{ SYSTEM_PARAM_IS_INITIATED, 		408, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_HAS_SYSTEM_ERROR, 	412, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_ENABLE_NFC_WRITER,	416, 	NO_CHANGE,		0},

		{ SYSTEM_PARAM_NFC_TRYED, 			420, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_ON_MONITORING, 		424, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_RUN1_STOP2, 			428, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_INIT_NVM, 			432, 	NO_CHANGE,		0},
		{ SYSTEM_PARAM_BACKUP_CMD, 			436, 	NO_CHANGE,		0},

};

int32_t sys_data[SYSTEM_PARAM_SIZE] = {0,0,0,0,0, 0,0,0,0};

extern uint32_t motor_run_cnt;
extern uint32_t motor_run_hour;
extern uint32_t device_on_hour;
extern uint32_t device_min_cnt;
extern uint32_t dev_start_time;
extern uint32_t run_minutes;

extern uint16_t table_getAddr(PARAM_IDX_t index);
extern uint32_t table_calcCRC(void);

uint8_t NVM_read(uint16_t addr, int32_t *value)
{
	uint8_t status=NVM_OK;

#ifdef SUPPORT_DRIVER_HW
	status = I2C_readData((uint8_t *)value, addr, sizeof(int32_t));
#else
	*value = nvm_table[addr];
#endif

	//kprintf(PORT_DEBUG, "NVM_read addr=%d, value=%d, status=%d \r\n", (int)addr, (int)*value, status);
	return status;
}

uint8_t NVM_write(int32_t addr, int32_t value)
{
	uint8_t status=NVM_OK;

#ifdef SUPPORT_DRIVER_HW
	status = I2C_writeData((uint8_t *)&value, addr, sizeof(int32_t));
#else
	nvm_table[addr] = value;
#endif
	//kprintf(PORT_DEBUG, "NVM_write addr=%d, value=%d, status=%d \r\n", (int)addr, (int)value, status);
	return status;
}

#ifndef SUPPORT_DRIVER_HW
void NVM_clear(void)
{
	int i;

	for(i=0; i<PARAM_TABLE_SIZE; i++) nvm_table[i] = 0;

}
#endif

uint8_t NVM_readParam(PARAM_IDX_t index, int32_t *value)
{
	uint8_t status=NVM_NOK;
	uint16_t nvm_addr;

	nvm_addr = table_getAddr(index);
	status = NVM_read(nvm_addr, value);
	if(status == NVM_OK)
	{
		table_nvm[index] = *value;
		//kprintf(PORT_DEBUG, "NVM_readParam idx=%d, value=%d\r\n", index, (int)table_nvm[index]);
	}
	else
		kprintf(PORT_DEBUG, "NVM_readParam ERR idx=%d\r\n", index);

	return status;
}

uint8_t NVM_writeParam(PARAM_IDX_t index, int32_t value)
{
	uint8_t status;
	uint16_t nvm_addr;
	//int32_t r_value;

	nvm_addr = table_getAddr(index);
	status = NVM_write(nvm_addr, value);
	if(status)
		table_nvm[index] = value;
	else
		kprintf(PORT_DEBUG, "NVM_writeParam ERR idx=%d\r\n", index);

	//status = NVM_read(nvm_addr, &r_value);
	//if(r_value != value || status == 0)
	//kprintf(PORT_DEBUG, "NVM_writeParam idx=%d, addr=%d, value=%d, r_value=%d, status=%d\r\n", index, nvm_addr, value, r_value, status);
	return status;
}

int8_t NVM_readTime(void)
{
	int errflag=0;
	int8_t status=NVM_OK;

	status = NVM_readParam(motor_on_cnt_type, (int32_t *)&motor_run_cnt);
	if(status==NVM_NOK) errflag++;

	status = NVM_readParam(elapsed_hour_type, (int32_t *)&motor_run_hour);
	if(status==NVM_NOK) errflag++;

	status = NVM_readParam(operating_hour_type, (int32_t *)&device_on_hour);
	if(status==NVM_NOK) errflag++;


#if 0
	// restore device counter
	status = NVM_read((uint16_t)INTERNAL_DEV_COUNTER, (int32_t *)&device_min_cnt);
	if(status==NVM_NOK) errflag++;

	dev_start_time = device_min_cnt;

	// run minute time
	status = NVM_read((uint16_t)INTERNAL_RUN_REMAIN_TIME, (int32_t *)&run_minutes);
	if(status==NVM_NOK) errflag++;
#endif

	if(errflag) status=NVM_NOK;

	return status;
}

int8_t NVM_setDeviceOnTime(uint32_t on_time)
{
	int8_t status;

	status = NVM_writeParam(operating_hour_type, (int32_t)on_time);
	//NVMQ_enqueueTableQ(elapsed_hour_type, (int32_t)on_time);

	return status;
}

int8_t NVM_setMotorRunTime(uint32_t run_time)
{
	int8_t status;

	status = NVM_writeParam(elapsed_hour_type, (int32_t)run_time);
	//NVMQ_enqueueTableQ(operating_hour_type, (int32_t)run_time);

	return status;
}

int8_t NVM_setMotorRunCount(uint32_t run_count)
{
	int8_t status;

	status = NVM_writeParam(motor_on_cnt_type, (int32_t)run_count);
	//NVMQ_enqueueTableQ(motor_on_cnt_type, (int32_t)run_count);

	return status;
}

int8_t NVM_initTime(void)
{
	int errflag=0;
	int8_t status=NVM_OK;

	motor_run_cnt=0;
	status = NVM_setMotorRunCount(motor_run_cnt);
	if(status==NVM_NOK) errflag++;

	device_on_hour=0;
	status = NVM_setDeviceOnTime(device_on_hour);
	if(status==NVM_NOK) errflag++;

	motor_run_hour=0;
	status = NVM_setMotorRunTime(motor_run_hour);
	if(status==NVM_NOK) errflag++;

#if 0
	device_min_cnt=0; dev_start_time=0;
	status = NVM_setMotorDevCounter(device_min_cnt);
	if(status==NVM_NOK) errflag++;

	run_minutes=0;
	status = NVM_setMotorRunTimeMinute(run_minutes);
	if(status==NVM_NOK) errflag++;
#endif

	if(errflag) status=NVM_NOK;

	return status;
}

uint16_t NVM_getSystemParamAddr(uint16_t index)
{
	return (uint16_t)sys_table[index].addr;
}

int32_t NVM_getSystemParamValue(uint16_t index)
{
	return sys_data[index];
}

void NVM_clearSysParamUpdateFlag(uint16_t index)
{
	sys_table[index].need_update = NO_CHANGE;
}

int NVM_isSysParamNeedUpdate(uint16_t index)
{
	return (sys_table[index].need_update != NO_CHANGE);
}

int8_t NVM_initSystemParam(void)
{
	int i, errflag=0;
	int8_t status=NVM_OK;

	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
	{
		status = NVM_write((uint16_t)sys_table[i].addr, (int32_t)sys_data[i]); // clear all flag
		if(status==NVM_NOK) errflag++;
	}

	if(errflag) status=NVM_NOK;

	return status;
}

int8_t NVM_initError(void)
{
	int i, errflag=0;
	int8_t status=NVM_OK;
	uint16_t addr;
	int32_t value=0;

	for(i=err_code_1_type; i<=err_freq_5_type; i++)
	{
		addr = table_getAddr((PARAM_IDX_t)i);
		status = NVM_write(addr, value); // clear all flag
		if(status==NVM_NOK) errflag++;
	}

	if(errflag) status=NVM_NOK;

	return status;
}

void NVM_setInit(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_IS_INITIATED] != 1)
	{
		sys_data[SYSTEM_PARAM_IS_INITIATED] = 1;
		sys_table[SYSTEM_PARAM_IS_INITIATED].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_IS_INITIATED].addr, sys_data[SYSTEM_PARAM_IS_INITIATED]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_IS_INITIATED);
	}
}

void NVM_clearInit(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_IS_INITIATED] != 0)
	{
		sys_data[SYSTEM_PARAM_IS_INITIATED] = 0;
		sys_table[SYSTEM_PARAM_IS_INITIATED].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_IS_INITIATED].addr, sys_data[SYSTEM_PARAM_IS_INITIATED]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_IS_INITIATED);
	}
}

int8_t NVM_isInit(void)
{
	int32_t isInit=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_IS_INITIATED].addr, &isInit);

	printf("NVM_isInit init=%d, status=%d\r\n", (int)isInit, status);
	if(status!=NVM_OK) return -1;

	sys_data[SYSTEM_PARAM_IS_INITIATED] = isInit;

	return (int8_t)isInit;
}

int8_t NVM_isNfcMonitoring(void)
{
	int32_t isMonitoring=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_ON_MONITORING].addr, &isMonitoring);
	if(status!=NVM_OK) return 0; // default not monitoring

	sys_data[SYSTEM_PARAM_ON_MONITORING] = isMonitoring;

	return (int8_t)isMonitoring;
}

void NVM_clearNfcMonitoring(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_ON_MONITORING] != 0)
	{
		sys_data[SYSTEM_PARAM_ON_MONITORING] = 0;
		sys_table[SYSTEM_PARAM_ON_MONITORING].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_ON_MONITORING].addr, sys_data[SYSTEM_PARAM_ON_MONITORING]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_ON_MONITORING);
	}
}

int8_t NVM_getNfcStatus(int32_t *tag_end)
{
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_NFC_TAGGED].addr, tag_end);
	//if(status!=NVM_OK) return NVM_NOK;

	//status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_NFC_TRYED].addr, tag_tryed);
	//if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

void NVM_clearNfcStatus(void)
{
	uint8_t status;

	sys_data[SYSTEM_PARAM_NFC_TAGGED] = 0;
	sys_table[SYSTEM_PARAM_NFC_TAGGED].need_update = WRITE_TO_NVM;
	status = NVM_write(sys_table[SYSTEM_PARAM_NFC_TAGGED].addr, sys_data[SYSTEM_PARAM_NFC_TAGGED]);
	if(status == 1)
		NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_NFC_TAGGED);

	sys_data[SYSTEM_PARAM_NFC_TRYED] = 0;
	sys_table[SYSTEM_PARAM_NFC_TRYED].need_update = WRITE_TO_NVM;
	status = NVM_write(sys_table[SYSTEM_PARAM_NFC_TRYED].addr, sys_data[SYSTEM_PARAM_NFC_TRYED]);
	if(status == 1)
		NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_NFC_TRYED);

}

int8_t NVM_getRunStopFlag(int32_t *run_stop)
{
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_RUN1_STOP2].addr, run_stop);
	if(status!=NVM_OK) return NVM_NOK;

	sys_data[SYSTEM_PARAM_RUN1_STOP2] = *run_stop;

	return NVM_OK;
}

void NVM_clearRunStopFlag(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_RUN1_STOP2] != 0)
	{
		sys_data[SYSTEM_PARAM_RUN1_STOP2] = 0;
		sys_table[SYSTEM_PARAM_RUN1_STOP2].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_RUN1_STOP2].addr, sys_data[SYSTEM_PARAM_RUN1_STOP2]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_RUN1_STOP2);
	}
}


int8_t NVM_verifyCRC(uint32_t crc32_calc)
{
	uint8_t status;
	uint32_t crc32_rd;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_CRC_VALUE].addr, (int32_t *)&crc32_rd);

	//kprintf(PORT_DEBUG, "calc=0x%x, read_crc=0x%x, status=%d\r\n", crc32_calc, crc32_rd, status);
	if(status != NVM_OK) return NVM_NOK;

	sys_data[SYSTEM_PARAM_CRC_VALUE] = crc32_rd;

	if(crc32_calc != crc32_rd) return NVM_NOK;

	return NVM_OK;
}

void NVM_setCRC(void)
{
	uint8_t status;
	uint32_t crc32_calc;

	crc32_calc = table_calcCRC();

	if(NVM_verifyCRC(crc32_calc) == NVM_NOK) // updated
	{
		sys_data[SYSTEM_PARAM_CRC_VALUE] = crc32_calc;
		sys_table[SYSTEM_PARAM_CRC_VALUE].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_CRC_VALUE].addr, sys_data[SYSTEM_PARAM_CRC_VALUE]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_CRC_VALUE);
	}
}

int NVM_getSysParamUpdateIndex(void)
{
	int i;

	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
	{
		if(sys_table[i].need_update == WRITE_TO_NVM) return i;
	}

	return SYSTEM_PARAM_SIZE;
}

#if 0
int8_t NVM_getMotorRunTimeMinute(int32_t *remain_time)
{
	uint8_t status;

	status = NVM_read((uint16_t)INTERNAL_RUN_REMAIN_TIME, remain_time);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_setMotorRunTimeMinute(uint32_t r_time)
{
	uint8_t status;

	status = NVM_write((uint16_t)INTERNAL_RUN_REMAIN_TIME, (int32_t)r_time);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_setMotorDevCounter(uint32_t r_time)
{
	uint8_t status;

	status = NVM_write((uint16_t)INTERNAL_DEV_COUNTER, (int32_t)r_time);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}
#endif


uint8_t NVM_isInitNvmNfc(void)
{
	return (uint8_t)(sys_data[SYSTEM_PARAM_INIT_NVM] == 1);
}

uint8_t NVM_isBackupCmd(void)
{
	return (uint8_t)(sys_data[SYSTEM_PARAM_BACKUP_CMD] != 0);
}

int8_t NVM_getBackupCmdNfc(void)
{
	return (int8_t)sys_data[SYSTEM_PARAM_BACKUP_CMD];
}

void NVM_getCommandParam(void)
{
	int32_t init_nvm=0, bk_cmd=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_INIT_NVM].addr, &init_nvm);
	if(status == NVM_OK)
		sys_data[SYSTEM_PARAM_INIT_NVM] = init_nvm;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_BACKUP_CMD].addr, &bk_cmd);
	if(status == NVM_OK)
		sys_data[SYSTEM_PARAM_BACKUP_CMD] = bk_cmd;

}


void NVM_clearInitParamCmd(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_INIT_NVM] != 0)
	{
		sys_data[SYSTEM_PARAM_INIT_NVM] = 0;
		sys_table[SYSTEM_PARAM_INIT_NVM].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_INIT_NVM].addr, sys_data[SYSTEM_PARAM_INIT_NVM]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_INIT_NVM);
	}
}

void NVM_clearBackupCmd(void)
{
	uint8_t status;

	if(sys_data[SYSTEM_PARAM_BACKUP_CMD] != 0)
	{
		sys_data[SYSTEM_PARAM_BACKUP_CMD] = 0;
		sys_table[SYSTEM_PARAM_BACKUP_CMD].need_update = WRITE_TO_NVM;
		status = NVM_write(sys_table[SYSTEM_PARAM_BACKUP_CMD].addr, sys_data[SYSTEM_PARAM_BACKUP_CMD]);
		if(status == 1)
			NVM_clearSysParamUpdateFlag(SYSTEM_PARAM_BACKUP_CMD);
	}
}

