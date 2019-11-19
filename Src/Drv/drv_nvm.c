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


//#define TABLE_SIZE_MAX	250

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
		412,		//SYSTEM_PARAM_HAS_SYSTEM_ERR
		416,		//SYSTEM_PARAM_ENABLE_NFC_WR
		420,		//SYSTEM_PARAM_NFC_TRYED
		424,		//SYSTEM_PARAM_ON_MONITORING
		428,		//SYSTEM_PARAM_IDLE0_RUN1_STOP2
		432,		//SYSTEM_PARAM_BACKUP_CMD
};
#endif

#ifndef SUPPORT_DRIVER_HW
static int32_t nvm_table[1024];
#endif
int32_t table_nvm[PARAM_TABLE_SIZE];


STATIC Param_sys_t sys_table[] =
{		// SYSTEM_PARAM_t				addr	value, 	need_update
		{ SYSTEM_PARAM_NFC_TAGGED, 		400, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_CRC_VALUE, 		404,	0,		NO_CHANGE},
		{ SYSTEM_PARAM_IS_INITIATED, 	408, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_HAS_SYSTEM_ERR, 	412, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_ENABLE_NFC_WR, 	416, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_NFC_TRYED, 		420, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_ON_MONITORING, 	424, 	0,		NO_CHANGE},
		{ SYSTEM_PARAM_RUN1_STOP2, 		428, 	0,		NO_CHANGE},
#ifdef SUPPORT_PARAMETER_BACKUP
		{ SYSTEM_PARAM_BACKUP_CMD, 		432, 	0,		NO_CHANGE},
#endif
};

int32_t isMonitoring=0;

extern uint32_t motor_run_cnt;
extern uint32_t motor_run_hour;
extern uint32_t device_on_hour;

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
	uint16_t addr;

	addr = table_getAddr(motor_on_cnt_type);
	status = NVM_read(addr, (int32_t *)&motor_run_cnt);
	if(status==NVM_NOK) errflag++;

	addr = table_getAddr(elapsed_hour_type);
	status = NVM_read(addr, (int32_t *)&device_on_hour);
	if(status==NVM_NOK) errflag++;

	addr = table_getAddr(operating_hour_type);
	status = NVM_read(addr, (int32_t *)&motor_run_hour);
	if(status==NVM_NOK) errflag++;

	if(errflag) status=NVM_NOK;

	return status;
}

int8_t NVM_setDeviceOnTime(uint32_t on_time)
{
	int8_t status;
	uint16_t addr;

	addr = table_getAddr(elapsed_hour_type);
	status = NVM_write(addr, (int32_t)on_time);

	return status;
}

int8_t NVM_setMotorRunTime(uint32_t run_time)
{
	int8_t status;
	uint16_t addr;

	addr = table_getAddr(operating_hour_type);
	status = NVM_write(addr, (int32_t)run_time);

	return status;
}

int8_t NVM_setMotorRunCount(uint32_t run_count)
{
	int8_t status;
	uint16_t addr;

	addr = table_getAddr(motor_on_cnt_type);
	status = NVM_write(addr, (int32_t)run_count);

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

	if(errflag) status=NVM_NOK;

	return status;
}

uint16_t NVM_getSystemParamAddr(uint16_t index)
{
	return (uint16_t)sys_table[index].addr;
}

int8_t NVM_initSystemParam(void)
{
	int i, errflag=0;
	int8_t status=NVM_OK;
	int32_t value=0;

	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
	{
		status = NVM_write((uint16_t)sys_table[i].addr, (int32_t)sys_table[i].value); // clear all flag
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

	for(i=err_date_0_type; i<=err_freq_4_type; i++)
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
	int32_t value=1;
	sys_table[SYSTEM_PARAM_IS_INITIATED].value = value;
	sys_table[SYSTEM_PARAM_IS_INITIATED].need_update = WRITE_TO_NVM;
}

void NVM_clearInit(void)
{
	sys_table[SYSTEM_PARAM_IS_INITIATED].value = 0;
	sys_table[SYSTEM_PARAM_IS_INITIATED].need_update = WRITE_TO_NVM;
}

int8_t NVM_isInit(void)
{
	int32_t isInit=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_IS_INITIATED].addr, &isInit);

	printf("NVM_isInit init=%d, status=%d\r\n", (int)isInit, status);
	if(status!=NVM_OK) return -1;

	sys_table[SYSTEM_PARAM_IS_INITIATED].value = isInit;

	return (int8_t)isInit;
}

int8_t NVM_isNfcMonitoring(void)
{
	int32_t isMonitoring=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_ON_MONITORING].addr, &isMonitoring);
	if(status!=NVM_OK) return -1;

	sys_table[SYSTEM_PARAM_ON_MONITORING].value = isMonitoring;

	return (int8_t)isMonitoring;
}

void NVM_clearNfcMonitoring(void)
{
	sys_table[SYSTEM_PARAM_ON_MONITORING].value = 0;
	sys_table[SYSTEM_PARAM_ON_MONITORING].need_update = WRITE_TO_NVM;
}

int8_t NVM_getNfcStatus(int32_t *tag_tryed, int32_t *tag_end)
{
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_NFC_TAGGED].addr, tag_end);
	//if(status!=NVM_OK) return NVM_NOK;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_NFC_TRYED].addr, tag_tryed);
	//if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

void NVM_clearNfcStatus(void)
{
	sys_table[SYSTEM_PARAM_NFC_TAGGED].value = 0;
	sys_table[SYSTEM_PARAM_NFC_TAGGED].need_update = WRITE_TO_NVM;

	sys_table[SYSTEM_PARAM_NFC_TRYED].value = 0;
	sys_table[SYSTEM_PARAM_NFC_TRYED].need_update = WRITE_TO_NVM;
}

int8_t NVM_getRunStopFlag(int32_t *run_stop)
{
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_RUN1_STOP2].addr, run_stop);
	if(status!=NVM_OK) return NVM_NOK;

	sys_table[SYSTEM_PARAM_RUN1_STOP2].value = *run_stop;

	return NVM_OK;
}

void NVM_clearRunStopFlag(void)
{
	sys_table[SYSTEM_PARAM_RUN1_STOP2].value = 0;
	sys_table[SYSTEM_PARAM_RUN1_STOP2].need_update = WRITE_TO_NVM;
}

#ifdef SUPPORT_PARAMETER_BACKUP
int8_t NVM_getBackupCmd(void)
{
	int32_t bk_cmd=0;
	uint8_t status;

	status = NVM_read((uint16_t)sys_table[SYSTEM_PARAM_BACKUP_CMD].addr, &bk_cmd);
	if(status!=NVM_OK) return NVM_NOK;

	sys_table[SYSTEM_PARAM_BACKUP_CMD].value = bk_cmd;

	return NVM_OK;
}

void NVM_clearBackupCmd(void)
{
	sys_table[SYSTEM_PARAM_BACKUP_CMD].value = 0;
	sys_table[SYSTEM_PARAM_BACKUP_CMD].need_update = WRITE_TO_NVM;
}
#endif

int8_t NVM_verifyCRC(uint32_t crc32_calc)
{
	if(crc32_calc != sys_table[SYSTEM_PARAM_CRC_VALUE].value) return NVM_NOK;

	return NVM_OK;
}

void NVM_setCRC(void)
{
	uint32_t crc32_calc;

	crc32_calc = table_calcCRC();

	if(NVM_verifyCRC(crc32_calc) == NVM_NOK) // updated
	{
		sys_table[SYSTEM_PARAM_CRC_VALUE].value = crc32_calc;
		sys_table[SYSTEM_PARAM_BACKUP_CMD].need_update = WRITE_TO_NVM;
	}
}

int NVM_isSysParamUpdateRequred(void)
{
	int i;

	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
	{
		if(sys_table[i].need_update == WRITE_TO_NVM) return i;
	}

	return SYSTEM_PARAM_SIZE;
}

