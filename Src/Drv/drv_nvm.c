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
		412,		//SYSTEM_PARAM_HAS_SYSTEM_ERROR
		416,		//SYSTEM_PARAM_ENABLE_NFC_WRITER
		420,		//SYSTEM_PARAM_NFC_TRYED
		424,		//SYSTEM_PARAM_ON_MONITORING
		428,		//SYSTEM_PARAM_IDLE0_RUN1_STOP2
};
#endif

#ifndef SUPPORT_DRIVER_HW
static int32_t nvm_table[PARAM_TABLE_SIZE];
#endif
int32_t table_nvm[PARAM_TABLE_SIZE];

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
	return status;
}

uint8_t NVM_write(int32_t addr, int32_t value)
{
	uint8_t status=NVM_NOK;

#ifdef SUPPORT_DRIVER_HW
	status = I2C_writeData((uint8_t *)&value, addr, sizeof(int32_t));
#else
	nvm_table[addr] = value;
#endif

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

	nvm_addr = table_getAddr(index);
	status = NVM_write(nvm_addr, value);
	if(status)
		table_nvm[index] = value;
	else
		kprintf(PORT_DEBUG, "NVM_writeParam ERR idx=%d\r\n", index);

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
	return (uint16_t)sysparam_addr[index];
}

int8_t NVM_initSystemParam(void)
{
	int i, errflag=0;
	int8_t status=NVM_OK;
	int32_t value=0;

	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
	{
		status = NVM_write((uint16_t)sysparam_addr[i], value); // clear all flag
		if(status==NVM_NOK) errflag++;
	}

	if(errflag) status=NVM_NOK;

	return status;
}

uint8_t NVM_setInit(void)
{
	int32_t value=1;
	return NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_IS_INITIATED], value);
}

uint8_t NVM_clearInit(void)
{
	int32_t value=0;
	return NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_IS_INITIATED], value);
}

int8_t NVM_isInit(void)
{
	int32_t isInit;
	uint8_t status;

	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_IS_INITIATED], &isInit);

	printf("NVM_isInit init=%d, status=%d\r\n", (int)isInit, status);
	if(status!=NVM_OK) return -1;

	return (int8_t)isInit;
}

int8_t NVM_isNfcMonitoring(void)
{
	uint8_t status;
#if 0

#else
	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_ON_MONITORING], &isMonitoring);
	if(status!=NVM_OK) return -1;
#endif

	return (int8_t)isMonitoring;
}

int32_t NVM_isMonitoring(void)
{
	return isMonitoring;
}

int8_t NVM_clearNfcMonitoring(void)
{
	isMonitoring=0;
	return NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_ON_MONITORING], isMonitoring);
}

int8_t NVM_getNfcStatus(int32_t *tag_tryed, int32_t *tag_end)
{
	uint8_t status;

	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_NFC_TAGGED], tag_end);
	//if(status!=NVM_OK) return NVM_NOK;

	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_NFC_TRYED], tag_tryed);
	//if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_clearNfcStatus(void)
{
	int32_t value=0;
	uint8_t status;

	status = NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_NFC_TAGGED], value);
	if(status!=NVM_OK) return NVM_NOK;

	status = NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_NFC_TRYED], value);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_getRunStopFlag(int32_t *run_stop)
{
	uint8_t status;

	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_IDLE0_RUN1_STOP2], run_stop);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_clearRunStopFlag(void)
{
	uint8_t status;
	int32_t value=0;

	status = NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_IDLE0_RUN1_STOP2], value);
	if(status!=NVM_OK) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_verifyCRC(uint32_t crc32_calc)
{
	uint8_t status;
	uint32_t crc32_rd;

	status = NVM_read((uint16_t)sysparam_addr[SYSTEM_PARAM_CRC_VALUE], (int32_t *)&crc32_rd);

	kprintf(PORT_DEBUG, "calc=0x%x, read_crc=0x%x\r\n", crc32_calc, crc32_rd);
	if(status != NVM_OK) return NVM_NOK;

	if(crc32_calc != crc32_rd) return NVM_NOK;

	return NVM_OK;
}

int8_t NVM_setCRC(void)
{
	uint32_t crc32_calc;
	uint8_t status;

	crc32_calc = table_calcCRC();
	status = NVM_write((uint16_t)sysparam_addr[SYSTEM_PARAM_CRC_VALUE], crc32_calc);
	//kprintf(PORT_DEBUG, "NVM_setCRC calc=0x%x, status=%d\r\n", crc32_calc, status);

	if(status != NVM_OK) return NVM_NOK;

	return NVM_OK;
}

