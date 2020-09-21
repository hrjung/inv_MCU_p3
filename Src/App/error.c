/*
 * error.c
 *
 *  Created on: 2019. 6. 11.
 *      Author: hrjung
 */
#include "includes.h"


#include "proc_uart.h"
#include "table.h"
#include "error.h"

#include "drv_gpio.h"


//int32_t err_cnt=0;

STATIC uint8_t err_state=TRIP_REASON_NONE;

//extern int32_t table_getStatusValue(int16_t index);
extern void table_getStatusFromTable(int32_t *status, float *current, float *freq);

uint8_t ERR_isErrorState(void)
{
	return (err_state != TRIP_REASON_NONE);
}

uint8_t ERR_isCommError(void)
{
	return (err_state == TRIP_REASON_MCU_COMM_FAIL);
}

uint8_t ERR_isInitialError(void)
{
	return (err_state == TRIP_REASON_MCU_INIT
			|| err_state == TRIP_REASON_MCU_CRC_FAILURE
			|| err_state == TRIP_REASON_MCU_MOTOR_TYPE
			|| err_state == TRIP_REASON_MCU_WAIT_FAIL);
}

uint16_t ERR_isNvmError(void)
{
	return (err_state == TRIP_REASON_MCU_INIT
			|| err_state == TRIP_REASON_MCU_CRC_FAILURE);
}

uint8_t ERR_getErrorState(void)
{
	return err_state;
}

void ERR_setErrorState(TRIP_REASON_t err_code)
{
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
	if(err_state == err_code) return;
#else
	if(err_state == err_code || err_code == TRIP_REASON_NONE) return; // same error happened, ignore
#endif

	// ignore error after error
	if(err_state != TRIP_REASON_NONE && err_state != TRIP_REASON_MCU_INPUT) return;

	err_state = err_code;

	if(err_state > TRIP_REASON_MAX) // trip from MCU
	{
#ifdef SUPPORT_RESTORE_EMERGENCY_STOP
		if(err_state != TRIP_REASON_MCU_INPUT) // emergency trip
#endif
			UTIL_setMTDpin(1); // notify to DSP
	}

	kprintf(PORT_DEBUG, "set Error=%d\r\n", err_state);
}

