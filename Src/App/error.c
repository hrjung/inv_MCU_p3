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


int32_t err_cnt=0;

STATIC uint8_t err_state=TRIP_REASON_NONE;

uint8_t ERR_isErrorState(void)
{
	return (err_state != TRIP_REASON_NONE);
}

uint8_t ERR_isCommError(void)
{
	return (err_state == TRIP_REASON_MCU_COMM_FAIL);
}

uint8_t ERR_getErrorState(void)
{
	return err_state;
}

void ERR_setErrorState(TRIP_REASON_t err_code)
{
	err_state = err_code;
	if(ERR_isErrorState())
	{
		UTIL_setMTDpin(1); // set error
	}


	//kprintf(PORT_DEBUG, "MCU Error=%d\r\n", err_state);
}

