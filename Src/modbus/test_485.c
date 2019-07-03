/*
 * test_485.c
 *
 *  Created on: 2018. 11. 14.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "proc_uart.h"


/* Private variables ---------------------------------------------------------*/

#define NUM_OF_DEBUGCHAR    64
#define	CMD_BUF_SIZE		128

uint8_t testMsgBuf[NUM_OF_DEBUGCHAR];

//uint8_t argc;
//char *argv[20];
volatile char Cmd[CMD_BUF_SIZE];

/* Global variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart3;
extern osSemaphoreId rs485SemaphoreIdHandle;


extern int get_arg(char *args);
/* Private function prototypes -----------------------------------------------*/

// RS485 RTS 1:TX, 0:RX
#define RS485_TX_ENABLE() {	HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_SET);}
#define RS485_TX_DISABLE() { HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_RESET);}

#define RS485_RX_ENABLE(x) RS485_TX_DISABLE(x)

void kputc2(uint8_t c)
{
	RS485_TX_ENABLE(); osDelay(1);
	osSemaphoreWait(rs485SemaphoreIdHandle, osWaitForever);
	HAL_UART_Transmit(&huart3, &c, 1, 1000);
	osSemaphoreRelease(rs485SemaphoreIdHandle);
	RS485_TX_DISABLE();
}

void kputs2(const char *pStr)
{
	int	num = strlen(pStr);

	if (num == 0) return;

	RS485_TX_ENABLE(); osDelay(1);
	osSemaphoreWait(rs485SemaphoreIdHandle, osWaitForever);
	memcpy(testMsgBuf, pStr, num);
	HAL_UART_Transmit(&huart3, (uint8_t *)testMsgBuf, num, 1000);
	osSemaphoreRelease(rs485SemaphoreIdHandle);
	RS485_TX_DISABLE();
}

uint8_t kgetc2(uint8_t *c)
{
	if (!EmptyQueue(&comm485_in_q))
	{
		*c = OutputQueue(&comm485_in_q);
		return 1;
	}
	else
		return 0;
}

void kprintf2(const char *fmt, ...)
{
	char buf[NUM_OF_DEBUGCHAR];
	static va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, NUM_OF_DEBUGCHAR, fmt, ap);
	va_end(ap);
	kputs2(buf);

}

int get_cmd2(char *Cmd)
{
	uint8_t	ch;
	int     pos=0;

	while(1)
	{
		while (!kgetc2(&ch)) osDelay(1);

		switch (ch)
		{
		case '\b' :
			if (pos > 0)
			{
				kputc2('\b');
				pos--;
			}
			else
				kputc2(0x07);
			break;
		case '\r' :
		case '\n' :
			*(Cmd+pos) = 0x00;
			kputc2((char)ch);
			return pos;
		default :
			kputc2(_toupper((char)ch));
			Cmd[pos++] = _toupper((char)ch);
			break;
		}

		if (pos > NUM_OF_DEBUGCHAR)
		{
			kputs2("\nout of length(enter command)");
			return -1;
		}
	}
}

void rs485TaskFunction(void)
{

  memset((void *)Cmd, 0, CMD_BUF_SIZE);
  kputs2("\r\n485> ");

  if(get_cmd2((char *)Cmd) > 0)
  {
	if(get_arg((char *)Cmd) > 0)
	{
		kputs2("\r\n");
		//exe_cmd(PORT_DEBUG);
	}
  }
}
