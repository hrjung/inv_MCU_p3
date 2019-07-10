/******************************************************************************
**  FILENAME:       proc_uart.c
**
**  PURPOSE:        
**  LAST MODIFIED:  2019.03.
******************************************************************************/ 
#define  PROC_UART_GLOBALS

#include "includes.h"

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "proc_uart.h"
#include "debug.h"



/********************************************************************************
* FUNCTION	 :    
* DESCRIPTION: 
* ARGUMENTS	 : 
* RETURNS	 : 
********************************************************************************/ 
int FullQueue(S_QUEUE *q)
{
	uint16_t	pos;

    pos = q->wp + 1;
    if(pos>=S_QUEUE_SIZE) pos=0;
    
    if (q->rp == pos)
    	return 1;
    else
    	return 0;
}

int EmptyQueue(S_QUEUE *q)
{
    if (q->rp == q->wp)
    	return 1;
    else
    	return 0;
}

int CountQueue(S_QUEUE *q)
{
    if (q->rp <= q->wp)
    	return (q->wp - q->rp);
    else
		return (S_QUEUE_SIZE + q->wp - q->rp);
}

int InputQueue(S_QUEUE *q, uint8_t ch)
{
	
	q->buf[q->wp++] = ch;
	if(q->wp>=S_QUEUE_SIZE) q->wp = 0;
    return 1;
}

uint8_t OutputQueue(S_QUEUE *q)
{
	uint8_t temp = q->buf[q->rp++];
	if(q->rp>=S_QUEUE_SIZE) q->rp = 0;
	return temp;
}


/********************************************************************************
* FUNCTION	 : 
* DESCRIPTION: 
* ARGUMENTS	 : 
* RETURNS	 : 
********************************************************************************/ 
void initUarts(void)
{
	__HAL_UART_ENABLE_IT(&hUartDbg, UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&huartNex, UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&huartNex, UART_IT_TXE);
}

#define	MAX_BUFF_SIZE	64
/**********************************************************************************************************
*     kputc, kputs, kgetc, kprintf
**********************************************************************************************************
*/
void kputc(uint8_t dport, uint8_t c)
{
  if (dport == PORT_DEBUG){
	osSemaphoreWait(debugSemaphoreIdHandle, osWaitForever);
	HAL_UART_Transmit(&hUartDbg, &c, 1, 1000);
	osSemaphoreRelease(debugSemaphoreIdHandle);
  }

}

//#pragma default_variable_attributes = @ "DATA_TCM"
static char txBuf[MAX_BUFF_SIZE];

//#pragma default_variable_attributes =
void kputs(uint8_t dport, const char *pStr)
{
	int	num = strlen(pStr);
	if(num == 0) return;
	if(num > MAX_BUFF_SIZE-1) return;
	if (dport == PORT_DEBUG){
		osSemaphoreWait(debugSemaphoreIdHandle, osWaitForever);
		memcpy(txBuf, pStr, num);
		HAL_UART_Transmit_DMA(&hUartDbg, (uint8_t *)txBuf, num);
		//HAL_UART_Transmit(&hUartDbg, (uint8_t *)txBuf, num, 1000);
		//osSemaphoreRelease(debugSemaphoreIdHandle);
	}

}

uint8_t kgetc(uint8_t dport, uint8_t *c)
{
	if (!EmptyQueue(&dbg_in_q))
	{
		*c = OutputQueue(&dbg_in_q);
		return 1;
	}
	else
		return 0;
}

void kprintf(uint8_t dport, const char *fmt, ...)
{
	char buf[MAX_BUFF_SIZE];
	static va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, MAX_BUFF_SIZE, fmt, ap);
	va_end(ap);
	kputs(dport, buf);
	
}

void kprintHex(uint8_t dport, const char *str, uint8_t *hex, uint8_t size)
{
	char buf[MAX_BUFF_SIZE];
	int i;
	char temp[4];

	uint8_t sizeMax = (MAX_BUFF_SIZE - 1) / 3 - strlen(str);
	if (size > sizeMax) size = sizeMax;

	strcpy(buf, str);
	for (i = 0; i < size; i++)
	{
		sprintf(temp, " %02X", hex[i]);
		strcat(buf, temp);
	}
	kputs(DPORT_RS232, buf);
}

// for Boot Sequence only
void _kputs(const char *pStr)
{
	int	num = strlen(pStr);
	if (num == 0) return;
	HAL_UART_Transmit(&hUartDbg, (uint8_t *)pStr, num, num);
}

// for Boot Sequence only
void _kprintf(const char *fmt, ...)
{
	char buf[MAX_BUFF_SIZE];
	static va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, MAX_BUFF_SIZE, fmt, ap);
	va_end(ap);
	_kputs(buf);
	
}

void dputs(uint32_t mask, const char *pStr)
{
	if(debugMask&mask)
	{
		kputs(DPORT_RS232, pStr);
	}
}

#if 0
void dprintf(uint32_t mask, const char *fmt, ...)
{
	char buf[MAX_BUFF_SIZE];
	static va_list ap;
	if(mask & debugMask)
	{
		va_start(ap, fmt);
		vsnprintf(buf, MAX_BUFF_SIZE, fmt, ap);
		va_end(ap);
		kputs(DPORT_RS232, buf);
	}
}
#endif

