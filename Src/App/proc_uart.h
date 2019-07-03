/******************************************************************************
**  FILENAME:       proc_uart.h
**
**  PURPOSE:        
**  LAST MODIFIED:  2016.08.
******************************************************************************/ 

#ifndef  __PROC_UART_H__
#define  __PROC_UART_H__

#include "cmsis_os.h"
#include "main.h"

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/

#ifdef   PROC_UART_GLOBALS
#define  PROC_UART_EXT
#else
#define  PROC_UART_EXT  extern
#endif

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

// define dport value
#define	DPORT_RS232			1


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern osSemaphoreId debugSemaphoreIdHandle;
extern osSemaphoreId rs485SemaphoreIdHandle;

#define hUartDbg 	huart1

#define PORT_DEBUG			1


#define S_QUEUE_SIZE		128
typedef struct {
	uint16_t	rp, wp;
	uint8_t	buf[S_QUEUE_SIZE];
} S_QUEUE;

/* Define DEBUG MASK Value */

#define DBG_MASK_APP		(1<<0)
#define DBG_MASK_SPIRX		(1<<1)
#define DBG_MASK_SPITX		(1<<2)
#define DBG_MASK_SPI		(1<<3)
#define DBG_MASK_SPIERR		(1<<4)
#define DBG_MASK_TIME		(1<<5)

#define DBG_MASK_RFREPVAL	(1<<10)
#define DBG_MASK_RFMSG		(1<<11)
#define DBG_MASK_RFTX		(1<<12)
#define DBG_MASK_RFRX		(1<<13)
#define DBG_MASK_RFRXERR	(1<<14)//2018-11-14
#define DBG_MASK_LOSS		(1<<15)
#define DBG_MASK_NEW		(1<<16)

#define DBG_MASK_ADC		(1<<17)
#define DBG_MASK_PROG		(1<<18)
#define DBG_MASK_19			(1<<19)
#define DBG_MASK_MBM		(1<<20)		// Modbus Master
#define DBG_MASK_MBS		(1<<21)		// Modbus Slave
#define DBG_MASK_PNT		(1<<22)		// Point Command
#define DBG_MASK_EC			(1<<23)		// Encoder Input
#define DBG_MASK_STAT		(1<<24)		// Modbus Status

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/
//PROC_UART_EXT 	uint8_t		dbg_in;			// debug Rx
//PROC_UART_EXT 	uint8_t		uart_a_in;		// uart A Rx (Arduino UART)
//PROC_UART_EXT 	uint8_t		uart_b_in;		// uart B Rx (Nucleo, ST ZIO)

PROC_UART_EXT 	S_QUEUE		dbg_in_q;		// debug Rx in Queue
PROC_UART_EXT 	S_QUEUE		comm485_in_q;		// rs485s in Queue


#ifdef   PROC_UART_GLOBALS
uint32_t		uart_errCnt[1] = {0};	// Error Count
uint32_t		debugMask=0; //DBG_MASK_RFTX|DBG_MASK_SPITX|DBG_MASK_PNT|DBG_MASK_PROG;
#else
extern uint32_t		uart_errCnt[1];
extern uint32_t		debugMask;		// debugMask
#endif

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/
#ifdef __cplusplus
extern "C" {
#endif
int FullQueue(S_QUEUE *q);
int EmptyQueue(S_QUEUE *q);
int CountQueue(S_QUEUE *q);
int InputQueue(S_QUEUE *q, uint8_t ch);
uint8_t OutputQueue(S_QUEUE *q);

void initUarts(void);
uint8_t kgetc(uint8_t dport, uint8_t *c);

void kputc(uint8_t dport, uint8_t c);
void kputs(uint8_t dport, const char *pStr);
void kprintf(uint8_t dport, const char *fmt,...);
void kprintHex(uint8_t dport, const char *str, uint8_t *hex, uint8_t size);

void _kputs(const char *pStr);
void _kprintf(const char *fmt,...);

void dputs(uint32_t mask, const char *pStr);
//void dprintf(uint32_t mask, const char *fmt,...);
#ifdef __cplusplus
}
#endif

#endif                                                          /* End of module include.                               */
