/*
 * modbus_task.c
 *
 *  Created on: 2019. 7. 4.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/
#include "includes.h"

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "proc_uart.h"
#include "modbus_func.h"
#include "modbus_queue.h"
#include "table.h"


// modbus broadcast address always accept
#define MB_BRAODCAST_ADDR		(0)

// in case of baudrate > 19200, inter-frame delay is 1.75ms
// baudrate < 19200, need to calculate as 3.5 character
#define MODBUS_INTER_FRAME_DELAY		35

// RS485 RTS 1:TX, 0:RX
#define RS485_TX_ENABLE() {	HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_SET);}
#define RS485_TX_DISABLE() { HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_RESET);}


uint16_t mb_timeout = 0;
uint16_t mb_downcounter = 0;

uint8_t mb_start_flag = 0;
uint8_t mb_frame_received = 0;
uint16_t mb_err_code = 0;

uint8_t reset_enabled_f=0;

uint32_t mb_baudrate[6] = {2400, 4800, 9600, 19200, 38400, 115200};
uint16_t mb_frame_delay[6] = {35*4, 35*2, MODBUS_INTER_FRAME_DELAY, MODBUS_INTER_FRAME_DELAY, MODBUS_INTER_FRAME_DELAY, MODBUS_INTER_FRAME_DELAY};

MODBUS_SLAVE_QUEUE mbBufRx, mbBufTx;

/* Global variables ---------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;
extern osSemaphoreId mbus485SemaphoreIdHandle;

extern uint8_t reset_requested_f;
extern uint8_t reset_cmd_send_f;
extern uint8_t	mb_slaveAddress;

extern uint32_t timer_100ms;

#ifdef SUPPORT_PRODUCTION_TEST_MODE
extern uint8_t p_test_enabled;
#endif

extern int MB_isCRC_OK(uint8_t *buf, uint32_t len);
/* Private function prototypes -----------------------------------------------*/


/*
 * 	timer related function
 */

void MB_initTimer(int32_t b_index)
{
	mb_timeout = mb_frame_delay[b_index];
}

void MB_enableTimer(void)
{
	mb_downcounter = mb_timeout;
	HAL_TIM_Base_Start_IT(&htim7);
}

void MB_disableTimer(void)
{
	HAL_TIM_Base_Stop_IT(&htim7);
}

void MB_UART_init(uint32_t baudrate_index)
{

	huart3.Instance = USART3;
	huart3.Init.BaudRate = mb_baudrate[(int)baudrate_index];
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

void MB_init(void)
{
	int i;
	int32_t b_index=2;

	mb_slaveAddress = (uint8_t)table_getValue(mb_address_type);
	b_index = table_getValue(baudrate_type);

	//b_index = 2; //// fix only 9600bps for exhibition
	MB_UART_init((uint32_t)b_index);
	MB_initTimer(b_index);

	kprintf(PORT_DEBUG, "MB init s_addr=%d, baud=%d \r\n", mb_slaveAddress, (int)mb_baudrate[b_index]);

	mbBufRx.wp = 0;
	mbBufTx.wp = 0;
	for(i=0; i<MODBUS_BUF_SIZE; i++)
	{
		mbBufRx.buf[i] = 0;
		mbBufTx.buf[i] = 0;
	}

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // rs485

	// enable RX
	RS485_TX_DISABLE();
}

// read byte from UART
void MB_readByte(uint8_t rcv_char)
{

	if(mb_start_flag == 0) // first byte
	{
		mbBufRx.wp = 0;
		mbBufRx.buf[mbBufRx.wp++] = rcv_char;
  		mb_start_flag = 1;
  		MB_enableTimer();
	}
	else
	{
		if(mbBufRx.wp < MODBUS_BUF_SIZE)
		{
			mbBufRx.buf[mbBufRx.wp++] = rcv_char;
			MB_enableTimer();
		}
		else
			mb_err_code = MOD_NORMAL_RX_ERR;
	}
}

void MB_writeRespPacket(int len)
{
	if (len <= 1) return;

	RS485_TX_ENABLE(); osDelay(1);
	osSemaphoreWait(mbus485SemaphoreIdHandle, osWaitForever);
//	memcpy(testMsgBuf, pStr, num);
	HAL_UART_Transmit(&huart3, (uint8_t *)mbBufTx.buf, len, 1000);
	osSemaphoreRelease(mbus485SemaphoreIdHandle);
	RS485_TX_DISABLE();
}


void MB_processTimerExpired(void)
{
	if(mb_start_flag == 1)
	{
		mb_start_flag = 0;
		mb_frame_received = 1; // modbus frame completed
	}
	MB_disableTimer();
}

int MB_isValidRecvPacket(void)
{
	if(mbBufRx.buf[0] != mb_slaveAddress && mbBufRx.buf[0] != MB_BRAODCAST_ADDR) return 0; // slave address or broadcast check

	if(mbBufRx.wp < 4) return 0; // discard invalid packet

	// CRC error
	if(!MB_isCRC_OK(mbBufRx.buf, mbBufRx.wp)) return 0; // CRC check

	return 1;
}

void MB_TaskFunction(void)
{
	int8_t result=0;
//	RTC_TimeTypeDef sTime;
//	uint8_t time_val;

	// get data from 485
	if(mb_frame_received)
	{
		// check received frame
		// wrong slave address
		if(MB_isValidRecvPacket() == 0) {mb_frame_received=0; return;}

//		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//		time_val = sTime.Seconds;
//		kprintf(PORT_DEBUG, "cnt=%d RX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
//			time_val, mbBufRx.buf[0], mbBufRx.buf[1], mbBufRx.buf[2], mbBufRx.buf[3], mbBufRx.buf[4]);

		kprintf(PORT_DEBUG, "%d RX: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%0x\r\n",
				timer_100ms, mbBufRx.buf[0], mbBufRx.buf[1], mbBufRx.buf[2], mbBufRx.buf[3], mbBufRx.buf[4], mbBufRx.buf[5]);

#ifdef SUPPORT_PRODUCTION_TEST_MODE
		if(p_test_enabled)
		{
			int i;

			osDelay(100);
			for(i=0; i<mbBufRx.wp; i++)  mbBufTx.buf[i] = mbBufRx.buf[i]; // copy Rx data to Tx

			mbBufTx.wp = mbBufRx.wp;
			MB_writeRespPacket(mbBufTx.wp);

		}
		else
#endif
		// put packet to req_q
		result = MBQ_putReqQ(mbBufRx.wp, mbBufRx.buf);

		mb_frame_received=0;
	}

	// response packet ready
	if(MBQ_isEmptyRespQ() == 0)
	{
		mbBufTx.wp = MBQ_getRespQ(mbBufTx.buf);

		MB_writeRespPacket(mbBufTx.wp);

		if(reset_requested_f)
		{
			osDelay(500); // delay till sending response
			reset_enabled_f=1;
			reset_requested_f=0;
		}

//		kprintf(PORT_DEBUG, "TX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
//				mbBufTx.buf[0], mbBufTx.buf[1], mbBufTx.buf[2], mbBufTx.buf[3], mbBufTx.buf[4]);
	}

}
