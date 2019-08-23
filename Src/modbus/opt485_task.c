/*
 * opt485_task.c
 *
 *  Created on: 2019. 8. 12.
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


// in case of baudrate > 19200, inter-frame delay is 1.75ms
// baudrate < 19200, need to calculate as 3.5 character
#define MODBUS_INTER_FRAME_DELAY		35

// RS485 RTS 1:TX, 0:RX
#define OPT485_TX_ENABLE() { HAL_GPIO_WritePin(OP485_RTS_GPIO_Port, OP485_RTS_Pin, GPIO_PIN_SET);}
#define OPT485_TX_DISABLE() { HAL_GPIO_WritePin(OP485_RTS_GPIO_Port, OP485_RTS_Pin, GPIO_PIN_RESET);}


uint16_t opt_timeout = 0;
uint16_t opt_downcounter = 0;

uint8_t opt_start_flag = 0;
uint8_t opt_frame_received = 0;
uint16_t opt_err_code = 0;

#ifdef SUPPORT_PRODUCTION_TEST_MODE
typedef enum
{
	P_TEST_SEND_PACKET,
	P_TEST_WAIT_RECEIVE_RESP,
	P_TEST_END,
	P_TEST_NEXT,

} p_test_state_p;

// test packet
const uint8_t mbus_packet[5][18] = {
		{ 0x1, 0x03, 0x9C, 0xB8, 0, 0x1, 0x2A, 0x7F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // read 1 holding register 40120
		{ 0x1, 0x04, 0x9C, 0xB8, 0, 0x3, 0x1E, 0x7E, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // read 3 input register, 40120
		{ 0x1, 0x04, 0x9C, 0xE0, 0, 0x7, 0x9E, 0x6E, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // read 7 input register, 40160 (inverter status)
		{ 0x1, 0x06, 0x9C, 0xA4, 2, 0x58, 0xE6, 0xE3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // write 1 reigster, 40100, 600
		{ 0x1, 0x10, 0x9D, 0x5A, 0, 0x3, 0x06, 0, 1, 0, 0x96, 0, 0x14, 0xDE, 0xAC, 0, 0}, // write 3 registers, 40282, 1, 150, 20
};

uint16_t mbus_packet_len[5] = {8, 8, 8, 8, 15};
uint8_t t_result[5] = {0,}; // 485 loopback test result

uint8_t p_test_enabled=0; // flag for start 485 loopback test in JIG test
uint8_t p_test_ended=0; // flag for finish 485 loopback test in JIG test

extern uint8_t jig_test_result[];
#endif

MODBUS_SLAVE_QUEUE optBufRx, optBufTx;

/* Global variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim10;
extern osSemaphoreId rs485SemaphoreIdHandle;

extern uint8_t	mb_slaveAddress;

extern int MB_isCRC_OK(uint8_t *buf, uint32_t len);
/* Private function prototypes -----------------------------------------------*/


/*
 * 	timer related function
 */

void OPT_initTimer(uint16_t timeout_value)
{
	opt_timeout = timeout_value;
}

void OPT_enableTimer(void)
{
	opt_downcounter = opt_timeout;
	HAL_TIM_Base_Start_IT(&htim10);
}

void OPT_disableTimer(void)
{
	HAL_TIM_Base_Stop_IT(&htim10);
}

void OPT_init(void)
{
	int i;

	OPT_initTimer(MODBUS_INTER_FRAME_DELAY); // 1.75ms for 3.5 char

	optBufRx.wp = 0;
	optBufTx.wp = 0;
	for(i=0; i<MODBUS_BUF_SIZE; i++)
	{
		optBufRx.buf[i] = 0;
		optBufTx.buf[i] = 0;
	}

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // rs485

	// default enable RX
	OPT485_TX_DISABLE();
}


void OPT_readByte(uint8_t rcv_char)
{

	if(opt_start_flag == 0)
	{
		optBufRx.wp = 0;
		optBufRx.buf[optBufRx.wp++] = rcv_char;
  		opt_start_flag = 1;
  		OPT_enableTimer();

	}
	else
	{
		if(optBufRx.wp < MODBUS_BUF_SIZE)
		{
			optBufRx.buf[optBufRx.wp++] = rcv_char;
			OPT_enableTimer();
		}
		else
			opt_err_code = MOD_NORMAL_RX_ERR;
	}
}

void OPT_writeRespPacket(int len)
{
	if (len <= 1) return;

	OPT485_TX_ENABLE(); osDelay(1);
	osSemaphoreWait(rs485SemaphoreIdHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t *)optBufTx.buf, len, 1000);
	osSemaphoreRelease(rs485SemaphoreIdHandle);
	OPT485_TX_DISABLE();
}


void OPT_processTimerExpired(void)
{
	if(opt_start_flag == 1)
	{
		opt_start_flag = 0;
		opt_frame_received = 1;

		//HAL_GPIO_TogglePin(TH_MCU_1_GPIO_Port, TH_MCU_1_Pin); // toggle test bit
	}
	OPT_disableTimer();
}

int OPT_isValidRecvPacket(void)
{
	if(optBufRx.buf[0] != mb_slaveAddress) return 0;

	// CRC error
	if(!MB_isCRC_OK(optBufRx.buf, optBufRx.wp)) return 0;

	return 1;
}

#ifdef SUPPORT_PRODUCTION_TEST_MODE
void OPT_setTestPacket(int pk_index)
{
	uint16_t i, len = mbus_packet_len[pk_index];

	// clear Tx
	optBufTx.wp = 0;
	for(i=0; i<MODBUS_BUF_SIZE; i++)
	{
		optBufTx.buf[i] = 0;
	}

	optBufTx.wp = len;
	for(i=0; i<len; i++)
		optBufTx.buf[i] = mbus_packet[pk_index][i];
}

uint8_t OPT_verifyPacket(int pk_index)
{
	int i;

	for(i=0; i<mbus_packet_len[pk_index]; i++)
		if(optBufRx.buf[i] != mbus_packet[pk_index][i]) return 0;

	return 1;
}

uint8_t OPT_is485TestStarted(void)
{
	return p_test_enabled;
}

uint8_t OPT_is485TestEnded(void)
{
	return p_test_ended;
}

void OPT_start485Test(void)
{
	p_test_enabled = 1;
}

#endif


void OPT_TaskFunction(void)
{
#ifdef SUPPORT_PRODUCTION_TEST_MODE
	static p_test_state_p p_state=P_TEST_SEND_PACKET;
	static int pkt_idx=0;
	int res=0;

	if(p_test_enabled && p_test_ended == 0) // test in progress
	{
		switch(p_state)
		{
		case P_TEST_SEND_PACKET:
			osDelay(500); // add delay
			// set packet
			OPT_setTestPacket(pkt_idx);

			// send packet
			OPT_writeRespPacket(optBufTx.wp);
			p_state = P_TEST_WAIT_RECEIVE_RESP;
			break;

		case P_TEST_WAIT_RECEIVE_RESP:
			if(opt_frame_received)
			{
				if(OPT_isValidRecvPacket() == 0)
				{opt_frame_received=0; kprintf(PORT_DEBUG, " Rvc index=%d err \r\n", pkt_idx); return;}

				kprintf(PORT_DEBUG, "RX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
					optBufRx.buf[0], optBufRx.buf[1], optBufRx.buf[2], optBufRx.buf[3], optBufRx.buf[4]);

				// verify packet
				t_result[pkt_idx] = OPT_verifyPacket(pkt_idx);

				opt_frame_received=0;

				pkt_idx++;
				if(pkt_idx >= 5)	p_state = P_TEST_END;
				else			p_state = P_TEST_SEND_PACKET;

			}
			break;

		case P_TEST_END:
			kprintf(PORT_DEBUG, " test result %d, %d, %d, %d, %d \r\n", \
					t_result[0], t_result[1], t_result[2], t_result[3], t_result[4]);

			res = t_result[0]+t_result[1]+t_result[2]+t_result[3]+t_result[4];
			if(res != 5) jig_test_result[3] = 1; // set error
			// move to next test
			p_state = P_TEST_NEXT;
			break;

		case P_TEST_NEXT:
			p_test_ended = 1;
			p_test_enabled = 0;
			osDelay(100);
			break;
		}
	}
	else
#endif
	{
		// get data from 485
		if(opt_frame_received)
		{
			// check received frame
			// wrong slave address
			if(OPT_isValidRecvPacket() == 0) {opt_frame_received=0; return;}

			kprintf(PORT_DEBUG, "RX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
				optBufRx.buf[0], optBufRx.buf[1], optBufRx.buf[2], optBufRx.buf[3], optBufRx.buf[4]);

			opt_frame_received=0;
		}
	}

}
