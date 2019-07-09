/*
 * drv_dsp_spi.c
 *
 *  Created on: 2019. 6. 11.
 *      Author: hrjung
 */

/*
 *  This is for mock implementation for test only
 *
 */
#include "includes.h"

#include "main.h"
#include "cmsis_os.h"

//#include "dsp_comm.h"

extern uint16_t testResp[];
extern uint16_t recvMsg[];
extern int8_t send_status;
extern int8_t recv_status;
extern int16_t seq_cnt;


extern SPI_HandleTypeDef hspi3;

#ifdef SUPPORT_DRIVER_HW
HAL_StatusTypeDef SPI_writeDSP(uint16_t *pBuf, uint16_t len)
{
	uint8_t i;
	uint16_t checksum=0;
	HAL_StatusTypeDef status;

	for(i=0; i<len-1; i++)
		checksum += pBuf[i];
	pBuf[len-1] = checksum;

	HAL_GPIO_WritePin(MCU_nSCS_OUT3_GPIO_Port, MCU_nSCS_OUT3_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi3, (uint8_t *)&pBuf[0], len, 100);

	HAL_GPIO_WritePin(MCU_nSCS_OUT3_GPIO_Port, MCU_nSCS_OUT3_Pin, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef SPI_readDSP(uint16_t *rxBuf, uint16_t rxlen)
{
	//uint8_t i;
	uint16_t dummy[2];
	//uint16_t tmp;
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(MCU_nSCS_OUT3_GPIO_Port, MCU_nSCS_OUT3_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Receive(&hspi3, (uint8_t *)&dummy[0], 1, 100);
	status = HAL_SPI_Receive(&hspi3, (uint8_t *)&rxBuf[0], rxlen, 100);
	//status = HAL_SPI_Receive(&hspi3, (uint8_t *)&pBuf[1], 1, 100);

	HAL_GPIO_WritePin(MCU_nSCS_OUT3_GPIO_Port, MCU_nSCS_OUT3_Pin, GPIO_PIN_SET);


	if(dummy[0] == 0xAAAA && rxBuf[0] == 0x5555)
	{
//		for(i=0; i<rxLen; i++)
//		{
//			tmp = rxBuf[i];
//			rxBuf[i] = rx
//		}
	}

	return status;
}
#else

int8_t SPI_writeDSP(uint16_t *pBuf, uint16_t len)
{
	int8_t status=send_status;

	// only one time write

	return status;
}

int8_t SPI_readDSP(uint16_t *rxBuf, uint16_t rxlen)
{
	uint8_t i;
	uint16_t checksum=0;
	int8_t status=recv_status;

	for(i=0; i<rxlen; i++) recvMsg[i] = testResp[i];

	recvMsg[3] = seq_cnt;
	for(i=0; i<rxlen-1; i++)
		checksum += recvMsg[i];
	recvMsg[rxlen-1] = checksum;

	// only one time receive

	return status;
}
#endif



