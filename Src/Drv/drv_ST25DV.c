/*
 * drv_ST25DV.c
 * 		: NFC tag chip I2C interface
 *
 *
 *  Created on: 2018. 6. 25.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include "drv_ST25DV.h"

/* Private variables ---------------------------------------------------------*/

#define NFC_ADDR_LEN		(2)
#define NFC_COMM_TIMOUT		(100)


int32_t i2c_rd_error=0;
int32_t i2c_wr_error=0;

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId I2CSemaphoreIdHandle;

/* Private function prototypes -----------------------------------------------*/

uint8_t I2C_writeData(uint8_t *value, uint16_t addr, uint16_t len)
{
	uint16_t i;
	uint8_t wrBuff[I2C_MAX_LEN];
	HAL_StatusTypeDef i2c_status = HAL_OK;

	if(len > I2C_MAX_LEN)
		return 0;

	for(i=0; i<len; i++) wrBuff[i] = value[i];

//	for(i=0; i<I2C_ERR_REPEAT_CNT; i++)
	{
		osSemaphoreWait(I2CSemaphoreIdHandle, osWaitForever);
		i2c_status = HAL_I2C_Mem_Write(&hi2c1, NFC_DevAddr_W, addr, (uint16_t)NFC_ADDR_LEN, (uint8_t *)&wrBuff[0], len, NFC_COMM_TIMOUT);
		osSemaphoreRelease(I2CSemaphoreIdHandle);
		//i2c_status = HAL_I2C_Master_Transmit(&hi2c1, NFC_DevAddr_W, (uint8_t *)wrBuff, (len+2), 100);
//		if(i2c_status == HAL_OK) break;
		osDelay(5);
	}

	if(i2c_status != HAL_OK)
		i2c_wr_error++;

	return (i2c_status == HAL_OK);
	//return (uint8_t)i2c_status;
}

uint8_t I2C_readData(uint8_t *value, uint16_t addr, uint16_t len)
{
	uint16_t i;
	uint8_t rdBuff[I2C_MAX_LEN];
	//uint8_t devAddr[2];
	HAL_StatusTypeDef i2c_status = HAL_OK;

	for(i=0; i<I2C_MAX_LEN; i++) rdBuff[i] = 0;

	if(len > I2C_MAX_LEN)
		return HAL_ERROR;

//	devAddr[0] = (uint8_t)((addr&0xFF00) >> 8);
//	devAddr[1] = (uint8_t)(addr&0x00FF);
	//for(i=0; i<I2C_ERR_REPEAT_CNT; i++)
	{
		osSemaphoreWait(I2CSemaphoreIdHandle, osWaitForever);
		i2c_status = HAL_I2C_Mem_Read(&hi2c1, NFC_DevAddr_R, addr, (uint16_t)NFC_ADDR_LEN, (uint8_t *)&rdBuff[0], len, NFC_COMM_TIMOUT);
		osSemaphoreRelease(I2CSemaphoreIdHandle);
//		i2c_status = HAL_I2C_Master_Transmit(&hi2c1, NFC_DevAddr_W, devAddr, 2, 100);
//		i2c_status = HAL_I2C_Master_Receive(&hi2c1, NFC_DevAddr_R, &rdBuff[0], len, 100);
//		if(i2c_status == HAL_OK) break;
		osDelay(5);
	}

	if(i2c_status == HAL_OK)
	{
		for(i=0; i<len; i++) value[i] = rdBuff[i];
	}
	else
		i2c_rd_error++;

	return (uint8_t)(i2c_status == HAL_OK);
	//return (uint8_t)i2c_status;
}
