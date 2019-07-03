/*
 * drv_ST25DV.h
 *
 *  Created on: 2018. 6. 22.
 *      Author: hrjung
 */

#ifndef APP_DRV_ST25DV_H_
#define APP_DRV_ST25DV_H_

#include "main.h"
#include "stm32f1xx_hal.h"

// NFC device address
#define NFC_DevAddr_R		(uint16_t)(0xA6)
#define NFC_DevAddr_W		(uint16_t)(0xA7)

#define NFC_DevSysAddr_R	(uint16_t)(0xAE)
#define NFC_DevSysAddr_W	(uint16_t)(0xAF)

// one sector length
#define I2C_MAX_LEN		(128)

extern uint8_t I2C_writeData(uint8_t *value, uint16_t addr, uint16_t len);
extern uint8_t I2C_readData(uint8_t *value, uint16_t addr, uint16_t len);

#endif /* APP_DRV_ST25DV_H_ */
