/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DO_2_Pin GPIO_PIN_13
#define DO_2_GPIO_Port GPIOC
#define F_CMD_Pin GPIO_PIN_0
#define F_CMD_GPIO_Port GPIOC
#define ACC_INT2_Pin GPIO_PIN_0
#define ACC_INT2_GPIO_Port GPIOA
#define OP485_RTS_Pin GPIO_PIN_1
#define OP485_RTS_GPIO_Port GPIOA
#define OP485_TXD_Pin GPIO_PIN_2
#define OP485_TXD_GPIO_Port GPIOA
#define OP485_RXD_Pin GPIO_PIN_3
#define OP485_RXD_GPIO_Port GPIOA
#define ACC_nSCS_OUT2_Pin GPIO_PIN_4
#define ACC_nSCS_OUT2_GPIO_Port GPIOA
#define ACC_SCK_OUT2_Pin GPIO_PIN_5
#define ACC_SCK_OUT2_GPIO_Port GPIOA
#define ACC_SDI2_Pin GPIO_PIN_6
#define ACC_SDI2_GPIO_Port GPIOA
#define ACC_SDO2_Pin GPIO_PIN_7
#define ACC_SDO2_GPIO_Port GPIOA
#define DI_2_Pin GPIO_PIN_4
#define DI_2_GPIO_Port GPIOC
#define DI_3_Pin GPIO_PIN_5
#define DI_3_GPIO_Port GPIOC
#define DTM1_Pin GPIO_PIN_0
#define DTM1_GPIO_Port GPIOB
#define DTM2_Pin GPIO_PIN_1
#define DTM2_GPIO_Port GPIOB
#define AUX_OUT_Pin GPIO_PIN_2
#define AUX_OUT_GPIO_Port GPIOB
#define Modbus_TX_Pin GPIO_PIN_10
#define Modbus_TX_GPIO_Port GPIOB
#define Modbus_RX_Pin GPIO_PIN_11
#define Modbus_RX_GPIO_Port GPIOB
#define Modbus_RTS_Pin GPIO_PIN_14
#define Modbus_RTS_GPIO_Port GPIOB
#define R_LED_Pin GPIO_PIN_6
#define R_LED_GPIO_Port GPIOC
#define B_LED_Pin GPIO_PIN_7
#define B_LED_GPIO_Port GPIOC
#define G_LED_Pin GPIO_PIN_8
#define G_LED_GPIO_Port GPIOC
#define DI_1_Pin GPIO_PIN_9
#define DI_1_GPIO_Port GPIOC
#define STATUS_MCU_Pin GPIO_PIN_8
#define STATUS_MCU_GPIO_Port GPIOA
#define STM32_DBG_TX_Pin GPIO_PIN_9
#define STM32_DBG_TX_GPIO_Port GPIOA
#define STM32_DBG_RX_Pin GPIO_PIN_10
#define STM32_DBG_RX_GPIO_Port GPIOA
#define TH_MCU_1_Pin GPIO_PIN_11
#define TH_MCU_1_GPIO_Port GPIOA
#define TH_MCU_2_Pin GPIO_PIN_12
#define TH_MCU_2_GPIO_Port GPIOA
#define MCU_nSCS_OUT3_Pin GPIO_PIN_15
#define MCU_nSCS_OUT3_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOC
#define DO_1_Pin GPIO_PIN_12
#define DO_1_GPIO_Port GPIOC
#define RF_WIP_Pin GPIO_PIN_8
#define RF_WIP_GPIO_Port GPIOB
#define RF_WIP_EXTI_IRQn EXTI9_5_IRQn
#define MTD1_Pin GPIO_PIN_9
#define MTD1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
