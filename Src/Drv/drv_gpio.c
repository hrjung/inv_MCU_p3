/*
 * drv_gpio.c
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */

#include "includes.h"


#include "main.h"
#include "cmsis_os.h"

#include "table.h"

#include "drv_gpio.h"
#include "ext_io.h"


// DTM pin
#define DSP_TRIP_ERROR		3


//extern uint8_t DTM_readNotify(void);

static uint8_t di_val[3];
uint8_t dtm_test_val=0;

static uint8_t din[EXT_DIN_COUNT][EXT_DIN_SAMPLE_CNT] = {0};

extern uint8_t mdin_value[];

void printDBG(char *str)
{
	puts(str);putchar('\n');
}

int8_t UTIL_isDspErrorState(void)
{
	//uint8_t errFlag = DTM_readNotify();
	uint8_t errFlag=0;

	if(errFlag == DSP_TRIP_ERROR) return 1;
	else	return 0;
}

void UTIL_setTestPin(uint8_t index, uint8_t onoff)
{
	if(index)
		HAL_GPIO_WritePin(TH_MCU_2_GPIO_Port, TH_MCU_2_Pin, (GPIO_PinState)onoff);
	else
		HAL_GPIO_WritePin(TH_MCU_1_GPIO_Port, TH_MCU_1_Pin, (GPIO_PinState)onoff);
}

// support only 1 color or off
void UTIL_setLED(uint8_t color, uint8_t blink_on)
{
	switch(color)
	{
	case LED_COLOR_OFF:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);
#endif
		printf("LED off\n");
		break;

	case LED_COLOR_R:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);
#endif
		printf("LED R on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_G:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);
#endif
		printf("LED G on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_B:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
#endif
		printf("LED B on, blink=%d\n", blink_on);
		break;
#if 0
	case LED_COLOR_RG:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);
#endif
		printf("LED RG on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_RB:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
#endif
		printf("LED RB on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_GB:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
#endif
		printf("LED GB on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_RGB:
#ifdef SUPPORT_DRIVER_HW
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
#endif
		printf("LED RGB on, blink=%d\n", blink_on);
		break;
#endif
	default :
		break;
	}
}

uint8_t UTIL_readNotifyDTM(void)
{
	uint8_t value=0, dtm1_val=0, dtm2_val=0;

#ifdef SUPPORT_DRIVER_HW
	dtm1_val = (uint16_t)HAL_GPIO_ReadPin(DTM1_GPIO_Port, DTM1_Pin);
	dtm2_val = (uint16_t)HAL_GPIO_ReadPin(DTM2_GPIO_Port, DTM2_Pin);

	value = (dtm2_val << 1) | dtm1_val;
#else
	value = dtm_test_val;
#endif

	return value;
}

void UTIL_setMTDpin(uint8_t onoff)
{
#ifdef SUPPORT_DRIVER_HW
	HAL_GPIO_WritePin(MTD1_GPIO_Port, MTD1_Pin, (GPIO_PinState)onoff);
#else

#endif
}

// debug test only
void UTIL_readDin(void)
{
#ifdef SUPPORT_DRIVER_HW
	static int din_idx=0;
	int i, di_sum[EXT_DIN_COUNT];

	// read DI
	din_idx = din_idx%EXT_DIN_SAMPLE_CNT;
	din[0][din_idx] = (uint8_t)HAL_GPIO_ReadPin(DI_1_GPIO_Port, DI_1_Pin);
	din[1][din_idx] = (uint8_t)HAL_GPIO_ReadPin(DI_2_GPIO_Port, DI_2_Pin);
	din[2][din_idx] = (uint8_t)HAL_GPIO_ReadPin(DI_3_GPIO_Port, DI_3_Pin);
	din_idx++;

	for(i=0; i<EXT_DIN_COUNT; i++) di_sum[i] = 0;
	for(i=0; i<EXT_DIN_SAMPLE_CNT; i++)
	{
		di_sum[0] += din[0][i];
		di_sum[1] += din[1][i];
		di_sum[2] += din[2][i];
	}

	for(i=0; i<EXT_DIN_COUNT; i++)
	{
		if(di_sum[i] == 0) mdin_value[i] = 0;
		else if(di_sum[i] == EXT_DIN_SAMPLE_CNT) mdin_value[i] = 1;
		// else no change
	}
#else
	int i;

	for(i=0; i<EXT_DIN_COUNT; i++) mdin_value[i] = di_val[i];
#endif

}

void UTIL_writeDout(uint8_t index, uint8_t onoff)
{
	GPIO_PinState value;

	if(index > 2) {printf("DI index error %d\n", index); return ;}

	value = onoff ? GPIO_PIN_RESET : GPIO_PIN_SET;
#ifdef SUPPORT_DRIVER_HW
	if(index == 0)
		HAL_GPIO_WritePin(DO_1_GPIO_Port, DO_1_Pin, value);
	else
		HAL_GPIO_WritePin(DO_2_GPIO_Port, DO_2_Pin, value);
#endif
}
