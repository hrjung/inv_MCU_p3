/*
 * drv_gpio.c
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */

#include "includes.h"


#include "main.h"
#include "cmsis_os.h"

#include "proc_uart.h"

#include "table.h"

#include "drv_gpio.h"
#include "ext_io.h"



//extern uint8_t DTM_readNotify(void);
extern uint8_t ERR_isErrorState(void);

#ifndef SUPPORT_DRIVER_HW
static uint8_t di_val[3];
#endif
uint8_t dtm_test_val=0;

LED_status_t LED_state[3] =
{
	{0, GPIO_PIN_RESET},	// R
	{0, GPIO_PIN_RESET},	// G
	{0, GPIO_PIN_RESET},	// B
};

static uint8_t LED_update_f=0;

static uint8_t din[EXT_DIN_COUNT][EXT_DIN_SAMPLE_CNT] = {0};
static uint8_t dtm[2][EXT_DIN_SAMPLE_CNT] = {0};

uint8_t dtm_value[2];
uint8_t mtd_pin=0;

uint16_t ain_val[EXT_AIN_SAMPLE_CNT];
uint32_t ain_sum=0;
extern ADC_HandleTypeDef hadc1;

extern uint8_t mdin_value[];

uint16_t ain_sample_val[EXT_AIN_SAMPLE_CNT];
//uint32_t ain_sample_sum=0;
//uint16_t adc_sample;

//extern uint16_t ain_sample_val[];
extern uint16_t adc_value;

extern int table_isWarning(void);


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
	static uint8_t color_state=LED_COLOR_OFF;
	static uint8_t blink_state=0;

	if(color == color_state && blink_on == blink_state) return; // no change

	LED_update_f = 1;
	switch(color)
	{
	case LED_COLOR_OFF:
		LED_state[0].onoff = GPIO_PIN_RESET;
		LED_state[0].blink = 0;
		LED_state[1].onoff = GPIO_PIN_RESET;
		LED_state[1].blink = 0;
		LED_state[2].onoff = GPIO_PIN_RESET;
		LED_state[2].blink = 0;
		//printf("LED off\n");
		break;

	case LED_COLOR_R:
		LED_state[0].onoff = GPIO_PIN_SET;
		LED_state[0].blink = blink_on;
		LED_state[1].onoff = GPIO_PIN_RESET;
		LED_state[2].onoff = GPIO_PIN_RESET;
		//printf("LED R on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_G:
		LED_state[0].onoff = GPIO_PIN_RESET;
		LED_state[1].onoff = GPIO_PIN_SET;
		LED_state[1].blink = blink_on;
		LED_state[2].onoff = GPIO_PIN_RESET;
		//printf("LED G on, blink=%d\n", blink_on);
		break;

	case LED_COLOR_B:
		LED_state[0].onoff = GPIO_PIN_RESET;
		LED_state[1].onoff = GPIO_PIN_RESET;
		LED_state[2].onoff = GPIO_PIN_SET;
		LED_state[2].blink = blink_on;
		//printf("LED B on, blink=%d\n", blink_on);
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

	color_state = color;
	blink_state = blink_on;
}

void UTIL_handleLED(void)
{
	if(table_isWarning()) // alarm : RED on
	{
		UTIL_setLED(LED_COLOR_R, 0);
	}

	if(ERR_isErrorState()) // trip : RED blinking
	{
		UTIL_setLED(LED_COLOR_R, 1);
	}

	if(LED_update_f == 0) return ; // no change

	if(LED_state[0].onoff == GPIO_PIN_SET)
	{
		if(LED_state[0].blink)
			HAL_GPIO_TogglePin(R_LED_GPIO_Port, R_LED_Pin);
		else
			HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
	}
	else
		HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);;

	if(LED_state[1].onoff == GPIO_PIN_SET)
	{
		if(LED_state[1].blink)
			HAL_GPIO_TogglePin(G_LED_GPIO_Port, G_LED_Pin);
		else
			HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
	}
	else
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);

	if(LED_state[2].onoff == GPIO_PIN_SET)
	{
		if(LED_state[2].blink)
			HAL_GPIO_TogglePin(B_LED_GPIO_Port, B_LED_Pin);
		else
			HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
	}
	else
		HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);

	LED_update_f=0; // clear after update done.
}

uint8_t UTIL_readDTMpin(void)
{
	return (dtm_value[1] << 1) | dtm_value[0];
}

uint8_t UTIL_isDspError(void)
{
	uint8_t value=0;

	value = (dtm_value[1] << 1) | dtm_value[0];

	return (value == DTM_DSP_TRIP_ERROR);
}

void UTIL_setMTDpin(uint8_t onoff)
{
	mtd_pin = onoff;
#ifdef SUPPORT_DRIVER_HW
	HAL_GPIO_WritePin(MTD1_GPIO_Port, MTD1_Pin, (GPIO_PinState)mtd_pin);
#else

#endif
}


void UTIL_readDspErrorPin(void)
{
	static int dtm_idx=0;
	int i, dtm_sum[2];

	// read DI
	dtm_idx = dtm_idx%EXT_DIN_SAMPLE_CNT;
	dtm[0][dtm_idx] = (uint8_t)HAL_GPIO_ReadPin(DTM1_GPIO_Port, DTM1_Pin);
	dtm[1][dtm_idx] = (uint8_t)HAL_GPIO_ReadPin(DTM2_GPIO_Port, DTM2_Pin);
	dtm_idx++;

	dtm_sum[0] = 0;
	dtm_sum[1] = 0;
	for(i=0; i<EXT_DIN_SAMPLE_CNT; i++)
	{
		dtm_sum[0] += dtm[0][i];
		dtm_sum[1] += dtm[1][i];
	}

	for(i=0; i<2; i++)
	{
		if(dtm_sum[i] == 0) dtm_value[i] = 0;
		else if(dtm_sum[i] == EXT_DIN_SAMPLE_CNT) dtm_value[i] = 1;
		// else no change
	}
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

	if(index > 2) {kprintf(PORT_DEBUG, "DI index error %d \r\n", index); return ;}

	value = onoff ? GPIO_PIN_RESET : GPIO_PIN_SET;
#ifdef SUPPORT_DRIVER_HW
	if(index == 0)
		HAL_GPIO_WritePin(DO_1_GPIO_Port, DO_1_Pin, value);
	else
		HAL_GPIO_WritePin(DO_2_GPIO_Port, DO_2_Pin, value);
#endif
}

void UTIL_startADC(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ain_sample_val, EXT_AIN_SAMPLE_CNT);
}

void UTIL_stopADC(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
}

uint16_t UTIL_getMaxADC(void)
{
	int i;
	uint16_t max=0;

	for(i=0; i<EXT_AIN_SAMPLE_CNT; i++)
	{
		if(max < ain_val[i]) max = ain_val[i];
	}

	return max;
}

uint16_t UTIL_getMinADC(void)
{
	int i;
	uint16_t min=65535; // fix bug

	for(i=0; i<EXT_AIN_SAMPLE_CNT; i++)
	{
		if(min > ain_val[i]) min = ain_val[i];
	}

	return min;
}

int8_t ain_ready_flag=0;

void UTIL_readADC(uint16_t adc_sample)
{
	static int ain_idx=0;
	uint16_t min=0, max=0;
	int i;

	if(ain_idx == EXT_AIN_SAMPLE_CNT) ain_ready_flag = 1;

	ain_idx = ain_idx%EXT_AIN_SAMPLE_CNT;
	ain_val[ain_idx] = adc_sample&0x0FFF;
	ain_idx++;

	ain_sum=0;
	for(i=0; i<EXT_AIN_SAMPLE_CNT; i++) ain_sum += (uint32_t)ain_val[i];

	// remove max, min value from ADC samples
	max = UTIL_getMaxADC();
	min = UTIL_getMinADC();
	ain_sum -= max;
	ain_sum -= min;

	adc_value = (uint16_t)(ain_sum/(EXT_AIN_SAMPLE_CNT-2));

}

void UTIL_getAdcSamples(void)
{
	int i;
	uint16_t adc_sample=0;
	uint32_t ain_sample_sum=0;

	for(i=0; i<EXT_AIN_SAMPLE_CNT; i++) ain_sample_val[i] = (ain_sample_val[i]&0x0FFF);
	ain_sample_sum = 0;
	for(i=0; i<EXT_AIN_SAMPLE_CNT; i++) ain_sample_sum += (uint32_t)ain_sample_val[i];
	adc_sample = (uint16_t)(ain_sample_sum/EXT_AIN_SAMPLE_CNT);

	UTIL_readADC(adc_sample);
}

void UTIL_setAOUT(uint16_t volt)
{
	// set Voltage output
}

