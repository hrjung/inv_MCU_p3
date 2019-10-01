/*
 * drv_gpio.h
 *
 *  Created on: 2019. 6. 11.
 *      Author: skson8495
 */

#ifndef SRC_DRV_GPIO_H_
#define SRC_DRV_GPIO_H_


#define DTM_DSP_TRIP_ERROR		3

#define EXT_DIN_SAMPLE_CNT			10
#define EXT_AIN_SAMPLE_CNT			16

#define LED_COLOR_OFF	0
#define LED_COLOR_R		1
#define LED_COLOR_G		2
#define LED_COLOR_B		3
#define LED_COLOR_RG	4
#define LED_COLOR_RB	5
#define LED_COLOR_GB	6
#define LED_COLOR_RGB	7

typedef struct
{
	uint8_t blink;
	uint8_t onoff;
} LED_status_t;

extern void UTIL_setTestPin(uint8_t index, uint8_t onoff);
extern void UTIL_setLED(uint8_t color, uint8_t blink_on);
extern void UTIL_handleLED(void);

extern uint8_t UTIL_readNotifyDTM(void);
extern void UTIL_setMTDpin(uint8_t onoff);

extern void UTIL_readDin(void);
extern void UTIL_writeDout(uint8_t index, uint8_t onoff);

extern void UTIL_readADC(uint16_t adc_sample);
extern void UTIL_startADC(void);

extern uint8_t UTIL_isDspError(void);
extern void UTIL_readDspErrorPin(void);
extern void UTIL_setMTDpin(uint8_t onoff);

#endif /* SRC_DRV_GPIO_H_ */
