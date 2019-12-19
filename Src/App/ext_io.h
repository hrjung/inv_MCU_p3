/*
 * ext_io.h
 *
 *  Created on: 2019. 6. 14.
 *      Author: hrjung
 */

#ifndef SRC_EXT_IO_H_
#define SRC_EXT_IO_H_


#define EXT_DIN_COUNT		3
#define EXT_DOUT_COUNT		2

#define EXT_AIN_COUNT		1
#define EXT_AOUT_COUNT		0

#define EXT_AIN_ADC_MIN		(60.0)
#define EXT_AIN_ADC_MAX		(3280.0)

#define EXT_DI_ACTIVE		0
#define EXT_DI_INACTIVE		1

typedef struct {
	uint8_t bit_L; 		// pin_number for bit_L, can be 0, 1, 2, ..  (EXT_DIN_COUNT for not assigned)
	uint8_t bit_M;
	uint8_t bit_H;
	uint8_t run_pin;
	uint8_t dir_pin;
	uint8_t emergency_pin;
	uint8_t trip_pin;
} DIN_PIN_NUM_t;


extern int32_t EXT_getDIValue(void);
extern int32_t EXT_getDOValue(void);
extern int32_t EXT_getAIValue(void);

//extern void EXT_DI_updateMultiDinPinIndex(uint8_t index, DIN_config_t func_set);
extern int8_t EXT_DI_setupMultiFuncDin(int index, DIN_config_t func_set, int16_t option);
extern uint8_t EXT_DI_convertMultiStep(void);
extern int8_t EXI_DI_handleEmergency(void);
extern int8_t EXI_DI_handleDin(int32_t ctrl_in);

extern int8_t EXT_DO_handleDout(void);

extern void EXT_AI_needReconfig(void);
extern int8_t EXT_AI_handleAin(void);

#ifdef SUPPORT_DI_AI_CONTROL
extern int8_t EXT_handleDAin(int32_t ctrl_in);
#endif

#endif /* SRC_EXT_IO_H_ */
