/*
 * error.h
 *
 *  Created on: 2019. 6. 10.
 *      Author: skson8495
 */

#ifndef SRC_ERROR_H_
#define SRC_ERROR_H_


#define ERROR_COUNT_MAX		5

// should be matched with DSP error code
typedef enum {
	TRIP_REASON_NONE	 = 0,
	TRIP_REASON_IPM_FAULT,
	TRIP_REASON_VDC_UNDER,
	TRIP_REASON_VDC_OVER,
	TRIP_REASON_OVERLOAD,

	TRIP_REASON_IPM_OVER_TEMP,
	TRIP_REASON_MTR_OVER_TEMP,
	TRIP_REASON_USER_STOP,
	TRIP_REASON_EXTERNAL_TRIP,
	TRIP_REASON_OVER_CURRENT,

	TRIP_REASON_I_PHASE_MISS,
	TRIP_REASON_V_PHASE_MISS,
	TRIP_REASON_INT_RELAY_FAULT,


	TRIP_REASON_ARRAY_ERR =		20,
	TRIP_REASON_ACCEL_ERR,
	TRIP_REASON_FREQ_RANGE_ERR,
	TRIP_REASON_RPM_RANGE_ERR,
	TRIP_REASON_INPUT_VOLT_ERR,

	TRIP_REASON_REGEN_CALC_ERR,
	TRIP_REASON_OFFSET_ERR,
	TRIP_REASON_MCU_ERR,


	TRIP_REASON_MAX	= 30,

	TRIP_REASON_MCU_UNKNOWN	= 40,
	TRIP_REASON_MCU_INIT,
	TRIP_REASON_MCU_INPUT,			// external trip or emergency stop
	TRIP_REASON_MCU_SETVALUE,		// EEPROM error
	TRIP_REASON_MCU_COMM_FAIL,

	TRIP_REASON_MCU_ERR_TEST,

} TRIP_REASON_t;

typedef struct {
	const int32_t addr;
	int32_t value;
} ERR_NVM_int_t;

typedef struct {
	const int32_t addr;
	float value_f;
} ERR_NVM_float_t;

typedef struct {
	ERR_NVM_int_t 	date;
	ERR_NVM_int_t 	code;
	ERR_NVM_int_t	status;
	ERR_NVM_float_t	current;
	ERR_NVM_float_t	freq;
} ERR_NVM_info_t;

typedef struct {
	int32_t 	date;
	int32_t 	code;
	int32_t 	status;

	float		current;
	float		freq;
} ERR_info_t;

extern uint8_t ERR_isErrorState(void);
extern uint8_t ERR_isCommError(void);
extern uint16_t ERR_isNvmError(void);
extern uint8_t ERR_getErrorState(void);
extern void ERR_setErrorState(TRIP_REASON_t err_code);

#endif /* SRC_ERROR_H_ */
