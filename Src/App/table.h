/*
 * table.h
 *
 *  Created on: 2019. 6. 3.
 *      Author: hrjung
 */

#ifndef SRC_TABLE_H_
#define SRC_TABLE_H_


#define NVM_TABLE_VERSION		0x301

#define NVM_BACKUP_FLAG_ADDR	0x5F8
#define NVM_BACKUP_START_ADDR	0x600

#define NVM_BACKUP_AVAILABLE_F	(0x5555)

typedef enum{
	value_type,
	freq_min_type,
	freq_max_type,
	accel_time_type,
	decel_time_type,
	acc_base_set_type,
	dir_cmd_type,
	dir_domain_type,

	ctrl_in_type,
	energy_save_type,
	pwm_freq_type,
	input_voltage_type,
	jmp_enable0_type,
	jmp_low0_type,
	jmp_high0_type,
	jmp_enable1_type,
	jmp_low1_type,
	jmp_high1_type,
	jmp_enable2_type,
	jmp_low2_type,
	jmp_high2_type,
	brake_type_type,
	brake_freq_type,
	dci_brk_freq_type,
	dci_brk_hold_type,
	dci_brk_time_type,
	dci_brk_rate_type,

	ovl_warn_limit_type,
	ovl_warn_dur_type,
	ovl_enable_type,
	ovl_trip_limit_type,
	ovl_trip_dur_type,
	regen_duty_type,
	regen_band_type,
	fan_onoff_type,
#ifdef SUPPORT_PASSWORD
	password_type,
	modify_lock_type,
#endif

	multi_Din_0_type,
	multi_Din_1_type,
	multi_Din_2_type,
	multi_val_0_type,
	multi_val_1_type,
	multi_val_2_type,
	multi_val_3_type,
	multi_val_4_type,
	multi_val_5_type,
	multi_val_6_type,
	multi_val_7_type,
	multi_Dout_0_type,
	multi_Dout_1_type,
	v_in_min_type,
	v_in_min_freq_type,
	v_in_max_type,
	v_in_max_freq_type,
	mb_address_type,
	baudrate_type,

	Rs_type,
	Rr_type,
	Ls_type,
	noload_current_type,
	rated_current_type,
	poles_type,
	model_type,
	motor_type_type,
	gear_ratio_type,

	err_code_1_type,
	err_status_1_type,
	err_current_1_type,
	err_freq_1_type,
	err_code_2_type,
	err_status_2_type,
	err_current_2_type,
	err_freq_2_type,
	err_code_3_type,
	err_status_3_type,
	err_current_3_type,
	err_freq_3_type,
	err_code_4_type,
	err_status_4_type,
	err_current_4_type,
	err_freq_4_type,
	err_code_5_type,
	err_status_5_type,
	err_current_5_type,
	err_freq_5_type,

	run_status1_type,
	run_status2_type,
	I_rms_type,
	run_freq_type,
	dc_voltage_type,
#ifdef SUPPORT_STATUS_TORQUE
	torque_value_type,
	torque_percent_type,
#endif	
	ipm_temperature_type,
	mtr_temperature_type,
	di_status_type,
	do_status_type,
	ai_status_type,
	motor_on_cnt_type,
	elapsed_hour_type,
	operating_hour_type,

	PARAM_TABLE_SIZE,
} PARAM_IDX_t ;//new


typedef enum{
	none_dsp = -1,

	value_dsp,
	freq_max_dsp,
	accel_time_dsp,
	decel_time_dsp,
	acc_base_set_dsp,

	dir_cmd_dsp,
	energy_save_dsp,
	pwm_freq_dsp,
	jmp_enable0_dsp,
	jmp_enable1_dsp,

	jmp_enable2_dsp,
	jmp_low0_dsp,
	jmp_low1_dsp,
	jmp_low2_dsp,
	jmp_high0_dsp,

	jmp_high1_dsp,
	jmp_high2_dsp,
	brake_type_dsp,
	brake_freq_dsp,
	dci_brk_freq_dsp,

	dci_brk_hold_dsp,
	dci_brk_time_dsp,
	dci_brk_rate_dsp,
	ovl_warn_limit_dsp,
	ovl_warn_dur_dsp,

	ovl_enable_dsp,
	ovl_trip_limit_dsp,
	ovl_trip_dur_dsp,
	regen_duty_dsp,
	regen_band_dsp,

	fan_onoff_dsp,
	motor_type_dsp,
	voltage_in_dsp,

	DSP_PARAM_SIZE,
}TABLE_DSP_PARAM_t;//new



typedef struct {
	const PARAM_IDX_t idx;
	const uint16_t addr;	// EEPROM address
	const uint16_t mb_addr;	// modbus address
	const int32_t initValue;
	int32_t minValue;
	int32_t maxValue;
	uint8_t isRW;		// RW or RO
	uint16_t ratio;		// for int -> 1, for float -> 10, instead of type defs
	uint8_t isWriterbleOnRunnuning;
	const TABLE_DSP_PARAM_t dsp_idx;
	int8_t (*param_func)(PARAM_IDX_t idx, int32_t value, int16_t opt);
} Param_t;


typedef enum{
	SYSTEM_PARAM_NFC_TAGGED,
	SYSTEM_PARAM_CRC_VALUE,
	SYSTEM_PARAM_IS_INITIATED,
	SYSTEM_PARAM_HAS_SYSTEM_ERROR,
	SYSTEM_PARAM_ENABLE_NFC_WRITER,
	SYSTEM_PARAM_NFC_TRYED,
	SYSTEM_PARAM_ON_MONITORING,
	SYSTEM_PARAM_RUN1_STOP2,
	SYSTEM_PARAM_INIT_NVM,
	SYSTEM_PARAM_BACKUP_CMD,

	SYSTEM_PARAM_SIZE,

} SYSTEM_PARAM_t;

typedef enum{
	CTRL_IN_NFC = 0,
	CTRL_IN_Digital,
	CTRL_IN_Analog_V,
#ifdef SUPPORT_DI_AI_CONTROL
	CTRL_IN_Din_Ain,
#endif
	CTRL_IN_Modbus,
	CTRL_IN_MAX
} CTRL_IN_t;

typedef enum {
	CMD_STOP = 0,
	CMD_RUN,

} CMD_RUN_t;

typedef enum {
	CMD_DIR_F = 0,
	CMD_DIR_R,
} CMD_DIR_t;

typedef enum {
	DIR_ALL,
	DIR_FORWARD_ONLY,
	DIR_REVERSE_ONLY,

} DIR_STATUS_t;

typedef enum{
	DIN_unuse = 0,
	DIN_run,
	DIN_direction,
	DIN_freq_low,
	DIN_freq_mid,
	DIN_freq_high,
	DIN_emergency_stop,
	DIN_external_trip,
	DIN_config_max,
} DIN_config_t;

typedef enum{
	DOUT_unuse = 0,
	DOUT_running,
	DOUT_stop,
	DOUT_trip_notify,
	DOUT_overload,
	DOUT_shaftbrake_on,

} DOUT_config_t;

typedef enum{
	VOLTAGE_380 = 0,
	VOLTAGE_400,
	VOLTAGE_415,
	VOLTAGE_480,

} Voltage_in_t;

typedef enum{
	MOTOR_NONE_TYPE = 0,
	MOTOR_1HP_4P_TYPE,
	MOTOR_2HP_4P_TYPE,
	MOTOR_3HP_4P_TYPE,
	MOTOR_MAX_TYPE,
} MOTOR_type_t;

typedef enum
{
	STATE_START = 0,
	STATE_STOP,
	STATE_ACCEL,
	STATE_DECEL,
	STATE_RUN
} INV_state_e;

enum {
	REQ_FROM_TEST = 0,
	REQ_FROM_DSP,
	REQ_FROM_NFC,
	REQ_FROM_EXTIO,
	REQ_FROM_MODBUS,
	REQ_FROM_KEPAD,

};


extern int8_t table_isInit(void);
//extern int8_t table_isNfcMonitoring(void);
//extern int8_t table_getNfcStatus(int32_t *tag_started, int32_t *tag_end);
//extern int8_t table_clearNfcStatus(void);

extern int8_t table_initNVM(void);

extern int32_t table_getValue(PARAM_IDX_t index);
extern uint16_t table_getRatio(PARAM_IDX_t index);
extern uint8_t table_getRW(PARAM_IDX_t index);

extern int8_t table_setStatusValue(PARAM_IDX_t index, int32_t value, int16_t option);
extern void table_setExtStatusValue(void);
extern int8_t table_setStatusDSP(void);
extern int8_t table_updateErrorDSP(uint16_t err_code, uint16_t status, float current, float freq);
extern int8_t table_updateCommError(uint16_t err_code);

extern int32_t table_getCtrllIn(void);

extern int table_isMotorStop(void);

#ifdef SUPPORT_PASSWORD
extern int table_isLocked(void);
#endif

extern int8_t table_runFunc(PARAM_IDX_t idx, int32_t value, int16_t opt);

extern int8_t table_updatebyTableQ(void);

extern void table_initParam(void);

#endif /* SRC_TABLE_H_ */
