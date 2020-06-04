/*
 * drv_nvm.h
 *
 *  Created on: 2019. 6. 3.
 *  mock implementation of NFC EEPROM driver
 *      Author: hrjung
 */

#ifndef SRC_DRV_NVM_H_
#define SRC_DRV_NVM_H_


#define 	NVM_NOK		0
#define 	NVM_OK		1

#define 	NVM_SYS_PARAM_UPDATE_RETRY_MAX	10


enum
{
	NO_CHANGE = 0,
	WRITE_TO_NVM,
	WRITE_TO_TABLE,
};

typedef struct {
	const SYSTEM_PARAM_t idx;
	const uint16_t addr;	// EEPROM address
	int8_t	need_update;	// require NVM write
	int8_t 	retry_cnt;		// write retry count
} Param_sys_t;

extern void NVM_clear(void);
extern uint8_t NVM_read(uint16_t addr, int32_t *value);
extern uint8_t NVM_write(int32_t addr, int32_t value);

extern uint8_t NVM_readParam(PARAM_IDX_t index, int32_t *value);
extern uint8_t NVM_writeParam(PARAM_IDX_t index, int32_t value);

extern int8_t NVM_readTime(void);
extern int8_t NVM_initTime(void);

extern int8_t NVM_initError(void);
extern int8_t NVM_initSystemParam(void);
//extern void NVM_setInit(void);
//extern int8_t NVM_isInit(void);
extern int8_t NVM_getNfcMonitoring(void);
extern void NVM_clearNfcMonitoring(void);
//extern int8_t NVM_getNfcStatus(int32_t *tag_end);
extern int8_t NVM_getNfcStatus(int32_t *tag_started, int32_t *tag_end);
extern void NVM_clearNfcStatus(void);
extern int8_t NVM_getRunStopFlag(int32_t *run_stop);
extern void NVM_clearRunStopFlag(void);

extern uint16_t NVM_getSystemParamAddr(uint16_t index);
extern int32_t NVM_getSystemParamValue(uint16_t index);
extern void NVM_clearSysParamUpdateFlag(uint16_t index);
extern int NVM_isSysParamNeedUpdate(uint16_t index);

extern void NVM_increaseSysParamRetryCnt(uint16_t index);
extern int8_t NVM_getSysParamRetryCnt(uint16_t index);

extern int8_t NVM_setMotorRunCount(uint32_t run_count);
extern int8_t NVM_setMotorDevCounter(uint32_t r_time);
extern int8_t NVM_setMotorRunTimeMinute(uint32_t r_time);

//extern int8_t NVM_verifyCRC(uint32_t crc32_calc);
extern int8_t NVM_verifyCRC(uint16_t crc16_calc);
extern void NVM_setCRC(void);

extern void NVM_setMotorStatus(int32_t status);
extern int NVM_getSysParamUpdateIndex(void);

extern void NVM_getCommandParam(void);
#ifdef SUPPORT_INIT_PARAM
extern uint8_t NVM_isInitNvmNfc(void);
extern uint8_t NVM_getInitSysParam(void);
extern void NVM_clearInitParamCmd(void);
#endif

extern uint8_t NVM_isResetEnabled(void);
extern void NVM_clearResetCmd(void);


extern void NVM_initSystemFlagStartup(void);

#endif /* SRC_DRV_NVM_H_ */
