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

enum
{
	NO_CHANGE = 0,
	WRITE_TO_NVM,
	WRITE_TO_TABLE,
};

typedef struct {
	const SYSTEM_PARAM_t idx;
	const uint16_t addr;	// EEPROM address
	int32_t value;			// stored value
	int8_t	need_update;
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
extern void NVM_setInit(void);
extern int8_t NVM_isInit(void);
extern int8_t NVM_isNfcMonitoring(void);
extern void NVM_clearNfcMonitoring(void);
extern int8_t NVM_getNfcStatus(int32_t *tag_started, int32_t *tag_end);
extern void NVM_clearNfcStatus(void);
extern int8_t NVM_getRunStopFlag(int32_t *run_stop);
extern void NVM_clearRunStopFlag(void);

#ifdef SUPPORT_PARAMETER_BACKUP
extern int8_t NVM_getBackupCmd(void);
extern void NVM_clearBackupCmd(void);
#endif

extern int8_t NVM_verifyCRC(uint32_t crc32_calc);
extern void NVM_setCRC(void);

extern int NVM_isSysParamUpdateRequred(void);
extern int NVM_isSysParamTableUpdateRequired(void);

#endif /* SRC_DRV_NVM_H_ */
