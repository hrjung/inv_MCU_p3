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

extern void NVM_clear(void);
extern uint8_t NVM_read(uint16_t addr, int32_t *value);
extern uint8_t NVM_write(int32_t addr, int32_t value);

extern uint8_t NVM_readParam(PARAM_IDX_t index, int32_t *value);
extern uint8_t NVM_writeParam(PARAM_IDX_t index, int32_t value);

extern int8_t NVM_initSystemParam(void);
extern uint8_t NVM_setInit(void);
extern int8_t NVM_isInit(void);
extern int32_t NVM_isMonitoring(void);
extern int8_t NVM_isNfcMonitoring(void);
extern int8_t NVM_clearNfcMonitoring(void);
extern int8_t NVM_getNfcStatus(int32_t *tag_started, int32_t *tag_end);
extern int8_t NVM_clearNfcStatus(void);
extern int8_t NVM_getRunStopFlag(int32_t *run_stop);
extern int8_t MVM_clearRunStopFlag(void);

extern int8_t NVM_verifyCRC(uint32_t crc32_calc);
extern int8_t NVM_setCRC(void);


#endif /* SRC_DRV_NVM_H_ */
