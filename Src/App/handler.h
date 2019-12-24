/*
 * handler.h
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */

#ifndef SRC_HANDLER_H_
#define SRC_HANDLER_H_


#define RUN_STOP_FLAG_IDLE		0
#define RUN_STOP_FLAG_RUN		1
#define RUN_STOP_FLAG_STOP		2

// mainhandler task function
extern int8_t HDLR_handleDspError(void);
extern int8_t HDLR_readDspStatus(void);
extern int8_t HDLR_restoreRunStopFlagNFC(void);
extern int8_t HDLR_handleRunStopFlagNFC(void);
extern int8_t HDLR_handleRunStopFlagModbus(void);

extern int8_t HDLR_isFactoryModeEnabled(void);


extern void HDLR_setStopFlag(uint8_t flag);
extern int HDLR_isStopInProgress(void);

#ifdef SUPPORT_PARAMETER_BACKUP
extern void HDLR_setBackupFlagModbus(int8_t flag);
extern void HDLR_clearBackupFlagModbus(void);
extern int8_t HDLR_getBackupFlag(void);
extern int8_t HDLR_isBackupEnabled(void);
extern int HDLR_isBackupAvailable(void);
extern int8_t HDLR_clearBackupFlag(void);

extern int8_t HDLR_backupParameter(void);
extern int8_t HDLR_restoreParameter(void);
#endif


// NFC task function

extern void HDLR_setStartRunTime(void);
extern void HDLR_saveMotorRunTime(void);

extern int8_t HDLR_updatebyNfc(void);
extern int8_t HDLR_restoreNVM(void);
extern int8_t HDLR_updateParamNVM(void);
extern int8_t HDLR_initNVM(void);

extern int8_t HDLR_updateTime(uint32_t cur_time);

extern int8_t HDLR_updateSysParam(int index);

#endif /* SRC_HANDLER_H_ */
