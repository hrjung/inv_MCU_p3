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

typedef enum
{
	NVM_INIT_PARAM_NONE = 0,
	NVM_INIT_PARAM_ALL,
	NVM_INIT_PARAM_CONFIG,
	NVM_INIT_PARAM_ERROR,
	NVM_INIT_PARAM_TIME,
} NVM_INIT_t;


// mainhandler task function
extern int8_t HDLR_handleDspError(void);
extern int8_t HDLR_readDspStatus(void);
extern int8_t HDLR_restoreRunStopFlagNFC(void);
extern int8_t HDLR_handleRunStopFlagNFC(void);
extern int8_t HDLR_handleRunStopFlagModbus(void);

extern int8_t HDLR_isFactoryModeEnabled(void);


extern void HDLR_setStopFlag(uint8_t flag);
extern int HDLR_isStopInProgress(void);


// NFC task function

extern void HDLR_setStartRunTime(void);
extern void HDLR_saveMotorRunTime(void);

extern int8_t HDLR_updatebyNfc(void);
extern int8_t HDLR_restoreNVM(void);
extern int8_t HDLR_updateParamNVM(void);
extern int8_t HDLR_initNVM(NVM_INIT_t init_type);
extern uint8_t HDLR_isNeedInitialize(void);

extern int8_t HDLR_updateTime(uint32_t cur_time);

extern int8_t HDLR_updateSysParam(int index);

#endif /* SRC_HANDLER_H_ */
