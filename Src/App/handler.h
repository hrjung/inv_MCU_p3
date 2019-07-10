/*
 * handler.h
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */

#ifndef SRC_HANDLER_H_
#define SRC_HANDLER_H_




extern int8_t HDLR_handleDspError(void);

extern int8_t HDLR_handleRunStopFlag(void);

// NFC task function
extern int8_t HDLR_updatebyNfc(void);
extern int8_t HDLR_restoreNVM(void);
extern int8_t HDLR_updateParamNVM(void);

#endif /* SRC_HANDLER_H_ */
