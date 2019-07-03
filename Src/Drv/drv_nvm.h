/*
 * drv_nvm.h
 *
 *  Created on: 2019. 6. 3.
 *  mock implementation of NFC EEPROM driver
 *      Author: hrjung
 */

#ifndef SRC_DRV_NVM_H_
#define SRC_DRV_NVM_H_



extern void NVM_clear(void);
extern uint8_t NVM_read(int32_t addr, int32_t *value);
extern uint8_t NVM_write(int32_t addr, int32_t value);


#endif /* SRC_DRV_NVM_H_ */
