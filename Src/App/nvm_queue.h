/*
 * nvm_queue.h
 *
 *  Created on: 2019. 6. 27.
 *      Author: hrjung
 */

#ifndef SRC_NVM_QUEUE_H_
#define SRC_NVM_QUEUE_H_


#include "table.h"


#define NVM_QUEUE_SIZE		(PARAM_TABLE_SIZE*2)

#define NVM_INVALID_ADDR		(-1)

typedef struct {
	uint16_t	addr;
	int32_t		value;
} NVM_Param_t;

typedef struct {
	uint16_t	index;
	int32_t		value;
} NVM_table_t;

typedef struct {
	int16_t count; //valid range : 0 ~ NVM_QUEUE_SIZE-1
	NVM_Param_t	to_nvm[NVM_QUEUE_SIZE];
} NVM_To_NFC_Queue_t;

typedef struct {
	int16_t count; //valid range : 0 ~ NVM_QUEUE_SIZE-1
	NVM_table_t	to_table[NVM_QUEUE_SIZE];
} NVM_To_Table_Queue_t;


extern void NVMQ_init(void);
extern int8_t NVMQ_isEmptyNfcQ(void);
extern int8_t NVMQ_enqueueNfcQ(uint16_t index, int32_t value);
extern int8_t NVMQ_dequeueNfcQ(uint16_t *addr, int32_t *value);

extern int8_t NVMQ_isEmptyTableQ(void);
extern int8_t NVMQ_enqueueTableQ(PARAM_IDX_t index, int32_t value);
extern int8_t NVMQ_dequeueTableQ(PARAM_IDX_t *index, int32_t *value);

#endif /* SRC_NVM_QUEUE_H_ */
