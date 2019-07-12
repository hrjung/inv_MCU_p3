/*
 * modbus_queue.h
 *
 *  Created on: 2019. 6. 27.
 *      Author: hrjung
 */

#ifndef SRC_MODBUS_QUEUE_H_
#define SRC_MODBUS_QUEUE_H_


#define MODBUS_BUF_SIZE		256

typedef struct {
	uint8_t 	empty;
	uint16_t	size;
	uint8_t 	packet[MODBUS_BUF_SIZE];
} MBQ_Transter_buffer_t;



extern void MBQ_init(void);
extern int MBQ_isEmptyReqQ(void);
extern int8_t MBQ_putReqQ(uint16_t size, uint8_t *data);
extern uint16_t MBQ_getReqQ(uint8_t *buf);

extern int MBQ_isEmptyRespQ(void);
extern int8_t MBQ_putRespQ(uint16_t size, uint8_t *data);
extern uint16_t MBQ_getRespQ(uint8_t *buf);

#endif /* SRC_MODBUS_QUEUE_H_ */
