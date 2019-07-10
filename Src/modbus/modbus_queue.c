/*
 * modbus_queue.c
 *
 *  Created on: 2019. 6. 27.
 *      Author: hrjung
 */


#include "includes.h"

#include "table.h"
#include "modbus_queue.h"


MBQ_Transter_buffer_t req_q, resp_q;


void MBQ_init(void)
{
	int i;

	req_q.size = 0;
	resp_q.size = 0;
	for(i=0; i<MODBUS_BUF_SIZE; i++)
	{
		req_q.packet[i] = 0;
		resp_q.packet[i] = 0;
	}
}

int MBQ_isEmptyReqQ(void)
{
	return (req_q.empty == 1);
}


int8_t MBQ_putReqQ(uint16_t size, uint8_t *data)
{
	int i;

	if(req_q.empty == 0) return 0;

	req_q.size = size;
	for(i=0; i<size; i++) req_q.packet[i] = data[i];
	req_q.empty = 0;

	return 1;
}

uint16_t MBQ_getReqQ(uint8_t *buf)
{
	int i;

	if(req_q.empty == 1) return 0; // empty

	for(i=0; i<req_q.size; i++) buf[i] = req_q.packet[i];
	req_q.empty = 1;

	for(i=0; i<MODBUS_BUF_SIZE; i++) req_q.packet[i]=0; //clear

	return req_q.size;
}

int MBQ_isRespQReady(void)
{
	return (resp_q.empty == 0);
}

int8_t MBQ_putRespQ(uint16_t size, uint8_t *data)
{
	int i;

	if(resp_q.empty == 0) return 0;

	resp_q.size = size;
	for(i=0; i<size; i++) resp_q.packet[i] = data[i];
	resp_q.empty = 0;

	return 1;
}

uint16_t MBQ_getRespQ(uint8_t *buf)
{
	int i;

	if(resp_q.empty == 1) return 0; // empty

	for(i=0; i<resp_q.size; i++) buf[i] = resp_q.packet[i];
	resp_q.empty = 1;

	for(i=0; i<MODBUS_BUF_SIZE; i++) resp_q.packet[i]=0; //clear

	return resp_q.size;
}
