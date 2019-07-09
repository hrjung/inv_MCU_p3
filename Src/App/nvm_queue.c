/*
 * nvm_queue.c
 *
 *  Created on: 2019. 6. 27.
 *      Author: hrjung
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "proc_uart.h"
#include "table.h"
#include "nvm_queue.h"


NVM_To_Table_Queue_t table_q;
NVM_To_NFC_Queue_t nfc_q;


extern uint16_t table_getAddr(PARAM_IDX_t index);
extern uint16_t table_getStatusNvmAddr(PARAM_STATUS_IDX_t index);

void NVMQ_init(void)
{
	int i;
	nfc_q.count = 0;
	for(i=0; i<NVM_QUEUE_SIZE; i++)
	{
		nfc_q.to_nvm[i].addr = 0;
		nfc_q.to_nvm[i].value = 0;
	}

	table_q.count = 0;
	for(i=0; i<NVM_QUEUE_SIZE; i++)
	{
		table_q.to_table[i].index = 0;
		table_q.to_table[i].value = 0;
	}
}


int8_t NVMQ_isEmptyNfcQ(void)
{
	return (nfc_q.count == 0);
}

int8_t NVMQ_enqueueNfcQ(uint8_t type, uint16_t index, int32_t value)
{
	if(nfc_q.count >= NVM_QUEUE_SIZE-1)
	{
		kprintf(PORT_DEBUG, "NVMQ_enqueueNfcQ: nfc_q full error !!\r\n");
		return 0;
	}

	if(nfc_q.count < 0) nfc_q.count = 0; // silently fix

	if(type == NVM_QUEUE_DATA_TYPE)
	{
		if(index >= PARAM_TABLE_SIZE)
		{
			kprintf(PORT_DEBUG, "NVMQ_enqueueNfcQ: invalid data index(%d) error !!\r\n", index);
			return 0;
		}
		nfc_q.to_nvm[nfc_q.count].addr = table_getAddr(index);
	}
	else
	{
		if(index >= PARAM_STATUS_SIZE)
		{
			kprintf(PORT_DEBUG, "NVMQ_enqueueNfcQ: invalid status index(%d) error !!\r\n", index);
			return 0;
		}
		nfc_q.to_nvm[nfc_q.count].addr = table_getStatusNvmAddr(index);
	}

	memcpy(&nfc_q.to_nvm[nfc_q.count].value, &value, sizeof(int32_t));

#ifdef DEBUG_NVM_QUEUE
	kprintf(PORT_DEBUG, "add nfc_q count=%d, type=%d, addr=%d, value=%d\r\n", \
			nfc_q.count, type, nfc_q.to_nvm[nfc_q.count].addr, nfc_q.to_nvm[nfc_q.count].value);
#endif

	if(nfc_q.count < NVM_QUEUE_SIZE-1) nfc_q.count++;

	return 1;
}

int8_t NVMQ_dequeueNfcQ(uint16_t *addr, int32_t *value)
{
	if(nfc_q.count <= 0 || nfc_q.count > PARAM_TABLE_SIZE)
	{
		kprintf(PORT_DEBUG, "NVMQ_dequeueNfcQ: nfc_q count=%d error !!\r\n", nfc_q.count);
		return 0;
	}

	*addr =  nfc_q.to_nvm[0].addr;
	memcpy(value, &nfc_q.to_nvm[0].value, sizeof(int32_t));

#ifdef DEBUG_NVM_QUEUE
	kprintf(PORT_DEBUG, "get nfc_q count=%d, addr=%d, value=%d\r\n", \
			nfc_q.count, nfc_q.to_dsp[0].addr, nfc_q.to_dsp[0].value);
#endif

	int i;
	for(i=0; i<nfc_q.count; i++)
	{
		nfc_q.to_nvm[i].addr = nfc_q.to_nvm[i+1].addr;
		nfc_q.to_nvm[i].value = nfc_q.to_nvm[i+1].value;
	}
	nfc_q.to_nvm[i+1].addr = 0;


	if(nfc_q.count > 0) nfc_q.count--;
	if(nfc_q.count == 0) nfc_q.to_nvm[0].addr = 0;

	return 1;
}


/////////////////////////////////////////////////////////////////

int8_t NVMQ_isEmptyTableQ(void)
{
	return (table_q.count == 0);
}

int8_t NVMQ_enqueueTableQ(PARAM_IDX_t index, int32_t value)
{
	if(table_q.count >= NVM_QUEUE_SIZE-1)
	{
		kprintf(PORT_DEBUG, "NVMQ_enqueueTableQ: table_q full error !!\r\n");
		return 0;
	}

	if(table_q.count < 0) table_q.count = 0; // silently fix

	if(index < 0 || index >= PARAM_TABLE_SIZE)
	{
		kprintf(PORT_DEBUG, "NVMQ_enqueueTableQ: invalid address(%d) error !!\r\n", index);
		return 0;
	}

	table_q.to_table[table_q.count].index = index;
	memcpy(&table_q.to_table[table_q.count].value, &value, sizeof(float));

#ifdef DEBUG_NVM_QUEUE
	kprintf(PORT_DEBUG, "add table_q count=%d, index=%d, value=%d\r\n", \
			table_q.count, table_q.to_table[table_q.count].index, table_q.to_table[table_q.count].value);
#endif

	if(table_q.count < NVM_QUEUE_SIZE-1) table_q.count++;

	return 1;
}

int8_t NVMQ_dequeueTableQ(PARAM_IDX_t *index, int32_t *value)
{
	if(table_q.count <= 0 || table_q.count > NVM_QUEUE_SIZE)
	{
		kprintf(PORT_DEBUG, "NVMQ_dequeueTableQ: table_q count=%d error !!\r\n", table_q.count);
		return 0;
	}

	*index =  table_q.to_table[0].index;
	memcpy(value, &table_q.to_table[0].value, sizeof(int32_t));

#ifdef DEBUG_NVM_QUEUE
	kprintf(PORT_DEBUG, ("get table_q count=%d, index=%d, value=%d\r\n", \
			table_q.count, table_q.to_table[0].index, table_q.to_table[0].value);
#endif

	int i;
	for(i=0; i<table_q.count; i++)
	{
		table_q.to_table[i].index = table_q.to_table[i+1].index;
		table_q.to_table[i].value = table_q.to_table[i+1].value;
	}
	table_q.to_table[i+1].index = PARAM_TABLE_SIZE;


	if(table_q.count > 0) table_q.count--;
	if(table_q.count == 0) table_q.to_table[0].index = PARAM_TABLE_SIZE;

	return 1;
}
