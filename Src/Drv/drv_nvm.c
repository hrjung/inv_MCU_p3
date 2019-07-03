/*
 * drv_nvm.c
 *
 *  Created on: 2019. 6. 3.
 *   mock implementation of NFC EEPROM driver
 *      Author: hrjung
 */

#include <stdint.h>
#include <stdbool.h>
#include <table.h>

#include "drv_nvm.h"


#define TABLE_SIZE_MAX	250


static int32_t nvm_table[TABLE_SIZE_MAX];


uint8_t NVM_read(int32_t addr, int32_t *value)
{
	uint8_t status=0;

	*value = nvm_table[addr];
	return status;
}

uint8_t NVM_write(int32_t addr, int32_t value)
{
	uint8_t status=0;

	nvm_table[addr] = value;

	return status;
}


void NVM_clear(void)
{
	int i;

	for(i=0; i<TABLE_SIZE_MAX; i++) nvm_table[i] = 0;

}
