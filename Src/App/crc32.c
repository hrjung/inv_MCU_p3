/*
 * crc32.c
 *
 *  Created on: 2019. 6. 3.
 *      Author: hrjung
 */


#include "crc32.h"


static uint32_t crc_table[256];

void gen_crc_table(void)
{
	uint32_t i, j;
	uint32_t crc_accum;

	for (i=0;  i<256;  i++)
	{
		crc_accum = ( i << 24 );
		for ( j = 0;  j < 8;  j++ )
		{
			if ( crc_accum & 0x80000000L )
				crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
			else
				crc_accum = (crc_accum << 1);
		}
		crc_table[i] = crc_accum;
	}
}

uint32_t update_crc(uint32_t crc_accum, uint8_t* data_blk_ptr, uint32_t data_blk_size)
{
	uint32_t i, j;


	for (j=0; j<data_blk_size; j++)
	{
		i = ((int) (crc_accum >> 24) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	crc_accum = ~crc_accum;

	return crc_accum;
}
