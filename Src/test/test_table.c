/*
 * test_table.c
 *
 *  Created on: 2019. 6. 29.
 *      Author: skson8495
 */

#include "includes.h"

#ifdef SUPPORT_UNIT_TEST


#include <memory.h>

#include "unity.h"
#include "table.h"

extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t opt);
extern int8_t table_setFreqValue(PARAM_IDX_t idx, int32_t value, int16_t opt);

extern Param_t param_table[];


/*
 *		test item : table_setValue
 *
 *
 *
 */
void test_setValue(void)
{
	uint16_t value;
	int32_t exp_freq, freq_l;



}


#endif
