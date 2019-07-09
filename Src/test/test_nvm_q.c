/*
 * test_nvm_q.c
 *
 *  Created on: 2019. 6. 10.
 *      Author: hrjung
 */
#include "includes.h"

#ifdef SUPPORT_UNIT_TEST

#include <memory.h>

#include "unity.h"
#include "nvm_queue.h"


extern NVM_To_Table_Queue_t table_q;
extern NVM_To_NFC_Queue_t nfc_q;


extern uint16_t table_getAddr(PARAM_IDX_t index);
extern uint16_t table_getStatusNvmAddr(PARAM_STATUS_IDX_t index);

/*
 * 		test items
 *
 * 		1. enqueue single data, check count, result
 * 		2. invalid address(= -1), should error
 * 		3. invalid address(>= PARAM_TABLE_SIZE), should error
 * 		4. dequeue data, check value, count, address
 * 		5. enqueue single int32 data, check count, result
 * 		6. dequeue data, check value, count, address
 * 		7. dequeue with no queue data : should error
 *
*/

void test_nfc_q_basic(void)
{
	int8_t i8_result, i8_exp;
	PARAM_IDX_t idx;
	uint16_t addr, exp_addr;
	int32_t val, exp;

	NVMQ_init();

	i8_exp = 1;
	i8_result = NVMQ_isEmptyNfcQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// normal single enqueue : OK
	idx = value_type;
	val = 300;
	i8_exp = 1;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, nfc_q.count); // count = 1

	// none_dsp address : NOK
	idx = -1;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// range over address : NOK
	idx = PARAM_TABLE_SIZE;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// normal single dequeue : OK
	val = 0.0;
	exp = 300;
	exp_addr = table_getAddr(value_type);
	i8_exp = 1;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_addr, addr);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // count = 0

	// normal int32 single enqueue : OK
	idx = mb_address_type;
	val = 200;
	i8_exp = 1;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, nfc_q.count); // count = 1

	// normal int32 single dequeue : OK
	val = 0;
	exp = 200;
	exp_addr = table_getAddr(mb_address_type);
	i8_exp = 1;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_addr, addr);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // count = 0

	i8_exp = 1;
	i8_result = NVMQ_isEmptyNfcQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// no data but try dequeue : NOK
	i8_exp = 0;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // still count = 0



	// normal single enqueue : OK
	idx = run_status1_type;
	val = 0;
	i8_exp = 1;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_STATUS_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, nfc_q.count); // count = 1

	// none_dsp address : NOK
	idx = -1;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_STATUS_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// range over address : NOK
	idx = PARAM_STATUS_SIZE;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_STATUS_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// normal single dequeue : OK
	val = 0;
	exp = 0;
	exp_addr = table_getStatusNvmAddr(run_status1_type);
	i8_exp = 1;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_addr, addr);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // count = 0

	// normal int32 single enqueue : OK
	idx = mtr_temperature_type;
	val = 1;
	i8_exp = 1;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_STATUS_TYPE, idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, nfc_q.count); // count = 1

	// normal int32 single dequeue : OK
	val = 0;
	exp = 1;
	exp_addr = table_getStatusNvmAddr(mtr_temperature_type);
	i8_exp = 1;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_addr, addr);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // count = 0

	i8_exp = 1;
	i8_result = NVMQ_isEmptyNfcQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// no data but try dequeue : NOK
	i8_exp = 0;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count); // still count = 0
}

/*
 * 		test items
 *
 * 		7. enqueue multiple data, check count increased correctly
 * 		8. dequeue data is valid
 * 		9. enqueue over count PARAM_TABLE_SIZE, should error
 *
 */

void test_nfc_q_muliple(void)
{
	int i;
	int8_t i8_result, i8_exp;
	//PARAM_IDX_t idx, exp_idx;
	int32_t val, exp;
	uint16_t addr, exp_addr;
	PARAM_IDX_t test_idx[] = {
		multi_val_3_type,
		freq_max_type,
		dir_cmd_type,
		jmp_low1_type,
		pwm_freq_type,

		dci_brk_freq_type,
		ovl_warn_limit_type,
		multi_Dout_0_type,
		err_current_3_type,
		operating_hour_type,
	};

	int32_t test_value[] = {
		300,
		800,
		0,
		100,
		550,

		50,
		150,
		2,
		25,
		1500,
	};

	NVMQ_init();

	// normal multiple enqueue : OK, count 10
	i8_exp = 1;
	for(i=0; i<10; i++)
	{
		i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, test_idx[i], test_value[i]);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(i+1, nfc_q.count);
	}

	// normal multiple dequeue : OK
	for(i=0; i<10; i++)
	{
		val = 0;
		exp = test_value[i];
		exp_addr = table_getAddr(test_idx[i]);
		i8_exp = 1;
		i8_result = NVMQ_dequeueNfcQ(&addr, &val);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(exp, val);
		TEST_ASSERT_EQUAL_INT(exp_addr, addr);
		TEST_ASSERT_EQUAL_INT(10-(i+1), nfc_q.count);
	}

	// q is empty but try dequeue : error
	i8_exp = 0;
	i8_result = NVMQ_dequeueNfcQ(&addr, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(0, nfc_q.count);

	// verify empty
	i8_exp = 1;
	i8_result = NVMQ_isEmptyNfcQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// enqueue over PARAM_TABLE_SIZE : NOK
	i8_exp = 1;
	for(i=0; i<PARAM_TABLE_SIZE-1; i++)
	{
		i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, test_idx[0], test_value[0]);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(i+1, nfc_q.count);
	}
	i8_exp = 0;
	i8_result = NVMQ_enqueueNfcQ(NVM_QUEUE_DATA_TYPE, test_idx[0], test_value[0]);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(PARAM_TABLE_SIZE-1, nfc_q.count);
}


/*
 * 		test items
 *
 * 		1. enqueue single data, check count, result
 * 		2. invalid address(= -1), should error
 * 		3. invalid address(>= PARAM_TABLE_SIZE), should error
 * 		4. dequeue data, check value, count, address
 * 		5. enqueue single int32 data, check count, result
 * 		6. dequeue data, check value, count, address
 * 		7. dequeue with no queue data : should error
 *
*/

void test_table_q_basic(void)
{
	int8_t i8_result, i8_exp;
	PARAM_IDX_t idx, exp_idx;
	int32_t val, exp;

	NVMQ_init();

	i8_exp = 1;
	i8_result = NVMQ_isEmptyTableQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// normal single enqueue : OK
	idx = value_type;
	val = 300;
	i8_exp = 1;
	i8_result = NVMQ_enqueueTableQ(idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, table_q.count); // count = 1

	// none_dsp address : NOK
	idx = -1;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueTableQ(idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// range over address : NOK
	idx = PARAM_TABLE_SIZE;
	val = 100;
	i8_exp = 0;
	i8_result = NVMQ_enqueueTableQ(idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// normal single dequeue : OK
	val = 0.0;
	exp = 300;
	exp_idx = value_type;
	i8_exp = 1;
	i8_result = NVMQ_dequeueTableQ(&idx, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_idx, idx);
	TEST_ASSERT_EQUAL_INT(0, table_q.count); // count = 0

	// normal int32 single enqueue : OK
	idx = mb_address_type;
	val = 200;
	i8_exp = 1;
	i8_result = NVMQ_enqueueTableQ(idx, val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(1, table_q.count); // count = 1

	// normal int32 single dequeue : OK
	val = 0;
	exp = 200;
	exp_idx = mb_address_type;
	i8_exp = 1;
	i8_result = NVMQ_dequeueTableQ(&idx, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(exp, val);
	TEST_ASSERT_EQUAL_INT(exp_idx, idx);
	TEST_ASSERT_EQUAL_INT(0, table_q.count); // count = 0

	i8_exp = 1;
	i8_result = NVMQ_isEmptyTableQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// no data but try dequeue : NOK
	i8_exp = 0;
	i8_result = NVMQ_dequeueTableQ(&idx, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(0, table_q.count); // still count = 0
}

/*
 * 		test items
 *
 * 		7. enqueue multiple data, check count increased correctly
 * 		8. dequeue data is valid
 * 		9. enqueue over count PARAM_TABLE_SIZE, should error
 *
 */

void test_table_q_muliple(void)
{
	int i;
	int8_t i8_result, i8_exp;
	PARAM_IDX_t idx, exp_idx;
	int32_t val, exp;
	PARAM_IDX_t test_idx[] = {
		multi_val_3_type,
		freq_max_type,
		dir_cmd_type,
		jmp_low1_type,
		pwm_freq_type,

		dci_brk_freq_type,
		ovl_warn_limit_type,
		multi_Dout_0_type,
		err_current_3_type,
		operating_hour_type,
	};

	int32_t test_value[] = {
		300,
		800,
		0,
		100,
		550,

		50,
		150,
		2,
		25,
		1500,
	};

	NVMQ_init();

	// normal multiple enqueue : OK, count 10
	i8_exp = 1;
	for(i=0; i<10; i++)
	{
		i8_result = NVMQ_enqueueTableQ(test_idx[i], test_value[i]);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(i+1, table_q.count);
	}

	// normal multiple dequeue : OK
	for(i=0; i<10; i++)
	{
		val = 0;
		exp = test_value[i];
		exp_idx = test_idx[i];
		i8_exp = 1;
		i8_result = NVMQ_dequeueTableQ(&idx, &val);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(exp, val);
		TEST_ASSERT_EQUAL_INT(exp_idx, idx);
		TEST_ASSERT_EQUAL_INT(10-(i+1), table_q.count);
	}

	// q is empty but try dequeue : error
	i8_exp = 0;
	i8_result = NVMQ_dequeueTableQ(&idx, &val);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(0, table_q.count);

	// verify empty
	i8_exp = 1;
	i8_result = NVMQ_isEmptyTableQ(); // empty
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);

	// enqueue over PARAM_TABLE_SIZE : NOK
	i8_exp = 1;
	for(i=0; i<PARAM_TABLE_SIZE-1; i++)
	{
		i8_result = NVMQ_enqueueTableQ(test_idx[0], test_value[0]);
		TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
		TEST_ASSERT_EQUAL_INT(i+1, table_q.count);
	}
	i8_exp = 0;
	i8_result = NVMQ_enqueueTableQ(test_idx[0], test_value[0]);
	TEST_ASSERT_EQUAL_INT(i8_exp, i8_result);
	TEST_ASSERT_EQUAL_INT(PARAM_TABLE_SIZE-1, table_q.count);
}

#endif
