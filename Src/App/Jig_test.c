/*
 * Jig_test.c
 *
 *  Created on: 2019. 8. 16.
 *      Author: hrjung
 */

#include "includes.h"
#include "proc_uart.h"

#ifdef SUPPORT_PRODUCTION_TEST_MODE

#include "table.h"

#include "dsp_comm.h"
#include "nvm_queue.h"
#include "drv_nvm.h"
#include "nvm_queue.h"
#include "error.h"

#include "drv_gpio.h"


#define JIG_TEST_CNT		7

#define JIG_ADC_VALUE_LOW		100
#define JIG_ADC_VALUE_HIGH		120

// JIG_TEST_NFC_STATE sub state
enum
{
	JIG_TEST_SUB_NFC_INIT,
	JIG_TEST_SUB_NFC_WRITE,
	JIG_TEST_SUB_NFC_READ,
	JIG_TEST_SUB_NFC_END,
};

// JIG_TEST_DI_DO_STATE sub state
enum
{
	JIG_TEST_SUB_DIO_INIT,
	JIG_TEST_SUB_WRITE_DO_0,	// set DO 0
	JIG_TEST_SUB_READ_DI_0,
	JIG_TEST_SUB_WRITE_DO_1,	// set DO 1
	JIG_TEST_SUB_READ_DI_1,
	JIG_TEST_SUB_DIO_END,
};

// JIG_TEST_AIN_STATE sub state
enum
{
	JIG_TEST_SUB_AIN_INIT,
	JIG_TEST_SUB_READ_AIN,
	JIG_TEST_SUB_AIN_END,
};

// JIG_TEST_485_STATE sub state
enum
{
	JIG_TEST_SUB_485_INIT,
	JIG_TEST_SUB_485_INPORGRESS,
	JIG_TEST_SUB_485_END,
};

// JIG_TEST_SPI_STATE sub state
enum
{
	JIG_TEST_SUB_SPI_INIT,
	JIG_TEST_SUB_SPI_READ,
	JIG_TEST_SUB_SPI_TEST,
	JIG_TEST_SUB_SPI_END,
};

// JIG_TEST_MTD_DTM_STATE sub state
enum
{
	JIG_TEST_SUB_MTD_INIT,
	JIG_TEST_SUB_DTM0_TEST, // read DTM pin
	JIG_TEST_SUB_DTM1_TEST,
	JIG_TEST_SUB_DTM2_TEST,
	JIG_TEST_SUB_DTM3_TEST,
	JIG_TEST_SUB_MTD0_TEST,  // set MTD pin
	JIG_TEST_SUB_MTD1_TEST,
	JIG_TEST_SUB_MTD_END,
};

// JIG_TEST_WAIT_DSP_STATE sub state
enum
{
	JIG_TEST_SUB_DSP_INIT,
	JIG_TEST_SUB_DSP_WAIT,
	JIG_TEST_SUB_DSP_GET_RESULT,
	JIG_TEST_SUB_DSP_END,
};

// jig test main state
enum
{
	JIG_TEST_INIT,
	JIG_TEST_NFC_STATE,		// EEPROM read/write test
	JIG_TEST_DI_DO_STATE, 	// write DO,  read DI
	JIG_TEST_AIN_STATE,		// read pre-determined voltage value via ADC
	JIG_TEST_485_STATE,		// 485 communication loopback test
	JIG_TEST_SPI_STATE,		// MCU -> DSP spi packet loopback test
	JIG_TEST_MTD_DTM_STATE,	//
	JIG_TEST_WAIT_DSP_STATE, // DSP jig test start, get result
	JIG_TEST_ERR_STATE,
	JIG_TEST_END,
};

uint8_t jig_test_enabled_f = 0;
uint8_t jig_err=0;

uint8_t jig_test_result[JIG_TEST_CNT]={0};


extern uint8_t mdout_value[];
extern uint8_t mdin_value[];

extern uint8_t OPT_is485TestStarted(void);
extern uint8_t OPT_is485TestEnded(void);

extern void OPT_start485Test(void);

extern uint16_t EXT_AI_readADC(void);

extern uint32_t COM_getReadValue(void);
extern uint8_t UTIL_readDTMpin(void);
extern void test_setTableValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern uint8_t NVM_clearInit(void);

int JIG_isTestEnabled(void)
{
	// read JIG Test enabled PIN only on boot time

	return jig_test_enabled_f;
}

int JIG_init_state(void)
{
	int j_state = JIG_TEST_NFC_STATE;




	return j_state;
}

int JIG_isValidAdc(uint16_t value)
{
	if(value < JIG_ADC_VALUE_LOW) return 0;

	if(value > JIG_ADC_VALUE_HIGH) return 0;

	return 1;
}

int JIG_Nfc_state(void)
{
	int j_state = JIG_TEST_NFC_STATE;
	static int sub_state = JIG_TEST_SUB_NFC_INIT;
	uint8_t status;
	uint16_t test_addr[4] = {0, 512, 1024, 1536};
	int32_t value = 0xAAAA5555, r_val[4]={0};

	switch(sub_state)
	{
	case JIG_TEST_SUB_NFC_INIT:
		jig_err = 0;
		sub_state = JIG_TEST_SUB_NFC_WRITE;
		break;

	case JIG_TEST_SUB_NFC_WRITE:
		osDelay(100);

		status = NVM_write(test_addr[0], value);
		if(status == 0) jig_err = 1;

		status = NVM_write(test_addr[1], value);
		if(status == 0) jig_err = 1;

		status = NVM_write(test_addr[2], value);
		if(status == 0) jig_err = 1;

		status = NVM_write(test_addr[3], value);
		if(status == 0) jig_err = 1;

		sub_state = JIG_TEST_SUB_NFC_READ;
		break;

	case JIG_TEST_SUB_NFC_READ:
		osDelay(100);

		status = NVM_read(test_addr[0], &r_val[0]);
		if(status == 0) jig_err = 2;
		else if(r_val[0] != value) jig_err = 3;

		status = NVM_read(test_addr[1], &r_val[1]);
		if(status == 0) jig_err = 2;
		else if(r_val[0] != value) jig_err = 3;

		status = NVM_read(test_addr[2], &r_val[2]);
		if(status == 0) jig_err = 2;
		else if(r_val[0] != value) jig_err = 3;

		status = NVM_read(test_addr[3], &r_val[3]);
		if(status == 0) jig_err = 2;
		else if(r_val[0] != value) jig_err = 3;

		sub_state = JIG_TEST_SUB_NFC_END;
		break;

	case JIG_TEST_SUB_NFC_END:

		if(jig_err) jig_test_result[JIG_TEST_NFC_STATE-1] = 1; // set error

		j_state = JIG_TEST_DI_DO_STATE; // next test state
		break;
	}

	return j_state;
}

int JIG_Dio_state(void)
{
	int j_state = JIG_TEST_DI_DO_STATE;
	static int sub_state = JIG_TEST_SUB_DIO_INIT;

	switch(sub_state)
	{
	case JIG_TEST_SUB_DIO_INIT:
		jig_err = 0;
		test_setTableValue(ctrl_in_type, CTRL_IN_Digital, REQ_FROM_TEST);
		sub_state = JIG_TEST_SUB_WRITE_DO_0;
		break;

	case JIG_TEST_SUB_WRITE_DO_0:
		osDelay(100);
		mdout_value[0] = 0;
		mdout_value[1] = 0;
		sub_state = JIG_TEST_SUB_READ_DI_0;
		break;

	case JIG_TEST_SUB_READ_DI_0:
		osDelay(300); // wait stable

		if(mdin_value[0] != 0) jig_err = 1;
		if(mdin_value[1] != 0) jig_err = 1;
		if(mdin_value[2] != 0) jig_err = 1;

		sub_state = JIG_TEST_SUB_WRITE_DO_1;
		break;

	case JIG_TEST_SUB_WRITE_DO_1:
		osDelay(100);
		mdout_value[0] = 1;
		mdout_value[1] = 1;
		sub_state = JIG_TEST_SUB_READ_DI_1;
		break;

	case JIG_TEST_SUB_READ_DI_1:
		osDelay(300); // wait stable

		if(mdin_value[0] != 1) jig_err = 1;
		if(mdin_value[1] != 1) jig_err = 1;
		if(mdin_value[2] != 1) jig_err = 1;

		sub_state = JIG_TEST_SUB_DIO_END;
		break;

	case JIG_TEST_SUB_DIO_END:

		if(jig_err) jig_test_result[JIG_TEST_DI_DO_STATE-1] = 1; // set error

		j_state = JIG_TEST_AIN_STATE; // next test state
		break;
	}

	return j_state;
}

int JIG_Ain_state(void)
{
	int j_state = JIG_TEST_AIN_STATE;
	static int sub_state = JIG_TEST_SUB_AIN_INIT;
	static int r_cnt=0;
	uint16_t value;

	switch(sub_state)
	{
	case JIG_TEST_SUB_AIN_INIT:
		jig_err = 0;
		test_setTableValue(ctrl_in_type, CTRL_IN_Analog_V, REQ_FROM_TEST);
		sub_state = JIG_TEST_SUB_READ_AIN;
		break;

	case JIG_TEST_SUB_READ_AIN:
		osDelay(300);

		value = EXT_AI_readADC();
		if(!JIG_isValidAdc(value)) jig_err=1;

		if(r_cnt++ > 3)  // check 3 times
			sub_state = JIG_TEST_SUB_AIN_END;
		break;

	case JIG_TEST_SUB_AIN_END:

		if(jig_err) jig_test_result[JIG_TEST_AIN_STATE-1] = 1; // set error

		j_state = JIG_TEST_485_STATE; // next test state
		break;
	}

	return j_state;
}

int JIG_SPI_state(void)
{
	int j_state = JIG_TEST_SPI_STATE;
	static int sub_state = JIG_TEST_SUB_SPI_INIT;
	uint16_t dsp_idx = value_dsp;
	uint16_t test_cmd[3] = {0,0,0};
	int32_t r_value=0;
	int8_t status;

	switch(sub_state)
	{
	case JIG_TEST_SUB_SPI_INIT:
		jig_err = 0;
		sub_state = JIG_TEST_SUB_SPI_READ;
		break;

	case JIG_TEST_SUB_SPI_READ:
		osDelay(300);
		// parameter parameter command
		test_cmd[0] = dsp_idx; // read command freq = 200
		status = COMM_sendMessage(SPICMD_PARAM_R, test_cmd);
		if(status)
		{
			// verify read data
			r_value = COM_getReadValue();
			if(r_value != (int32_t)200) jig_err = 1;
		}
		else
			jig_err = 1;

		sub_state = JIG_TEST_SUB_SPI_TEST;
		break;

	case JIG_TEST_SUB_SPI_TEST:
		osDelay(300);
		// send test command : SPI_TEST_CMD_SPI_TEST, get ACK
		status = COMM_sendTestCmd(SPI_TEST_CMD_SPI_TEST);
		// if ack, pass
		if(status == 0) jig_err = 1;

		sub_state = JIG_TEST_SUB_SPI_END;
		break;

	case JIG_TEST_SUB_SPI_END:
		if(jig_err) jig_test_result[JIG_TEST_SPI_STATE-1] = 1; // set error

		j_state = JIG_TEST_MTD_DTM_STATE; // next test state
		break;
	}

	return j_state;
}

int JIG_MTD_state(void)
{
	int j_state = JIG_TEST_MTD_DTM_STATE;
	static int sub_state = JIG_TEST_SUB_MTD_INIT;
	uint8_t dtm_val=0;
	int8_t status;

	switch(sub_state)
	{
	case JIG_TEST_SUB_MTD_INIT:
		jig_err = 0;
		sub_state = JIG_TEST_SUB_DTM0_TEST;
		break;

	case JIG_TEST_SUB_DTM0_TEST:
		osDelay(300);
		// ask set DTM pin 0
		status = COMM_sendTestCmd(SPI_TEST_CMD_DTM0_READ);
		// if ack, pass
		if(status == 0) jig_err = 1;

		// verify read data
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 0) jig_err = 1;

		sub_state = JIG_TEST_SUB_DTM1_TEST;
		break;

	case JIG_TEST_SUB_DTM1_TEST:
		osDelay(300);
		// ask set DTM pin 0
		status = COMM_sendTestCmd(SPI_TEST_CMD_DTM1_READ);
		// if ack, pass
		if(status == 0) jig_err = 1;

		// verify read data
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 1) jig_err = 1;

		sub_state = JIG_TEST_SUB_DTM2_TEST;
		break;

	case JIG_TEST_SUB_DTM2_TEST:
		osDelay(300);
		// ask set DTM pin 0
		status = COMM_sendTestCmd(SPI_TEST_CMD_DTM2_READ);
		// if ack, pass
		if(status == 0) jig_err = 1;

		// verify read data
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 2) jig_err = 1;

		sub_state = JIG_TEST_SUB_DTM3_TEST;
		break;

	case JIG_TEST_SUB_DTM3_TEST:
		osDelay(300);
		// ask set DTM pin 0
		status = COMM_sendTestCmd(SPI_TEST_CMD_DTM3_READ);
		// if ack, pass
		if(status == 0) jig_err = 1;

		// verify read data
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 3) jig_err = 1;

		sub_state = JIG_TEST_SUB_MTD0_TEST;
		break;

	case JIG_TEST_SUB_MTD0_TEST:
		osDelay(300);

		// set MTD pin 0
		UTIL_setMTDpin(0);
		// send test command
		status = COMM_sendTestCmd(SPI_TEST_CMD_MTD_READ);
		if(status == 0) jig_err = 1;

		// verify read data as 00
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 0) jig_err = 1;

		sub_state = JIG_TEST_SUB_MTD1_TEST;
		break;

	case JIG_TEST_SUB_MTD1_TEST:
		osDelay(300);

		// set MTD pin 1
		UTIL_setMTDpin(1);
		// send test command
		status = COMM_sendTestCmd(SPI_TEST_CMD_MTD_READ);
		if(status == 0) jig_err = 1;

		// verify read data as 01
		dtm_val = UTIL_readDTMpin();
		if(dtm_val != 1) jig_err = 1;

		sub_state = JIG_TEST_SUB_MTD_END;
		break;

	case JIG_TEST_SUB_MTD_END:
		if(jig_err) jig_test_result[JIG_TEST_MTD_DTM_STATE-1] = 1; // set error

		j_state = JIG_TEST_WAIT_DSP_STATE; // next test state
		break;
	}

	return j_state;
}

int JIG_DSP_state(void)
{
	int j_state = JIG_TEST_WAIT_DSP_STATE;
	static int sub_state = JIG_TEST_SUB_DSP_INIT;

	switch(sub_state)
	{
	case JIG_TEST_SUB_DSP_INIT: // set DSP to start JIG test

		sub_state = JIG_TEST_SUB_DSP_WAIT;
		break;

	case JIG_TEST_SUB_DSP_WAIT: // wait test finished

		sub_state = JIG_TEST_SUB_DSP_GET_RESULT;
		break;

	case JIG_TEST_SUB_DSP_GET_RESULT: // if finished, request result

		sub_state = JIG_TEST_SUB_DSP_END;
		break;

	case JIG_TEST_SUB_DSP_END: // check result and end

		if(jig_err) jig_test_result[JIG_TEST_WAIT_DSP_STATE-1] = 1; // set error

		j_state = JIG_TEST_END;
		break;
	}

	return j_state;
}

void JIG_test_state(void)
{
	static int jig_state = JIG_TEST_INIT;


	switch(jig_state)
	{
	case JIG_TEST_INIT:
		JIG_init_state();
		jig_state = JIG_TEST_NFC_STATE;
		break;

	case JIG_TEST_NFC_STATE:

		jig_state = JIG_Nfc_state();
		break;

	case JIG_TEST_DI_DO_STATE:

		jig_state = JIG_Dio_state();;
		break;

	case JIG_TEST_AIN_STATE:

		jig_state = JIG_Ain_state();;
		break;

	case JIG_TEST_485_STATE:
		if(OPT_is485TestStarted() == 0)	OPT_start485Test();


		if(OPT_is485TestEnded())
			jig_state = JIG_TEST_SPI_STATE;
		break;

	case JIG_TEST_SPI_STATE:


		jig_state = JIG_TEST_MTD_DTM_STATE;
		break;

	case JIG_TEST_MTD_DTM_STATE:

		jig_state = JIG_TEST_WAIT_DSP_STATE;
		break;

	case JIG_TEST_WAIT_DSP_STATE: // wait DSP test finished
		jig_state = JIG_DSP_state();
		break;

	case JIG_TEST_END:
		NVM_clearInit(); // need re-init NVM
		break;

	case JIG_TEST_ERR_STATE:

		break;
	}
}

#endif //SUPPORT_PRODUCTION_TEST_MODE

