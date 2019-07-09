/*
 * test_dsp_comm.c
 *
 *  Created on: 2019. 6. 12.
 *      Author: hrjung
 */
#include "includes.h"

#ifdef SUPPORT_UNIT_TEST

#include <stdio.h>
#include <string.h>
#include <memory.h>

#include "unity.h"
#include "table.h"

#include "dsp_comm.h"
#include "drv_dsp_spi.h"

// test for SPI_sendDSP return value
int8_t send_status;
int8_t recv_status;

uint16_t testResp[20];

extern int32_t table_data[];

extern uint16_t sendMsg[];
extern uint16_t recvMsg[];
extern int16_t read_idx;

extern int16_t COMM_getRecvLength(COMM_CMD_t cmd);
extern int8_t COMM_generateMessage(COMM_CMD_t cmd, const uint16_t* data);
extern int32_t COMM_parseValue(int16_t dsp_index, uint16_t *data, int8_t *err);
extern int8_t COMM_parseMessage(void);
extern int8_t COMM_sendCommand(COMM_CMD_t cmd, const uint16_t* data);

/*
 * 		test item : COMM_getRecvLength
 *
 *		1. check COMM_generateMessage return correct value for the command
 * 		2. test generateMessage correctly make send packet, check error for wrong command
 *
 */
void test_getRecvLength(void)
{
	int16_t recvlen, exp_len;

	exp_len = 18;
	recvlen = COMM_getRecvLength(SPICMD_REQ_ST);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	exp_len = 7;
	recvlen = COMM_getRecvLength(SPICMD_CTRL_RUN);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_CTRL_STOP);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_CTRL_DIR_F);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_CTRL_DIR_R);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_PARAM_W);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	exp_len = 9;
	recvlen = COMM_getRecvLength(SPICMD_PARAM_R);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	exp_len = 0;
	recvlen = COMM_getRecvLength(SPICMD_RESP_ACK);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_RESP_PARAM);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_RESP_ST);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(SPICMD_RESP_ERR);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);

	recvlen = COMM_getRecvLength(0x1111);
	TEST_ASSERT_EQUAL_INT(exp_len, recvlen);
}

/*
 * 		test item : COMM_generateMessage
 *
 * 		2. test generateMessage correctly make send packet,
 * 		3. check error for wrong command
 *
 */
void test_generateMessage(void)
{
	uint16_t dummy[]= {1}; // not used
	uint16_t param_w[3], param_r[1];
	int32_t value;
	float value_f;
	uint8_t exp_len;
	int8_t exp_result=COMM_SUCCESS, result;


	// command packet
	exp_result=COMM_SUCCESS;
	exp_len = 6;
	result = COMM_generateMessage(SPICMD_CTRL_RUN, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_CTRL_RUN, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	result = COMM_generateMessage(SPICMD_CTRL_STOP, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_CTRL_STOP, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	result = COMM_generateMessage(SPICMD_CTRL_DIR_F, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_CTRL_DIR_F, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	result = COMM_generateMessage(SPICMD_CTRL_DIR_R, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_CTRL_DIR_R, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	result = COMM_generateMessage(SPICMD_REQ_ST, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_REQ_ST, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	result = COMM_generateMessage(SPICMD_REQ_ERR, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_REQ_ERR, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);

	// parameter_w packet, int
	exp_len = 9;
	value = 180;
	param_w[0] = ovl_trip_limit_dsp;
	memcpy(&param_w[1], &value, sizeof(int32_t));
	result = COMM_generateMessage(SPICMD_PARAM_W, param_w);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_PARAM_W, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);
	TEST_ASSERT_EQUAL_INT(param_w[0], sendMsg[5]); // index
	TEST_ASSERT_EQUAL_INT(param_w[1], sendMsg[6]); // value
	TEST_ASSERT_EQUAL_INT(param_w[2], sendMsg[7]);

	// parameter_w packet, float
	value_f = 200.0;
	param_w[0] = freq_max_dsp;
	memcpy(&param_w[1], &value_f, sizeof(float));
	result = COMM_generateMessage(SPICMD_PARAM_W, param_w);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_PARAM_W, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);
	TEST_ASSERT_EQUAL_INT(param_w[0], sendMsg[5]); // index
	TEST_ASSERT_EQUAL_INT(param_w[1], sendMsg[6]); // value
	TEST_ASSERT_EQUAL_INT(param_w[2], sendMsg[7]);

	// parameter read
	exp_len = 7;
	param_w[0] = value_dsp;
	result = COMM_generateMessage(SPICMD_PARAM_R, param_r);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_PARAM_R, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);
	TEST_ASSERT_EQUAL_INT(param_r[0], sendMsg[5]); // index

	param_w[0] = fan_onoff_dsp;
	result = COMM_generateMessage(SPICMD_PARAM_R, param_r);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(SPICMD_PARAM_R, sendMsg[4]);
	TEST_ASSERT_EQUAL_INT(exp_len, sendMsg[2]);
	TEST_ASSERT_EQUAL_INT(param_r[0], sendMsg[5]); // index

	// wrong command
	exp_result = COMM_FAILED;
	result = COMM_generateMessage(SPICMD_RESP_ACK, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_generateMessage(SPICMD_RESP_ST, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_generateMessage(SPICMD_RESP_ERR, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_generateMessage(SPICMD_RESP_PARAM, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_generateMessage(0x1201, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
}


/*
 * 		test item : COMM_parseValue
 *
 * 		1. test valid dsp_index 0, last, return correct value
 * 		2. index for float value, return correct value
 * 		3. index for int value, return correct value
 * 		4. invalid dsp_index, set err = 1
 *
 *
 */
void test_parseValue(void)
{
	int16_t dsp_index;
	int32_t exp_value, value;
	int8_t err_flag, exp_err;
	uint16_t rcvData[2];
	float value_f;

	// valid index
	dsp_index = value_dsp;
	exp_err = 0;
	value_f = 20.0;
	exp_value = (int32_t)(value_f*10.0);
	memcpy(&rcvData[0], &value_f, sizeof(int32_t));
	value = COMM_parseValue(dsp_index, &rcvData[0], &err_flag);
	TEST_ASSERT_EQUAL_INT(exp_err, err_flag);
	TEST_ASSERT_EQUAL_INT(exp_value, value);

	// float value
	dsp_index = accel_time_dsp;
	exp_err = 0;
	value_f = 10.0;
	exp_value = (int32_t)(value_f*10.0);
	memcpy(&rcvData[0], &value_f, sizeof(int32_t));
	value = COMM_parseValue(dsp_index, &rcvData[0], &err_flag);
	TEST_ASSERT_EQUAL_INT(exp_err, err_flag);
	TEST_ASSERT_EQUAL_INT(exp_value, value);

	// int value
	dsp_index = ovl_warn_limit_dsp;
	exp_err = 0;
	exp_value = 150;
	memcpy(&rcvData[0], &exp_value, sizeof(int32_t));
	value = COMM_parseValue(dsp_index, &rcvData[0], &err_flag);
	TEST_ASSERT_EQUAL_INT(exp_err, err_flag);
	TEST_ASSERT_EQUAL_INT(exp_value, value);

	dsp_index = fan_onoff_dsp;
	exp_err = 0;
	exp_value = 1;
	memcpy(&rcvData[0], &exp_value, sizeof(int32_t));
	value = COMM_parseValue(dsp_index, &rcvData[0], &err_flag);
	TEST_ASSERT_EQUAL_INT(exp_err, err_flag);
	TEST_ASSERT_EQUAL_INT(exp_value, value);

	// index error
	dsp_index = DSP_PARAM_SIZE;
	exp_err = 1;
	memcpy(&rcvData[0], &exp_value, sizeof(int32_t));
	value = COMM_parseValue(dsp_index, &rcvData[0], &err_flag);
	TEST_ASSERT_EQUAL_INT(exp_err, err_flag);
}

/*
 * 		test item : COMM_parseMessage
 *
 * 		1. test RESP_ACK, check ACK, NAK
 * 		2. test RESP_ST, check value correctly set, use tabe_getStatusValue()
 * 		3. test RESP_ERR, check value correctly set, verify table_data[err0]
 * 		4. test RESP_PARAM
 * 		5. test other command, cause error
 *

 *
 */
void test_parseMessage(void)
{
	int i;
	int16_t exp_value;
	int16_t dsp_index;
	int8_t result, exp_result;
	uint16_t rcvMsg_ACK[6] = {0xAAAA, 0x5555, 0, 0, SPICMD_RESP_ACK, SPI_ACK};
	uint16_t rcvMsg_ST[16] = {0xAAAA, 0x5555, 0, 0, SPICMD_RESP_ST, 0,0,0,0,0, 0,0,0,0,0,0};
	uint16_t rcvMsg_ERR[10] = {0xAAAA, 0x5555, 0, 0, SPICMD_RESP_ERR, 0,0,0,0,0};
	uint16_t rcvMsg_PARAM[8] = {0xAAAA, 0x5555, 0, 0, SPICMD_RESP_PARAM, 0,0,0};

	//========= RESP_ACK, ACK
	for(i=0; i<6; i++) recvMsg[i] = rcvMsg_ACK[i];
	exp_result = COMM_SUCCESS;
	exp_value = SPI_ACK;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_value, recvMsg[5]);

	//========= RESP_ACK, NAK
	for(i=0; i<6; i++) recvMsg[i] = rcvMsg_ACK[i];
	recvMsg[5] = SPI_NAK;
	exp_result = COMM_SUCCESS;
	exp_value = SPI_NAK;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_value, recvMsg[5]);

	//========= RESP_ST
	int32_t exp_run_st, exp_dir_st, exp_ovload_st, exp_brake_st, exp_i_rms, exp_dc_volt, exp_freq, exp_ipm_tmp, exp_mtr_temp;
	int32_t test_mtr_temp=1;
	float test_i_rms=1.4, test_freq=34.5, test_dc_volt=532.4, test_ipm_temp=72.3;
#ifdef SUPPORT_NFC_OLD
	int32_t exp_status1, exp_status2;
#endif

	for(i=0; i<16; i++) recvMsg[i] = rcvMsg_ST[i];
	recvMsg[5] = 1; //status1=1
	recvMsg[6] = (uint16_t)0x100; //status2
	memcpy(&recvMsg[5+2], &test_i_rms, sizeof(float));
	memcpy(&recvMsg[5+4], &test_freq, sizeof(float));
	memcpy(&recvMsg[5+6], &test_dc_volt, sizeof(float));
	memcpy(&recvMsg[5+8], &test_ipm_temp, sizeof(float));
	memcpy(&recvMsg[5+10], &test_mtr_temp, sizeof(float));
	exp_result = COMM_SUCCESS;
#ifdef SUPPORT_NFC_OLD
	exp_status1 = 1;
	exp_status2 = 0x100;
#else
	exp_run_st = (int32_t)1;
	exp_dir_st = (int32_t)0;
	exp_ovload_st = (int32_t)0;
	exp_brake_st = (int32_t)1;
#endif
	exp_i_rms = (int32_t)(test_i_rms*10.0 + 0.05);
	exp_freq = (int32_t)(test_freq*10.0 + 0.05);
	exp_dc_volt = (int32_t)(test_dc_volt*10.0);
	exp_ipm_tmp = (int32_t)(test_ipm_temp*10.0);
	exp_mtr_temp = (int32_t)1;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
#ifdef SUPPORT_NFC_OLD
	TEST_ASSERT_EQUAL_INT(exp_status1, table_getStatusValue(run_status1_type));
	TEST_ASSERT_EQUAL_INT(exp_status2, table_getStatusValue(run_status2_type));
#else
	TEST_ASSERT_EQUAL_INT(exp_run_st, table_getStatusValue(run_status_type));
	TEST_ASSERT_EQUAL_INT(exp_dir_st, table_getStatusValue(dir_status_type));
	TEST_ASSERT_EQUAL_INT(exp_ovload_st, table_getStatusValue(overload_alarm_type));
	TEST_ASSERT_EQUAL_INT(exp_brake_st, table_getStatusValue(shaftbrake_status_type));
#endif
	TEST_ASSERT_EQUAL_INT(exp_freq, table_getStatusValue(run_freq_type));
	TEST_ASSERT_EQUAL_INT(exp_i_rms, table_getStatusValue(I_rms_type));
	TEST_ASSERT_EQUAL_INT(exp_dc_volt, table_getStatusValue(dc_voltage_type));
	TEST_ASSERT_EQUAL_INT(exp_ipm_tmp, table_getStatusValue(ipm_temperature_type));
	TEST_ASSERT_EQUAL_INT(exp_mtr_temp, table_getStatusValue(mtr_temperature_type));

	test_mtr_temp=2;
	test_i_rms=3.8, test_freq=83.2, test_dc_volt=659.4, test_ipm_temp=88.7;
	for(i=0; i<16; i++) recvMsg[i] = rcvMsg_ST[i];
	recvMsg[5] = 0x100; //status1=1
	recvMsg[6] = 1; //status2
	memcpy(&recvMsg[5+2], &test_i_rms, sizeof(float));
	memcpy(&recvMsg[5+4], &test_freq, sizeof(float));
	memcpy(&recvMsg[5+6], &test_dc_volt, sizeof(float));
	memcpy(&recvMsg[5+8], &test_ipm_temp, sizeof(float));
	memcpy(&recvMsg[5+10], &test_mtr_temp, sizeof(float));
	exp_result = COMM_SUCCESS;
#ifdef SUPPORT_NFC_OLD
	exp_status1 = 0x100;
	exp_status2 = 1;
#else
	exp_run_st = (int32_t)0;
	exp_dir_st = (int32_t)1;
	exp_ovload_st = (int32_t)1;
	exp_brake_st = (int32_t)0;
#endif
	exp_i_rms = (int32_t)(test_i_rms*10.0 + 0.05);
	exp_freq = (int32_t)(test_freq*10.0 + 0.05);
	exp_dc_volt = (int32_t)(test_dc_volt*10.0);
	exp_ipm_tmp = (int32_t)(test_ipm_temp*10.0);
	exp_mtr_temp = (int32_t)2;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
#ifdef SUPPORT_NFC_OLD
	TEST_ASSERT_EQUAL_INT(exp_status1, table_getStatusValue(run_status1_type));
	TEST_ASSERT_EQUAL_INT(exp_status2, table_getStatusValue(run_status2_type));
#else
	TEST_ASSERT_EQUAL_INT(exp_run_st, table_getStatusValue(run_status_type));
	TEST_ASSERT_EQUAL_INT(exp_dir_st, table_getStatusValue(dir_status_type));
	TEST_ASSERT_EQUAL_INT(exp_ovload_st, table_getStatusValue(overload_alarm_type));
	TEST_ASSERT_EQUAL_INT(exp_brake_st, table_getStatusValue(shaftbrake_status_type));
#endif
	TEST_ASSERT_EQUAL_INT(exp_freq, table_getStatusValue(run_freq_type));
	TEST_ASSERT_EQUAL_INT(exp_i_rms, table_getStatusValue(I_rms_type));
	TEST_ASSERT_EQUAL_INT(exp_dc_volt, table_getStatusValue(dc_voltage_type));
	TEST_ASSERT_EQUAL_INT(exp_ipm_tmp, table_getStatusValue(ipm_temperature_type));
	TEST_ASSERT_EQUAL_INT(exp_mtr_temp, table_getStatusValue(mtr_temperature_type));

	//========= RESP_ERR
	int16_t err_code=4, exp_err_code, run_status=2, exp_run_status;
	int32_t exp_current;
	float test_current;

	test_current = 3.3;
	test_freq = 48.7;
	for(i=0; i<10; i++) recvMsg[i] = rcvMsg_ERR[i];
	recvMsg[5] = err_code; //status1=1
	recvMsg[6] = run_status; //status2
	memcpy(&recvMsg[5+2], &test_current, sizeof(float));
	memcpy(&recvMsg[5+4], &test_freq, sizeof(float));
	exp_result = COMM_SUCCESS;
	exp_err_code = (int32_t)err_code;
	exp_run_status = (int32_t)run_status;
	exp_current = (int32_t)(test_current*10.0 + 0.05);
	exp_freq = (int32_t)(test_freq*10.0 + 0.05);
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_err_code, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_run_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_current, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	err_code=34, run_status=1;
	test_current = 0.7, test_freq = 81.1;
	for(i=0; i<10; i++) recvMsg[i] = rcvMsg_ERR[i];
	recvMsg[5] = err_code; //status1=1
	recvMsg[6] = run_status; //status2
	memcpy(&recvMsg[5+2], &test_current, sizeof(float));
	memcpy(&recvMsg[5+4], &test_freq, sizeof(float));
	exp_result = COMM_SUCCESS;
	exp_err_code = (int32_t)err_code;
	exp_run_status = (int32_t)run_status;
	exp_current = (int32_t)(test_current*10.0 + 0.05);
	exp_freq = (int32_t)(test_freq*10.0 + 0.05);
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(exp_err_code, table_data[err_code_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_run_status, table_data[err_status_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_current, table_data[err_current_0_type]);
	TEST_ASSERT_EQUAL_INT(exp_freq, table_data[err_freq_0_type]);

	//========= RESP_PARAM : valid dsp_index
	dsp_index = value_dsp;
	for(i=0; i<8; i++) recvMsg[i] = rcvMsg_PARAM[i];
	recvMsg[5] = dsp_index;
	recvMsg[6] = 0, recvMsg[7] = 0; // no need to set
	exp_result = COMM_SUCCESS;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(dsp_index, read_idx);

	dsp_index = fan_onoff_dsp;
	for(i=0; i<8; i++) recvMsg[i] = rcvMsg_PARAM[i];
	recvMsg[5] = dsp_index;
	recvMsg[6] = 0, recvMsg[7] = 0; // no need to set
	exp_result = COMM_SUCCESS;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
	TEST_ASSERT_EQUAL_INT(dsp_index, read_idx);

	//========= invalid dsp_index
	dsp_index = none_dsp;
	for(i=0; i<8; i++) recvMsg[i] = rcvMsg_PARAM[i];
	recvMsg[5] = dsp_index;
	recvMsg[6] = 0, recvMsg[7] = 0; // no need to set
	exp_result = COMM_FAILED;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = DSP_PARAM_SIZE;
	for(i=0; i<8; i++) recvMsg[i] = rcvMsg_PARAM[i];
	recvMsg[5] = dsp_index;
	recvMsg[6] = 0, recvMsg[7] = 0; // no need to set
	exp_result = COMM_FAILED;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	//========= invalid cmd
	for(i=0; i<8; i++) recvMsg[i] = rcvMsg_PARAM[i];
	recvMsg[4] = SPICMD_REQ_ST;
	exp_result = COMM_FAILED;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	recvMsg[4] = SPICMD_PARAM_W;
	exp_result = COMM_FAILED;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	recvMsg[4] = 0x1011;
	exp_result = COMM_FAILED;
	result = COMM_parseMessage();
	TEST_ASSERT_EQUAL_INT(exp_result, result);
}


/*
 * 		test item : COMM_sendCommand
 *
 * 		1. test cmd, check ACK, NAK case
 * 		2. test REQ_ST, check value correctly set, use tabe_getStatusValue()
 * 		3. test REQ_ERR, check value correctly set, verify table_data[err0]
 * 		4. test REQ_PARAM
 * 		5. test other command, cause error
 *

 *
 */
void test_sendCommand(void)
{
	int i;
	int16_t dsp_index;
	int8_t result, exp_result;

	uint16_t dummy[] = {0,0,0};
	uint16_t data_param[] = {0,0,0};
	int32_t value;
	float value_f;

	uint16_t rcvMsg_ACK[6] = {0xAAAA, 0x5555, 7, 0, SPICMD_RESP_ACK, SPI_ACK};
	uint16_t rcvMsg_ST[16] = {0xAAAA, 0x5555, 18, 0, SPICMD_RESP_ST, 0,0,0,0,0, 0,0,0,0,0,0};
	uint16_t rcvMsg_ERR[10] = {0xAAAA, 0x5555, 12, 0, SPICMD_RESP_ERR, 0,0,0,0,0};
	uint16_t rcvMsg_PARAM[8] = {0xAAAA, 0x5555, 9, 0, SPICMD_RESP_PARAM, 0,0,0};


	send_status = COMM_SUCCESS;
	recv_status = COMM_SUCCESS;
	//========= command test : response ACK
	for(i=0; i<6; i++) testResp[i] = rcvMsg_ACK[i];
	exp_result = COMM_SUCCESS;
	result = COMM_sendCommand(SPICMD_CTRL_RUN, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_STOP, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_F, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_R, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = value_dsp;
	value_f = 20.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = jmp_low0_dsp;
	value_f = 1.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = fan_onoff_dsp;
	value = 1;
	memcpy(&data_param[1], &value, sizeof(int32_t));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	//========= command test : response NAK
	testResp[5] = SPI_NAK;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_CTRL_RUN, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_STOP, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_F, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_R, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = value_dsp;
	value_f = 20.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = jmp_low0_dsp;
	value_f = 1.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = fan_onoff_dsp;
	value = 1;
	memcpy(&data_param[1], &value, sizeof(int32_t));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= command test : SPI send error
	send_status = COMM_FAILED;
	recv_status = COMM_SUCCESS;

	testResp[5] = SPI_ACK;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_CTRL_RUN, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_STOP, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_F, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_R, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = value_dsp;
	value_f = 20.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = jmp_low0_dsp;
	value_f = 1.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = fan_onoff_dsp;
	value = 1;
	memcpy(&data_param[1], &value, sizeof(int32_t));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= command test : SPI read error
	send_status = COMM_SUCCESS;
	recv_status = COMM_FAILED;

	testResp[5] = SPI_ACK;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_CTRL_RUN, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_STOP, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_F, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	result = COMM_sendCommand(SPICMD_CTRL_DIR_R, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = value_dsp;
	value_f = 20.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = jmp_low0_dsp;
	value_f = 1.0;
	memcpy(&data_param[1], &value_f, sizeof(float));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	data_param[0] = fan_onoff_dsp;
	value = 1;
	memcpy(&data_param[1], &value, sizeof(int32_t));
	result = COMM_sendCommand(SPICMD_PARAM_W, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_status test : normal
	send_status = COMM_SUCCESS;
	recv_status = COMM_SUCCESS;
	for(i=0; i<6; i++) testResp[i] = rcvMsg_ST[i];
	exp_result = COMM_SUCCESS;
	result = COMM_sendCommand(SPICMD_REQ_ST, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_status test : SPI send error
	send_status = COMM_FAILED;
	recv_status = COMM_SUCCESS;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_REQ_ST, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_status test : SPI receive error
	send_status = COMM_SUCCESS;
	recv_status = COMM_FAILED;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_REQ_ST, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	//========= req_error test : normal
	send_status = COMM_SUCCESS;
	recv_status = COMM_SUCCESS;
	for(i=0; i<6; i++) testResp[i] = rcvMsg_ST[i];
	exp_result = COMM_SUCCESS;
	result = COMM_sendCommand(SPICMD_REQ_ERR, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_error test : SPI send error
	send_status = COMM_FAILED;
	recv_status = COMM_SUCCESS;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_REQ_ERR, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_error test : SPI receive error
	send_status = COMM_SUCCESS;
	recv_status = COMM_FAILED;
	exp_result = COMM_FAILED;
	result = COMM_sendCommand(SPICMD_REQ_ERR, dummy);
	TEST_ASSERT_EQUAL_INT(exp_result, result);


	//========= req_param test : normal
	send_status = COMM_SUCCESS;
	recv_status = COMM_SUCCESS;
	for(i=0; i<6; i++) testResp[i] = rcvMsg_PARAM[i];
	exp_result = COMM_SUCCESS;

	dsp_index = value_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = brake_freq_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = fan_onoff_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_param test : SPI send error
	send_status = COMM_FAILED;
	recv_status = COMM_SUCCESS;
	exp_result = COMM_FAILED;

	dsp_index = value_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = brake_freq_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = fan_onoff_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	//========= req_param test : SPI receive error
	send_status = COMM_SUCCESS;
	recv_status = COMM_FAILED;
	exp_result = COMM_FAILED;

	dsp_index = value_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = brake_freq_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);

	dsp_index = fan_onoff_dsp;
	data_param[0] = dsp_index;
	result = COMM_sendCommand(SPICMD_PARAM_R, data_param);
	TEST_ASSERT_EQUAL_INT(exp_result, result);
}

#endif
