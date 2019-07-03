/*
 * test_modbus.c
 *
 *  Created on: 2019. 3. 21.
 *      Author: hrjung
 */

#ifdef SUPPORT_UNIT_TEST

#include <stdio.h>
#include <string.h>
#include <memory.h>

#include "unity.h"

#include "proc_uart.h"
#include "modbus_func.h"
#include "table.h"
/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/* modbus format
 *
 *
 *
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint8_t	mb_slaveAddress;
extern MODBUS_SLAVE_QUEUE modbusRx, modbusTx;

extern MODBUS_addr_st mb_drive, mb_config, mb_protect, mb_ext_io;
extern MODBUS_addr_st mb_motor, mb_device, mb_err;
/*******************************************************************************
 * EXTERNS
 */

extern uint16_t MB_convModbusAddr(uint16_t addr, uint16_t count);
extern int MB_isValidRecvPacket(void);
extern int MB_processModbusPacket(void);
extern int MB_handleReadRegister(uint8_t func_code, uint16_t addr, uint16_t cnt);
extern int MB_handleWriteSingleRegister(uint16_t addr, uint16_t value);
/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *
 *


 */

void test_modbusBasic(void)
{
	int result=0;
	int exp=0;

	// normal request : read holding register
	uint8_t read_holding_reg[] = {0x1, 0x3, 0x0, 0x0, 0x0, 0xA, 0xC5, 0xCD}; // normal register
	exp = 1;
	modbusRx.wp = 8;
	memcpy(modbusRx.buf, read_holding_reg, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// normal request : read input register
	uint8_t read_input_reg[] = {0x1, 0x4, 0x0, 0x0, 0x0, 0xA, 0x70, 0x0D}; // normal input
	memset(modbusRx.buf, 0, MODBUS_BUF_SIZE);
	exp = 1;
	modbusRx.wp = 8;
	memcpy(modbusRx.buf, read_input_reg, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// normal request : write single register
	uint8_t write_input_reg[] = {0x1, 0x6, 0x0, 0x14, 0x0, 0x0, 0xC9, 0xCE}; // normal write single register
	memset(modbusRx.buf, 0, MODBUS_BUF_SIZE);
	exp = 1;
	modbusRx.wp = 8;
	memcpy(modbusRx.buf, write_input_reg, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// normal write multiple register
	uint8_t write_multi_reg[] = {0x1, 0x10, 0x0, 0x14, 0x0, 0x2, 0x4, 0x0, 0x0, 0x0, 0x0, 0xF3, 0x50}; // multi write single register
	memset(modbusRx.buf, 0, MODBUS_BUF_SIZE);
	exp = 1;
	modbusRx.wp = 13;
	memcpy(modbusRx.buf, write_multi_reg, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// slave address error
	uint8_t slave_addr_test[] = {0x2, 0x4, 0x0, 0x0, 0x0, 0xA, 0x70, 0x0D}; // wrong slave address
	memset(modbusRx.buf, 0, MODBUS_BUF_SIZE);
	exp = 0;
	modbusRx.wp = 8;
	memcpy(modbusRx.buf, slave_addr_test, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// CRC error
	uint8_t crc_err_test[] = {0x1, 0x4, 0x0, 0x0, 0x0, 0xA, 0x71, 0x0D}; // crc error
	memset(modbusRx.buf, 0, MODBUS_BUF_SIZE);
	exp = 0;
	modbusRx.wp = 8;
	memcpy(modbusRx.buf, crc_err_test, (int)modbusRx.wp);
	result = MB_isValidRecvPacket();
	TEST_ASSERT_EQUAL_INT(exp, result);
}


void test_modbusAddress(void)
{
	int result=0;
	int exp=0;
	uint16_t test_addr, count=1;

	// control parameter range, normal,
	test_addr = 40113; // direction command
	exp = (uint16_t)direction_control_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_DRIVER_START_ADDR; //
	exp = (uint16_t)value_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_DRIVER_END_ADDR; //
	exp = acc_base_set_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_DRIVER_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_drive.map[10].valid = 0;
	test_addr = 40110;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_drive.map[10].valid = 1; // restore

	// valid count
	count = 5;
	test_addr = MB_DRIVER_END_ADDR-4;
	exp = jmp_high0_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 5;
	test_addr = MB_DRIVER_END_ADDR-3;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_drive.map[10].valid = 0;
	count = 10;
	test_addr = 40101;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_drive.map[10].valid = 1; // restore


	// config parameter range, normal
	count = 1;
	test_addr = 40203;
	exp = brake_type_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_CONFIG_START_ADDR; //
	exp = ctrl_in_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_CONFIG_END_ADDR; //
	exp = dci_brake_rate_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_CONFIG_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_config.map[5].valid = 0;
	test_addr = 40205;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_config.map[5].valid = 1; // restore

	// valid count
	count = 5;
	test_addr = MB_CONFIG_END_ADDR-4;
	exp = brake_freq_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 5;
	test_addr = MB_CONFIG_END_ADDR-3;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_config.map[5].valid = 0;
	count = 5;
	test_addr = 40202;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_config.map[5].valid = 1; //restore


	//protection parameter, normal
	count = 1;
	test_addr = 40282; //
	exp = ovl_enable_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_PROTECT_START_ADDR; //
	exp = ovl_warn_limit_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_PROTECT_END_ADDR; //
	exp = fan_onoff_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_PROTECT_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_protect.map[3].valid = 0;
	test_addr = 40283;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_protect.map[3].valid = 1; // restore

	// valid count
	count = 5;
	test_addr = MB_PROTECT_END_ADDR-4;
	exp = ovl_trip_limit_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 5;
	test_addr = MB_PROTECT_END_ADDR-3;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_protect.map[3].valid = 0;
	count = 5;
	test_addr = 40281;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_protect.map[3].valid = 1; //restore


	// extern IO parameter, normal
	count = 1;
	test_addr = 40305; //
	exp = multi_Dout_1_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_EXT_IO_START_ADDR; //
	exp = multi_Din_0_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_EXT_IO_END_ADDR; //
	exp = baudrate_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_EXT_IO_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_ext_io.map[9].valid = 0;
	test_addr = 40309;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_ext_io.map[9].valid = 1; // restore

	// valid count
	count = 5;
	test_addr = MB_EXT_IO_END_ADDR-4;
	exp = v_in_max_freq_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 5;
	test_addr = MB_EXT_IO_END_ADDR-3;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_ext_io.map[9].valid = 0;
	count = 5;
	test_addr = 40306;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_ext_io.map[9].valid = 1; //restore


	// motor parameter, normal
	count = 1;
	test_addr = 40023; //
	exp = noload_current_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_MOTOR_START_ADDR; //
	exp = Rs_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_MOTOR_END_ADDR; //
	exp = rated_freq_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_MOTOR_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_motor.map[4].valid = 0;
	test_addr = 40024;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_motor.map[4].valid = 1; // restore

	// valid count
	count = 3;
	test_addr = MB_MOTOR_END_ADDR-2;
	exp = poles_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 5;
	test_addr = MB_MOTOR_END_ADDR-3;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_motor.map[4].valid = 0;
	count = 5;
	test_addr = 40023;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_motor.map[4].valid = 1; //restore


	// device setting parameter, normal
	count = 1;
	test_addr = 40063; //
	exp = motor_on_count_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//start,
	test_addr = MB_DEVICE_START_ADDR; //
	exp = model_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//end,
	test_addr = MB_DEVICE_END_ADDR; //
	exp = operating_hour_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//out of range
	test_addr = MB_DEVICE_END_ADDR+1;
	exp = MODBUS_ADDR_MAP_ERR;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// valid flag test
	mb_device.map[4].valid = 0;
	test_addr = 40064;
	exp = MODBUS_ADDR_MAP_ERR-1;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_device.map[4].valid = 1; // restore

	// valid count
	count = 3;
	test_addr = MB_DEVICE_END_ADDR-2;
	exp = motor_on_count_type;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid count
	count = 3;
	test_addr = MB_DEVICE_END_ADDR-1;
	exp = MODBUS_ADDR_MAP_ERR-2;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// invalid flag on multiple
	mb_device.map[3].valid = 0;
	count = 3;
	test_addr = 40061;
	exp = MODBUS_ADDR_MAP_ERR-3;
	result = MB_convModbusAddr(test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, result);
	mb_device.map[3].valid = 1; //restore



	// error parameter, normal

	//start,

	//end,

	//out of range


	// status parameter, normal

	//start,

	//end,

	//out of range
}


void test_modbusGetValue(void)
{
	uint16_t index, test_addr, count=1;
	int32_t result, exp;
	uint8_t buf[10];

	// control parameter range, normal,
	test_addr = 40100; // command frequency
	exp = 200;
	index = MB_convModbusAddr(test_addr, count);
	result = table_database_getValue(index);
	TEST_ASSERT_EQUAL_INT32(exp, result);
	buf[0] = (uint8_t)((result&0xFF00) >> 8);
	buf[1] = (uint8_t)(result&0x00FF);
	TEST_ASSERT_EQUAL_INT8(buf[0], 0);
	TEST_ASSERT_EQUAL_INT8(buf[1], 0xc8); // 200


	// config parameter range, normal,
	test_addr = 40312; // mb address
	exp = 1;
	index = MB_convModbusAddr(test_addr, count);
	result = table_database_getValue(index);
	TEST_ASSERT_EQUAL_INT32(exp, result);
	buf[0] = (uint8_t)((result&0xFF00) >> 8);
	buf[1] = (uint8_t)(result&0x00FF);
	TEST_ASSERT_EQUAL_INT8(buf[0], 0);
	TEST_ASSERT_EQUAL_INT8(buf[1], 0x1); // 1

}

void test_readCommand(uint8_t func_code)
{
	uint16_t test_addr, count=1;
	int exp, index;

	test_addr = 40100; // command frequency
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0xc8); // 200

	test_addr = 40112; // deceleration time
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x64); // 100

	test_addr = 40123; // direction control setting
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0); // 0


	test_addr = MB_CONFIG_START_ADDR;
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0); // 0

	test_addr = 40206; // DC injection brake block time
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0xA); // 10

	test_addr = MB_CONFIG_END_ADDR; // DC injection brake rate
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 1);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0xF4); // 500


	test_addr = MB_PROTECT_START_ADDR; // overload warning level
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x96); // 150

	test_addr = 40284; // overload trip duration
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x1E); // 30

	test_addr = MB_PROTECT_END_ADDR; // fan control
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x0); // 0


	test_addr = MB_EXT_IO_START_ADDR; // multi digital input 0
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x0); // 0

	test_addr = 40309; // voltage input maximum frequency
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x7);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0xD0); // 2000

	test_addr = MB_EXT_IO_END_ADDR; // baudrate
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x2); // 2


	test_addr = 40400; // err_date_0
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[3], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[4], 0x0); // 0
}

void test_modbusReadMultiCommand(uint8_t func_code)
{
	uint16_t test_addr, count=3;
	int exp, index;

	// multiple read test, count=3
	// check only 3rd value
	test_addr = 40100; // command frequency + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0xc8); // 200 : Frequency level 1

	test_addr = 40112; // deceleration time + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0x0); // 0 : enable jump frequency 0

	test_addr = 40121; // high limit of jump frequency 1 + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0x0); // 0 : direction control setting

	test_addr = 40200; // command source setting
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0); // 0 : energy save setting

	test_addr = 40206; // DC injection brake block time + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0x1);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0xF4); // 500 : DC injection brake rate


	test_addr = 40280; // overload warning level
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0x1); // 1 : enable overload trip

	test_addr = 40284; // overload trip duration + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0xA); // 10 : regen band


	test_addr = 40300; // multi digital input 0 + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0x0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0x0); // 0 : multi digital input 2

	test_addr = 40311; // Analog output rate + 3
	exp = MOD_EX_NO_ERR;
	index = MB_handleReadRegister(func_code, test_addr, count);
	TEST_ASSERT_EQUAL_INT(exp, index);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[7], 0);
	TEST_ASSERT_EQUAL_INT8(modbusTx.buf[8], 0x2); // baudrate
}

void test_modbusFC03(void)
{
	test_readCommand(MOD_FC03_RD_HREG);
}

void test_modbusFC03_multi(void)
{
	test_modbusReadMultiCommand(MOD_FC03_RD_HREG);
}

void test_modbusFC04(void)
{
	test_readCommand(MOD_FC04_RD_IREG);
}

void test_modbusFC04_multi(void)
{
	test_modbusReadMultiCommand(MOD_FC04_RD_IREG);
}

void test_modbusFC06(void)
{
	uint16_t test_addr, w_val, r_val;
	int exp, result;

	test_addr = MB_DRIVER_START_ADDR;
	exp = MOD_EX_NO_ERR;
	w_val = 300;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(value_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 200); //restore

	test_addr = 40112;  // decel time
	exp = MOD_EX_NO_ERR;
	w_val = 300;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(decel_time_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 100); //restore

	test_addr = MB_DRIVER_END_ADDR;	// accel time base
	exp = MOD_EX_NO_ERR;
	w_val = 1;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(acc_base_set_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 0); //restore


	test_addr = MB_CONFIG_START_ADDR;
	exp = MOD_EX_NO_ERR;
	w_val = 2;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(ctrl_in_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 0); //restore

	test_addr = 40205;		// dci brake start freq
	exp = MOD_EX_NO_ERR;
	w_val = 20;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(dci_brake_freq_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 30); //restore

	test_addr = MB_CONFIG_END_ADDR;
	exp = MOD_EX_NO_ERR;
	w_val = 700;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(dci_brake_rate_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 500); //restore



	test_addr = MB_PROTECT_START_ADDR;
	exp = MOD_EX_NO_ERR;
	w_val = 120;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(ovl_warn_limit_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 150); //restore

	test_addr = 40284;		// ovl trip duration
	exp = MOD_EX_NO_ERR;
	w_val = 20;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(ovl_trip_duration_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 30); //restore

	test_addr = MB_PROTECT_END_ADDR;
	exp = MOD_EX_NO_ERR;
	w_val = 1;
	result = MB_handleWriteSingleRegister(test_addr, w_val);
	r_val = table_database_getValue(fan_onoff_type);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_UINT16(w_val, r_val);
	result = MB_handleWriteSingleRegister(test_addr, 0); //restore
}

#endif
