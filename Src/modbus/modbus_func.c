/*
 * modbus_func.c
 *
 *  Created on: 2019. 3. 22.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/
#include "includes.h"

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "proc_uart.h"
#include "modbus_func.h"
#include "table.h"
#include "modbus_queue.h"
#include "error.h"
#include "handler.h"


/* Private variables ---------------------------------------------------------*/

// CRC16 High-Order Byte Table
const unsigned char ModbusCRCHi[256] = {
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40, 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41,
 0x00, 0xc1, 0x81, 0x40, 0x01, 0xc0, 0x80, 0x41, 0x01, 0xc0, 0x80, 0x41, 0x00, 0xc1, 0x81, 0x40
};

// CRC16 Low-Order Byte Table
const unsigned char ModbusCRCLo[256] = {
 0x00, 0xc0, 0xc1, 0x01, 0xc3, 0x03, 0x02, 0xc2, 0xc6, 0x06, 0x07, 0xc7, 0x05, 0xc5, 0xc4, 0x04,
 0xcc, 0x0c, 0x0d, 0xcd, 0x0f, 0xcf, 0xce, 0x0e, 0x0a, 0xca, 0xcb, 0x0b, 0xc9, 0x09, 0x08, 0xc8,
 0xd8, 0x18, 0x19, 0xd9, 0x1b, 0xdb, 0xda, 0x1a, 0x1e, 0xde, 0xdf, 0x1f, 0xdd, 0x1d, 0x1c, 0xdc,
 0x14, 0xd4, 0xd5, 0x15, 0xd7, 0x17, 0x16, 0xd6, 0xd2, 0x12, 0x13, 0xd3, 0x11, 0xd1, 0xd0, 0x10,
 0xf0, 0x30, 0x31, 0xf1, 0x33, 0xf3, 0xf2, 0x32, 0x36, 0xf6, 0xf7, 0x37, 0xf5, 0x35, 0x34, 0xf4,
 0x3c, 0xfc, 0xfd, 0x3d, 0xff, 0x3f, 0x3e, 0xfe, 0xfa, 0x3a, 0x3b, 0xfb, 0x39, 0xf9, 0xf8, 0x38,
 0x28, 0xe8, 0xe9, 0x29, 0xeb, 0x2b, 0x2a, 0xea, 0xee, 0x2e, 0x2f, 0xef, 0x2d, 0xed, 0xec, 0x2c,
 0xe4, 0x24, 0x25, 0xe5, 0x27, 0xe7, 0xe6, 0x26, 0x22, 0xe2, 0xe3, 0x23, 0xe1, 0x21, 0x20, 0xe0,
 0xa0, 0x60, 0x61, 0xa1, 0x63, 0xa3, 0xa2, 0x62, 0x66, 0xa6, 0xa7, 0x67, 0xa5, 0x65, 0x64, 0xa4,
 0x6c, 0xac, 0xad, 0x6d, 0xaf, 0x6f, 0x6e, 0xae, 0xaa, 0x6a, 0x6b, 0xab, 0x69, 0xa9, 0xa8, 0x68,
 0x78, 0xb8, 0xb9, 0x79, 0xbb, 0x7b, 0x7a, 0xba, 0xbe, 0x7e, 0x7f, 0xbf, 0x7d, 0xbd, 0xbc, 0x7c,
 0xb4, 0x74, 0x75, 0xb5, 0x77, 0xb7, 0xb6, 0x76, 0x72, 0xb2, 0xb3, 0x73, 0xb1, 0x71, 0x70, 0xb0,
 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
 0x9c, 0x5c, 0x5d, 0x9d, 0x5f, 0x9f, 0x9e, 0x5e, 0x5a, 0x9a, 0x9b, 0x5b, 0x99, 0x59, 0x58, 0x98,
 0x88, 0x48, 0x49, 0x89, 0x4b, 0x8b, 0x8a, 0x4a, 0x4e, 0x8e, 0x8f, 0x4f, 0x8d, 0x4d, 0x4c, 0x8c,
 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

MODBUS_SLAVE_QUEUE modbusRx, modbusTx;

uint8_t	mb_slaveAddress = 1;
uint8_t reset_requested_f=0;

uint8_t param_init_requested_f=0;


MODBUS_addr_st mb_drive, mb_config, mb_protect, mb_ext_io;
MODBUS_addr_st mb_motor, mb_device, mb_err, mb_status;

extern void HDLR_setRunStopFlagModbus(int8_t flag);
extern void HDLR_setFactoryModeFlagModbus(int8_t flag);
#ifdef SUPPORT_PASSWORD
extern int table_isPasswordAddrModbus(uint16_t mb_addr);
#endif
extern int8_t table_setValueFactoryMode(PARAM_IDX_t idx, int32_t value);

/* Private function prototypes -----------------------------------------------*/


/********************************************************************************
* FUNCTION	 : CRC16
* DESCRIPTION: MODBus CRC16 »ý¼º
* ARGUMENTS	 : 	*pMsg : Modbus buf pointer(from address)
				Length: buf size - address+function+data
* RETURNS	 :
********************************************************************************/
uint16_t CRC16(uint8_t *pMsg, uint32_t Length)
{
	uint8_t CRCHi = 0xFF; // high CRC byte initialized
	uint8_t CRCLo = 0xFF; // low CRC byte initialized
	uint32_t uIndex;   // will index into CRC look up table
	uint16_t crc;

	while(Length--)
	{
 		uIndex = CRCHi ^ *pMsg++; // calculate the CRC
 		CRCHi = CRCLo ^ ModbusCRCHi[uIndex];
		CRCLo = ModbusCRCLo[uIndex];
 	}

 	crc = (uint16_t)CRCHi<<8|(uint16_t)CRCLo;
	return crc;
}

int MB_isCRC_OK(uint8_t *buf, uint32_t len)
{
	uint16_t recvCRC, caclCRC;

	recvCRC = (((uint16_t)buf[len-2] << 8) & 0xFF00) | ((uint16_t)buf[len-1] & 0x0FF);
	caclCRC = CRC16(buf, len-2);

	if(recvCRC == caclCRC) return 1;
	else return 0;
}

void MB_setSlaveAddress(uint8_t addr)
{
	mb_slaveAddress = addr;
}

/*
 * 	modbus <-> NFC address mapping
 *
 */
void MB_initAddrMap(void)
{
	int i, count;

	// driver map
	mb_drive.start = MB_DRIVER_START_ADDR;
	mb_drive.end = MB_DRIVER_END_ADDR;
	mb_drive.start_index = value_type;
	count = MB_DRIVER_END_ADDR - MB_DRIVER_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_drive.map[i].valid = 1;
		mb_drive.map[i].rd_only = 0;
		mb_drive.map[i].conv_index = mb_drive.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_drive.map[i].valid = 0;
		mb_drive.map[i].rd_only = 0;
		mb_drive.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	//printf("\r\n st_addr=%d, end_addr=%d", mb_drive.start, mb_drive.end);

	// config map
	mb_config.start = MB_CONFIG_START_ADDR;
	mb_config.end = MB_CONFIG_END_ADDR;
	mb_config.start_index = ctrl_in_type;
	count = MB_CONFIG_END_ADDR - MB_CONFIG_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_config.map[i].valid = 1;
		mb_config.map[i].rd_only = 0;
		mb_config.map[i].conv_index = mb_config.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_config.map[i].valid = 0;
		mb_config.map[i].rd_only = 0;
		mb_config.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	//printf("\r\n st_addr=%d, end_addr=%d", mb_config.start, mb_config.end);

	// protect map
	mb_protect.start = MB_PROTECT_START_ADDR;
	mb_protect.end = MB_PROTECT_END_ADDR;
	mb_protect.start_index = ovl_warn_limit_type;
	count = MB_PROTECT_END_ADDR - MB_PROTECT_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_protect.map[i].valid = 1;
		mb_protect.map[i].rd_only = 0;
		mb_protect.map[i].conv_index = mb_protect.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_protect.map[i].valid = 0;
		mb_protect.map[i].rd_only = 0;
		mb_protect.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	//printf("\r\n st_addr=%d, end_addr=%d", mb_protect.start, mb_protect.end);

	// ext_io map
	mb_ext_io.start = MB_EXT_IO_START_ADDR;
	mb_ext_io.end = MB_EXT_IO_END_ADDR;
	mb_ext_io.start_index = multi_Din_0_type;
	count = MB_EXT_IO_END_ADDR - MB_EXT_IO_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_ext_io.map[i].valid = 1;
		mb_ext_io.map[i].rd_only = 0;
		mb_ext_io.map[i].conv_index = mb_ext_io.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_ext_io.map[i].valid = 0;
		mb_ext_io.map[i].rd_only = 0;
		mb_ext_io.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	//printf("\r\n st_addr=%d, end_addr=%d", mb_ext_io.start, mb_ext_io.end);

	// motor parameter
	mb_motor.start = MB_MOTOR_START_ADDR;
	mb_motor.end = MB_MOTOR_END_ADDR;
	mb_motor.start_index = Rs_type;
	count = MB_MOTOR_END_ADDR - MB_MOTOR_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_motor.map[i].valid = 1;
		mb_motor.map[i].rd_only = 1; // read_only
		mb_motor.map[i].conv_index = mb_motor.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_motor.map[i].valid = 0;
		mb_motor.map[i].rd_only = 0;
		mb_motor.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	// device setting
	mb_device.start = MB_DEVICE_START_ADDR;
	mb_device.end = MB_DEVICE_END_ADDR;
	mb_device.start_index = model_type;
	count = MB_DEVICE_END_ADDR - MB_DEVICE_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_device.map[i].valid = 1;
		mb_device.map[i].rd_only = 1; // read_only
		mb_device.map[i].conv_index = mb_device.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_device.map[i].valid = 0;
		mb_device.map[i].rd_only = 0;
		mb_device.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	// error map
	mb_err.start = MB_ERROR_START_ADDR;
	mb_err.end = MB_ERROR_END_ADDR;
	mb_err.start_index = err_date_0_type;
	count = MB_ERROR_END_ADDR - MB_ERROR_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_err.map[i].valid = 1;
		mb_err.map[i].rd_only = 1; // read_only
		mb_err.map[i].conv_index = mb_err.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_err.map[i].valid = 0;
		mb_err.map[i].rd_only = 0;
		mb_err.map[i].conv_index = PARAM_TABLE_SIZE;
	}

	//printf("\r\n st_addr=%d, end_addr=%d", mb_err.start, mb_err.end);
	

	// status map
	mb_status.start = MB_STATUS_START_ADDR;
	mb_status.end = MB_STATUS_END_ADDR;
	mb_status.start_index = run_status1_type;
	count = MB_STATUS_END_ADDR - MB_STATUS_START_ADDR + 1;
	for(i=0; i<count; i++)
	{
		mb_status.map[i].valid = 1;
		mb_status.map[i].rd_only = 2; // read from variable, not from EEPROM
		mb_status.map[i].conv_index = mb_status.start_index + i;
	}
	// clear else
	for(i=count; i<MODBUS_ADDR_MAP_SIZE; i++)
	{
		mb_status.map[i].valid = 0;
		mb_status.map[i].rd_only = 0;
		mb_status.map[i].conv_index = PARAM_TABLE_SIZE;
	}
	
	//printf("\r\n st_addr=%d, end_addr=%d", mb_status.start, mb_status.end);
	
}



uint16_t MB_getActualAddress(MODBUS_addr_st *mb_addr, uint16_t addr, uint16_t count)
{
	uint16_t i, index;

	if(addr >= mb_addr->start && addr <= mb_addr->end)
	{
		if(count == 1) // single
		{
			index = addr - mb_addr->start;
			if(mb_addr->map[index].valid)
				return (index + mb_addr->start_index);
			else
			{
				//kprintf(PORT_DEBUG, " index=%d, valid=%d, mod_addr=%d \r\n", index, mb_addr->map[index].valid, mb_addr->map[index].mod_addr);
				return MODBUS_ADDR_MAP_ERR-1;
			}
		}
		else // multiple
		{
			// range check
			if(addr + count - 1 > mb_addr->end) return MODBUS_ADDR_MAP_ERR-2;

			// validity check
			index = addr - mb_addr->start;
			for(i=0; i<count; i++)
			{
				if(!mb_addr->map[index+i].valid) return MODBUS_ADDR_MAP_ERR-3;
			}
			return (index + mb_addr->start_index);
		}
	}

	//kprintf(PORT_DEBUG, " addr=%d, count=%d, st_addr=%d, end_addr=%d \r\n", addr, count, mb_addr->start, mb_addr->end);
	return MODBUS_ADDR_MAP_ERR; //not found
}

uint16_t MB_convModbusAddr(uint16_t addr, uint16_t count, int8_t *type)
{
	int8_t find_f = 0;
	uint16_t index=MODBUS_ADDR_MAP_ERR;
	MODBUS_addr_st *mb_addr;

	if(addr >= mb_drive.start && addr <= mb_drive.end) {mb_addr = &mb_drive; find_f = 1;}

	else if(addr >= mb_config.start && addr <= mb_config.end) {mb_addr = &mb_config; find_f = 2;}

	else if(addr >= mb_protect.start && addr <= mb_protect.end) {mb_addr = &mb_protect; find_f = 3;}

	else if(addr >= mb_ext_io.start && addr <= mb_ext_io.end) {mb_addr = &mb_ext_io; find_f = 4;}

	else if(addr >= mb_err.start && addr <= mb_err.end) {mb_addr = &mb_err; find_f = 5;}

	else if(addr >= mb_motor.start && addr <= mb_motor.end) {mb_addr = &mb_motor; find_f = 6;}

	else if(addr >= mb_device.start && addr <= mb_device.end) {mb_addr = &mb_device; find_f = 7;}

	else if(addr >= mb_status.start && addr <= mb_status.end) {mb_addr = &mb_status; find_f = 8;}

	else return MODBUS_ADDR_MAP_ERR;

	if(find_f)
	{
		index = MB_getActualAddress(mb_addr, addr, count);
		//kprintf(PORT_DEBUG, " addr=%d, count=%d, find=%d, st_addr=%d, end_addr=%d \r\n", addr, count, find_f, mb_addr->start, mb_addr->end);
	}

	*type = find_f;
	return index;
}

void MB_generateErrorResp(uint8_t func_code, uint8_t excep_code)
{
	memset(modbusTx.buf, 0, MODBUS_BUF_SIZE);

	modbusTx.buf[0] = mb_slaveAddress;
	modbusTx.buf[1] = func_code | 0x80;
	modbusTx.buf[2] = excep_code;
	modbusTx.wp = 3;

}

int MB_handleDummyRead(uint8_t func_code, uint16_t addr, uint16_t cnt)
{
	int result=MOD_EX_NO_ERR;
	int32_t value=1;

	switch(addr)
	{
	case MB_CTRL_CONN_CHECK_ADDR:
		modbusTx.wp = 0;
		modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
		modbusTx.buf[modbusTx.wp++] = func_code;
		modbusTx.buf[modbusTx.wp++] = cnt*2;
		modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

		result = MOD_EX_NO_ERR;
		kprintf(PORT_DEBUG, "addr=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);

		break;
	}

	return result;
}

int MB_handleReadRegister(uint8_t func_code, uint16_t addr, uint16_t cnt)
{
	int i, result=MOD_EX_NO_ERR;
	int8_t type=0; // flag for status data
	uint16_t index;
	int32_t value;

	/* return error type
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataADD
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */

	if(addr == MB_CTRL_CONN_CHECK_ADDR)
	{
		result = MB_handleDummyRead(func_code, addr, cnt);
		goto FC03_ERR;
	}

	// valid address range ?
	index = MB_convModbusAddr(addr, cnt, &type);
	if(index > MODBUS_ADDR_MAP_ERR-4) {result = MOD_EX_DataADD; goto FC03_ERR; }

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = func_code;
	modbusTx.buf[modbusTx.wp++] = cnt*2;

	if(cnt == 1)
	{
		value = table_getValue(index);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);
		kprintf(PORT_DEBUG, "s_read index=%d, value=%d, wp=%d \r\n", index, (uint16_t)value, modbusTx.wp);
	}
	else
	{
		for(i=0; i<cnt; i++)
		{
			value = table_getValue(index + i);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0x0000FF00) >> 8);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x000000FF);
		}
		//kprintf(PORT_DEBUG, "s_read index=%d, value=%d, wp=%d \r\n", index, (uint16_t)value, modbusTx.wp);
	}


FC03_ERR:
	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC03_RD_HREG, result);
	}

	return result;
}

int MB_handleFlagRegister(uint16_t addr, uint16_t value)
{
	int result=MOD_EX_NO_ERR;

	// only accept at Modbus control enable
	if(table_getCtrllIn() != CTRL_IN_Modbus) return MOD_EX_SLAVE_FAIL;

	switch(addr)
	{
	case MB_CTRL_RUN_STOP_ADDR:
		if(!ERR_isErrorState())
		{
			if(value == RUN_STOP_FLAG_RUN || value == RUN_STOP_FLAG_STOP)
			{
				HDLR_setRunStopFlagModbus((int8_t)value);
				modbusTx.wp = 0;
				modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
				modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

				result = MOD_EX_NO_ERR;
				kprintf(PORT_DEBUG, "set run_stop=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);
			}
			else
				result = MOD_EX_DataVAL;
		}
		else
			result = MOD_EX_SLAVE_FAIL;

		break;

	case MB_CTRL_RESET_ADDR:
		if(value == 1) // only accept at Modbus control enable
		{
			if(table_isMotorStop()) // only on motor not running
			{
				reset_requested_f = 1;

				modbusTx.wp = 0;
				modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
				modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

				result = MOD_EX_NO_ERR;
				kprintf(PORT_DEBUG, "set run_stop=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);
			}
			else
				result = MOD_EX_SLAVE_FAIL; // motor is running
		}
		else
			result = MOD_EX_DataVAL;
		break;

#ifdef SUPPORT_PARAMETER_BACKUP
	case MB_CTRL_BACKUP_FLAG_ADDR:
		if(value == MB_BACKUP_SAVE || value == MB_BACKUP_RESTORE)
		{
			HDLR_setBackupFlagModbus(value);
			modbusTx.wp = 0;
			modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
			modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
			modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

			result = MOD_EX_NO_ERR;
			kprintf(PORT_DEBUG, "set backup_addr=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);
		}
		else
			result = MOD_EX_DataVAL;

		break;
#endif
	case MB_CTRL_FACTORY_MODE_ADDR: // enable/disable factory mode

		if(value == 0 || value == 1)
		{
			if(table_isMotorStop()) // only on motor not running
			{
				HDLR_setFactoryModeFlagModbus(value);
				modbusTx.wp = 0;
				modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
				modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

				result = MOD_EX_NO_ERR;
				kprintf(PORT_DEBUG, "set fct_mode addr=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);
			}
			else
				result = MOD_EX_SLAVE_FAIL; // motor is running
		}
		else
			result = MOD_EX_DataVAL;

		break;

#ifdef SUPPORT_INIT_PARAM
	case MB_CTRL_NVM_INIT_ADDR:
		if(value == 1)
		{
			if(table_isMotorStop()) // only on motor not running
			{
				param_init_requested_f = 1; // set flag

				modbusTx.wp = 0;
				modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
				modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
				modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);

				result = MOD_EX_NO_ERR;
				kprintf(PORT_DEBUG, "set NVM init addr=%d, value=%d, wp=%d \r\n", addr, (uint16_t)value, modbusTx.wp);
			}
			else
				result = MOD_EX_SLAVE_FAIL; // motor is running
		}
		else
			result = MOD_EX_DataVAL;

		break;
#endif

	default:
		result = MOD_EX_SLAVE_FAIL;
		break;
	}

	return result;
}

int MB_handleFactoryModeWriteRegister(uint16_t addr, uint16_t value)
{
	int ret, result=MOD_EX_NO_ERR;
	uint16_t index;

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

	ret = table_setValueFactoryMode(index, (int32_t)value);
	if(ret == 1)
	{
		modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);
		kprintf(PORT_DEBUG, "s_write index=%d, value=%d, wp=%d \r\n", index, (uint16_t)value, modbusTx.wp);
		result = MOD_EX_NO_ERR;
	}
	else
		result = MOD_EX_SLAVE_FAIL;

	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC06_WR_REG, result);
	}

	return result;
}

int MB_handleWriteSingleRegister(uint16_t addr, uint16_t value)
{
	int ret, result=MOD_EX_NO_ERR;
	int8_t type=0; // flag for status data
	uint16_t index;

	/* return error type
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataVAL
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */

	if(addr == MB_CTRL_RUN_STOP_ADDR
		|| addr == MB_CTRL_RESET_ADDR		// device reset
		|| addr == MB_CTRL_FACTORY_MODE_ADDR  // go into factory mode
#ifdef SUPPORT_PARAMETER_BACKUP
		|| addr == MB_CTRL_BACKUP_FLAG_ADDR
#endif
#ifdef SUPPORT_INIT_PARAM
		|| addr == MB_CTRL_NVM_INIT_ADDR	// initialize NVM
#endif
		)
	{
		result = MB_handleFlagRegister(addr, value);
		goto FC06_ERR;
	}

	// valid address range ?
	index = MB_convModbusAddr(addr, 1, &type);
	if(index > MODBUS_ADDR_MAP_ERR-4) {result = MOD_EX_DataADD; goto FC06_ERR; }

	if(type == 8) {result = MOD_EX_SLAVE_FAIL; goto FC06_ERR; } // error : status read only

	if(table_getRW(index) == 0) {result = MOD_EX_SLAVE_FAIL; goto FC06_ERR; } // error : read only

#ifdef SUPPORT_PASSWORD
	if(table_isLocked()) {result = MOD_EX_SLAVE_FAIL; goto FC06_ERR; } // error : parameter locked
#endif

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

	ret = table_runFunc(index, (int32_t)value, REQ_FROM_MODBUS);
	if(ret == 1)
	{
		modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);
		kprintf(PORT_DEBUG, "s_write index=%d, value=%d, wp=%d \r\n", index, (uint16_t)value, modbusTx.wp);
		result = MOD_EX_NO_ERR;
	}
	else
		result = MOD_EX_SLAVE_FAIL;

FC06_ERR:
	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC06_WR_REG, result);
	}

	return result;
}

#ifdef SUPPORT_PASSWORD
int lock_pass_ok=0;
int MB_handleLockReq(uint16_t addr, uint16_t count, uint16_t *value)
{
	int ret, result=MOD_EX_NO_ERR;
	uint16_t password, lock;
	int32_t stored_password=0;

	password = value[0];
	lock = value[1];

	stored_password = table_getValue(password_type);
	if(stored_password != (int32_t)password) return MOD_EX_SLAVE_FAIL;
	else lock_pass_ok = 1; // password OK

	ret = table_runFunc(modify_lock_type, (int32_t)lock, REQ_FROM_MODBUS);
	if(ret == 0) return MOD_EX_SLAVE_FAIL;

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC16_WRM_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

	modbusTx.buf[modbusTx.wp++] = (uint8_t)((count&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(count&0x00FF);

	return result;
}
#endif

int MB_handleWriteMultiRegister(uint16_t addr, uint16_t count, uint16_t *value)
{
	int i, ret, result=MOD_EX_NO_ERR;
	int8_t type=0; // flag for status data
	uint16_t index;

	/* return error types
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataVAL
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */

#ifdef SUPPORT_PASSWORD
	if(table_isPasswordAddrModbus(addr) && count == 2)
	{
		result = MB_handleLockReq(addr, count, value);
	}
	else
		result = MOD_EX_SLAVE_FAIL;

	goto FC16_ERR; // multi write is not supported
#endif

	// count is over 123 or not matched with byte size
	if(count == MODBUS_COUNT_ERR) {result = MOD_EX_DataVAL; goto FC16_ERR; }

	// valid address range ?
	index = MB_convModbusAddr(addr, count, &type);
	if(index > MODBUS_ADDR_MAP_ERR-4) {result = MOD_EX_DataADD; goto FC16_ERR; }

	if(type == 8) {result = MOD_EX_SLAVE_FAIL; goto FC16_ERR; } // error : status read only

	if(table_getRW(index) == 0) {result = MOD_EX_SLAVE_FAIL; goto FC16_ERR; } // error : read only

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC16_WRM_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

	for(i=0; i<count; i++)
	{
		ret = table_runFunc(index+i, (int32_t)value[i], REQ_FROM_MODBUS);
		kprintf(PORT_DEBUG, "m_write index=%d, value=%d, ret=%d \r\n", index+i, (uint16_t)value[i], ret);
		if(ret == 0)
		{
			result = MOD_EX_SLAVE_FAIL;
			goto FC16_ERR;
		}
	}
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((count&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(count&0x00FF);

FC16_ERR:
	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC16_WRM_REG, result);
	}
	//kprintf(PORT_DEBUG, " result=%d \r\n", result);
	return result;
}

int MB_processModbusPacket(void) // error or response packet
{
	uint8_t i, func_code, byte_cnt;
	uint16_t calcCRC=0;
	uint16_t reg_addr=0, reg_cnt=0, value=0;
	uint16_t multi_val[123];
	int ret_code=MOD_EX_NO_ERR;

	func_code = modbusRx.buf[1];
	reg_addr = (uint16_t)(((uint16_t)modbusRx.buf[2] << 8) | modbusRx.buf[3]);
	switch(func_code)
	{
	case MOD_FC03_RD_HREG:
	case MOD_FC04_RD_IREG:
		reg_cnt = (uint16_t)(((uint16_t)modbusRx.buf[4] << 8) | modbusRx.buf[5]);
		ret_code = MB_handleReadRegister(func_code, reg_addr, reg_cnt);

		break;

	case MOD_FC06_WR_REG:
		value = (uint16_t)(((uint16_t)modbusRx.buf[4] << 8) | modbusRx.buf[5]);
		if(HDLR_isFactoryModeEnabled()) // factory mode
		{
			ret_code = MB_handleFactoryModeWriteRegister(reg_addr, value);
		}
		else
		{
			ret_code = MB_handleWriteSingleRegister(reg_addr, value);
			kprintf(PORT_DEBUG, " MOD_FC06_WR_REG : addr=%d, ret_code=%d \r\n", reg_addr, ret_code);
		}
		break;

	case MOD_FC16_WRM_REG:
		reg_cnt = (uint16_t)(((uint16_t)modbusRx.buf[4] << 8) | modbusRx.buf[5]);
		byte_cnt = modbusRx.buf[6];
		if(reg_cnt > 123 || byte_cnt != (uint8_t)reg_cnt*2)
			reg_cnt = MODBUS_COUNT_ERR;
		else
		{
			for(i=0; i<reg_cnt; i++)
			{
				multi_val[i] = (uint16_t)(((uint16_t)modbusRx.buf[7+i*2] << 8) | modbusRx.buf[8+i*2]);
			}
		}
		//kprintf(PORT_DEBUG, " addr=%d, reg_cnt=%d, byte_cnt=%d \r\n", reg_addr, reg_cnt, byte_cnt);
		ret_code = MB_handleWriteMultiRegister(reg_addr, reg_cnt, multi_val);
		break;

	default: // not supported func_code
		ret_code = MOD_EX_FUNC; // illegal function code
		MB_generateErrorResp(func_code, ret_code);
		break;
	}
	calcCRC = CRC16(modbusTx.buf, modbusTx.wp);
	modbusTx.buf[modbusTx.wp++] = (int8_t)(calcCRC >> 8) &0x00FF;
	modbusTx.buf[modbusTx.wp++] = (int8_t)(calcCRC & 0x00FF);
	//MB_writeRespPacket(modbusTx.wp+1);

	return ret_code;
}

int8_t MB_handlePacket(void)
{
	int result=0;

	// check modbus queue
	if(MBQ_isEmptyReqQ()) return 1;

	// get packet from queue
	modbusRx.wp = MBQ_getReqQ(modbusRx.buf);
	if(modbusRx.wp == 0) { kprintf(PORT_DEBUG, "no data in req_q\r\n"); return 0; }

//	kprintf(PORT_DEBUG, "main RX: 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",
//		modbusRx.buf[0], modbusRx.buf[1], modbusRx.buf[2], modbusRx.buf[3], modbusRx.buf[4]);

	// generate response or error frame
	result = MB_processModbusPacket();

//	kprintf(PORT_DEBUG, "main TX: 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",
//		modbusTx.buf[0], modbusTx.buf[1], modbusTx.buf[2], modbusTx.buf[3], modbusTx.buf[4]);

	// wait response Q empty
	while(MBQ_isEmptyRespQ()==0) osDelay(1);

	//send response to modbus task
	MBQ_putRespQ(modbusTx.wp, modbusTx.buf);



	return 1;
}

