/*
 * modbus_func.c
 *
 *  Created on: 2019. 3. 22.
 *      Author: hrjung
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "proc_uart.h"
#include "modbus_func.h"
#include "table.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>


// in case of baudrate > 19200, inter-frame delay is 1.75ms
// baudrate < 19200, need to calculate as 3.5 character
#define MODBUS_INTER_FRAME_DELAY		35

// RS485 RTS 1:TX, 0:RX
#define RS485_TX_ENABLE() {	HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_SET);}
#define RS485_TX_DISABLE() { HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_RESET);}
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

uint16_t mb_timeout = 0;
uint16_t mb_downcounter = 0;

uint8_t mb_start_flag = 0;
uint8_t mb_frame_received = 0;
uint16_t mb_err_code = 0;

uint32_t mb_baudrate[] = {2400, 4800, 9600, 19200, 38400, 115200};

MODBUS_addr_st mb_drive, mb_config, mb_protect, mb_ext_io;
MODBUS_addr_st mb_motor, mb_device, mb_err;

/* Global variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim7;
extern osSemaphoreId rs485SemaphoreIdHandle;


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

	recvCRC = (((uint16_t)buf[len-2] << 8) & 0xFF00) | ((uint16_t)modbusRx.buf[len-1] & 0x0FF);
	caclCRC = CRC16(buf, len-2);

	if(recvCRC == caclCRC) return 1;
	else return 0;
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
		mb_motor.map[i].rd_only = 1;
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
		mb_device.map[i].rd_only = 1;
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
		mb_err.map[i].rd_only = 1;
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
}


/*
 * 	timer related function
 */

void MB_initTimer(uint16_t timeout_value)
{
	mb_timeout = timeout_value;
}

void MB_enableTimer(void)
{
	mb_downcounter = mb_timeout;
	HAL_TIM_Base_Start_IT(&htim7);
}

void MB_disableTimer(void)
{
	HAL_TIM_Base_Stop_IT(&htim7);
}

void MB_UART_init(uint32_t baudrate)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = baudrate;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

void MB_init(void)
{
	int i;
	uint16_t b_index=2;
	uint32_t baudrate;

//	while(getIsEEPROMInit() != 1) osDelay(10);
//	mb_slaveAddress = table_database_getValue(mb_address_type);
//	b_index = (uint16_t)table_database_getValue(baudrate_type);

	MB_UART_init(mb_baudrate[b_index]);
	MB_initTimer(MODBUS_INTER_FRAME_DELAY); // 1.75ms for 3.5 char

	printf("MB init s_addr=%d, baud=%d \r\n", mb_slaveAddress, mb_baudrate[b_index]);

	modbusRx.wp = 0;
	modbusTx.wp = 0;
	for(i=0; i<MODBUS_BUF_SIZE; i++)
	{
		modbusRx.buf[i] = 0;
		modbusTx.buf[i] = 0;
	}

	MB_initAddrMap();

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // rs485

	// enable RX
	RS485_TX_DISABLE();
	//HAL_GPIO_WritePin(Modbus_RTS_GPIO_Port, Modbus_RTS_Pin, GPIO_PIN_RESET); //485 RX enable
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
				//kprintf(PORT_DEBUG, "\r\n index=%d, valid=%d, mod_addr=%d", index, mb_addr->map[index].valid, mb_addr->map[index].mod_addr);
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

	kprintf(PORT_DEBUG, "\r\n addr=%d, count=%d, st_addr=%d, end_addr=%d", addr, count, mb_addr->start, mb_addr->end);
	return MODBUS_ADDR_MAP_ERR; //not found
}

uint16_t MB_convModbusAddr(uint16_t addr, uint16_t count)
{
	int find_f = 0;
	uint16_t index=MODBUS_ADDR_MAP_ERR;
	MODBUS_addr_st *mb_addr;

	if(addr >= mb_drive.start && addr <= mb_drive.end) {mb_addr = &mb_drive; find_f = 1;}

	else if(addr >= mb_config.start && addr <= mb_config.end) {mb_addr = &mb_config; find_f = 2;}

	else if(addr >= mb_protect.start && addr <= mb_protect.end) {mb_addr = &mb_protect; find_f = 3;}

	else if(addr >= mb_ext_io.start && addr <= mb_ext_io.end) {mb_addr = &mb_ext_io; find_f = 4;}

	else if(addr >= mb_err.start && addr <= mb_err.end) {mb_addr = &mb_err; find_f = 5;}

	else if(addr >= mb_motor.start && addr <= mb_motor.end) {mb_addr = &mb_motor; find_f = 6;}

	else if(addr >= mb_device.start && addr <= mb_device.end) {mb_addr = &mb_device; find_f = 7;}

	else return MODBUS_ADDR_MAP_ERR;

	if(find_f)
	{
		index = MB_getActualAddress(mb_addr, addr, count);
		//kprintf(PORT_DEBUG, "\r\n addr=%d, count=%d, find=%d, st_addr=%d, end_addr=%d", addr, count, find_f, mb_addr->start, mb_addr->end);
	}

	return index;
}

void MB_readByte(uint8_t rcv_char)
{

	if(mb_start_flag == 0)
	{
		modbusRx.wp = 0;
		modbusRx.buf[modbusRx.wp++] = rcv_char;
  		mb_start_flag = 1;
  		MB_enableTimer();
	}
	else
	{
		if(modbusRx.wp < MODBUS_BUF_SIZE)
		{
			modbusRx.buf[modbusRx.wp++] = rcv_char;
			MB_enableTimer();
		}
		else
			mb_err_code = MOD_NORMAL_RX_ERR;
	}
}

void MB_writeRespPacket(int len)
{
	if (len == 0) return;

	RS485_TX_ENABLE(); osDelay(1);
	osSemaphoreWait(rs485SemaphoreIdHandle, osWaitForever);
//	memcpy(testMsgBuf, pStr, num);
	HAL_UART_Transmit(&huart3, (uint8_t *)modbusTx.buf, len, 1000);
	osSemaphoreRelease(rs485SemaphoreIdHandle);
	RS485_TX_DISABLE();
}


void MB_processTimerExpired(void)
{
	if(mb_start_flag == 1)
	{
		mb_start_flag = 0;
		mb_frame_received = 1;
	}
	MB_disableTimer();
}

int MB_isValidRecvPacket(void)
{
	if(modbusRx.buf[0] != mb_slaveAddress) return 0;

	// CRC error
	if(!MB_isCRC_OK(modbusRx.buf, modbusRx.wp)) return 0;

	return 1;
}

void MB_generateErrorResp(uint8_t func_code, uint8_t excep_code)
{
	memset(modbusTx.buf, 0, MODBUS_BUF_SIZE);

	modbusTx.buf[0] = mb_slaveAddress;
	modbusTx.buf[1] = func_code | 0x80;
	modbusTx.buf[2] = excep_code;
	modbusTx.wp = 3;

}

int MB_handleReadRegister(uint8_t func_code, uint16_t addr, uint16_t cnt)
{
	int i, result=MOD_EX_NO_ERR;
	uint16_t index;
	int32_t value;

	/* return error type
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataADD
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */

	// valid address range ?
	index = MB_convModbusAddr(addr, cnt);
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
		kprintf(PORT_DEBUG, "\r\n index=%d, value=%d, wp=%d", index, (uint16_t)value, modbusTx.wp);
	}
	else
	{
		for(i=0; i<cnt; i++)
		{
			//value = table_database_getValue(index + i);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0x0000FF00) >> 8);
			modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x000000FF);
		}
	}


FC03_ERR:
	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC03_RD_HREG, result);
	}

	return result;
}

int MB_handleWriteSingleRegister(uint16_t addr, uint16_t value)
{
	int ret, result=MOD_EX_NO_ERR;
	uint16_t index;

	/* return error type
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataVAL
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */

	// valid address range ?
	index = MB_convModbusAddr(addr, 1);
	if(index > MODBUS_ADDR_MAP_ERR-4) {result = MOD_EX_DataADD; goto FC06_ERR; }

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC06_WR_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

#if 0
	//TODO : update EEPROM, need new API
	ret = table_database_setValue(index, (int32_t)value, 0);
	if(ret == 1)
	{
		modbusTx.buf[modbusTx.wp++] = (uint8_t)((value&0xFF00) >> 8);
		modbusTx.buf[modbusTx.wp++] = (uint8_t)(value&0x00FF);
		//kprintf(PORT_DEBUG, "\r\n index=%d, value=%d, wp=%d", index, (uint16_t)value, modbusTx.wp);
		result = MOD_EX_NO_ERR;
	}
	else
		result = MOD_EX_SLAVE_FAIL;
#endif

FC06_ERR:
	if(result != MOD_EX_NO_ERR)
	{
		MB_generateErrorResp(MOD_FC06_WR_REG, result);
	}

	return result;
}

int MB_handleWriteMultiRegister(uint16_t addr, uint16_t count, uint16_t *value)
{
	int i, ret, result=MOD_EX_NO_ERR;
	uint16_t index;

	/* return error types
	 * - address range error : MOD_EX_DataADD
	 * - data count error : MOD_EX_DataVAL
	 * - function cannot be processed : MOD_EX_SLAVE_FAIL
	 */
	// count is over 123 or not matched with byte size
	if(count == MODBUS_COUNT_ERR) {result = MOD_EX_DataVAL; goto FC16_ERR; }

	// valid address range ?
	index = MB_convModbusAddr(addr, count);
	if(index > MODBUS_ADDR_MAP_ERR-4) {result = MOD_EX_DataADD; goto FC16_ERR; }

	modbusTx.wp = 0;
	modbusTx.buf[modbusTx.wp++] = mb_slaveAddress;
	modbusTx.buf[modbusTx.wp++] = MOD_FC16_WRM_REG;
	modbusTx.buf[modbusTx.wp++] = (uint8_t)((addr&0xFF00) >> 8);
	modbusTx.buf[modbusTx.wp++] = (uint8_t)(addr&0x00FF);

	for(i=0; i<count; i++)
	{
		// TODO : read EEPROM, update API
		//ret = table_database_setValue(index+i, (int32_t)value[i], 0);
		//kprintf(PORT_DEBUG, "\r\n index=%d, value=%d, ret=%d", index+i, (uint16_t)value[i], ret);
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
	//kprintf(PORT_DEBUG, "\r\n result=%d", result);
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
		ret_code = MB_handleWriteSingleRegister(reg_addr, value);
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
		//kprintf(PORT_DEBUG, "\r\n addr=%d, reg_cnt=%d, byte_cnt=%d", reg_addr, reg_cnt, byte_cnt);
		ret_code = MB_handleWriteMultiRegister(reg_addr, reg_cnt, multi_val);
		break;

	default: // not supported func_code
		ret_code = MOD_EX_FUNC; // illegal function code
		MB_generateErrorResp(func_code, ret_code);
		break;
	}
	calcCRC = CRC16(modbusTx.buf, modbusTx.wp);
	modbusTx.buf[modbusTx.wp++] = (int8_t)(calcCRC >> 8) &0x00FF;
	modbusTx.buf[modbusTx.wp] = (int8_t)(calcCRC & 0x00FF);
	MB_writeRespPacket(modbusTx.wp+1);

	return ret_code;
}

void MB_TaskFunction(void)
{
	int result=0;

	// get data from 485
	if(mb_frame_received)
	{
		// check received frame
		// wrong slave address
		if(MB_isValidRecvPacket() == 0) {mb_frame_received=0; return;}

		kprintf(PORT_DEBUG, "RX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
			modbusRx.buf[0], modbusRx.buf[1], modbusRx.buf[2], modbusRx.buf[3], modbusRx.buf[4]);

		// generate response or error frame
		result = MB_processModbusPacket();

		kprintf(PORT_DEBUG, "TX: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
				modbusTx.buf[0], modbusTx.buf[1], modbusTx.buf[2], modbusTx.buf[3], modbusTx.buf[4]);

		mb_frame_received=0;
	}

}
