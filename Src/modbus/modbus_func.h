/*
 * modbus_func.h
 *
 *  Created on: 2019. 6. 28.
 *      Author: hrjung
 */

#ifndef SRC_MODBUS_FUNC_H_
#define SRC_MODBUS_FUNC_H_

#include "cmsis_os.h"
#include "main.h"

/*
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


#define MODBUS_BUF_SIZE		256

#define MODBUS_ADDR_MAP_SIZE	32

#define MODBUS_ADDR_MAP_ERR		0xFFFF
#define MODBUS_COUNT_ERR		0xFFFF

// special purpose parameter for PC application
#define MB_CTRL_RESET_ADDR			(40090)
#define MB_CTRL_FACTORY_MODE_ADDR	(40092)
#define MB_CTRL_RUN_STOP_ADDR		(40095)

// normal parameter
#define MB_DRIVER_START_ADDR		(40100)
#define MB_DRIVER_END_ADDR			(40124)

#define MB_CONFIG_START_ADDR		(40200)
#define MB_CONFIG_END_ADDR			(40208)

#define MB_PROTECT_START_ADDR		(40280)
#define MB_PROTECT_END_ADDR			(40287)

#define MB_EXT_IO_START_ADDR		(40300)
#define MB_EXT_IO_END_ADDR			(40313)

#define MB_MOTOR_START_ADDR			(40020)
#define MB_MOTOR_END_ADDR			(40027)

#define MB_DEVICE_START_ADDR		(40060)
#define MB_DEVICE_END_ADDR			(40065)

#define MB_ERROR_START_ADDR			(40400)
#define MB_ERROR_END_ADDR			(40424)

#define MB_STATUS_START_ADDR		(40160)
#define MB_STATUS_END_ADDR			(40166)


/*
RX485 PDU max size = 253bytes
RS485 ADU max size = ADDR(1) + Modbus PDU + CRC(2) = 256bytes

RX: mb_req_pdu = {func_code, req_data}
TX: mb_rsp_pdu = {func_code, rsp_data}
    mb_excep_rsp_pdu = {exception-func_code, exception_code}

data model

*/
typedef struct {
	uint16_t	wp;
	uint8_t		buf[MODBUS_BUF_SIZE];
} MODBUS_SLAVE_QUEUE;



typedef struct {
	uint8_t		valid;
	uint8_t		rd_only; // 0: read/write, 1: read only, 2: for status(read from variable, not EEPROM)
	uint16_t 	conv_index;
} MODBUS_addr_map_st;

typedef struct {
	uint16_t		start;
	uint16_t		end;
	uint16_t 		start_index;
	MODBUS_addr_map_st 		map[MODBUS_ADDR_MAP_SIZE];

} MODBUS_addr_st;

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

//---------------- Modbus Function code define
enum
{
	MOD_FC02_RD_DI		=0x02,		// Read Discrete Inputs
	MOD_FC01_RD_COIL	=0x01,		// Read Coil
	MOD_FC05_WR_COIL	=0x05,		// Write Single Coil
	MOD_FC15_WR_COIL	=0x0F,		// Write Single Coil

	MOD_FC03_RD_HREG	=0x03,		// Read Holding Registers
	MOD_FC04_RD_IREG	=0x04,		// Read Input Registers (Read Only)
	MOD_FC06_WR_REG		=0x06,		// Write Single Register
	MOD_FC16_WRM_REG	=0x10,		// Write Multiple Registers
	MOD_FC08_DIAGNOSTIC	=0x08,		// Diagnostics
};


//---------------- Modbus Exception code define
enum
{
	MOD_EX_NO_ERR		=0x0,		// no error
	MOD_EX_FUNC			=0x01,		// ILLEGAL FUNCTION
	MOD_EX_DataADD		=0x02,		// ILLEGAL DATA ADDRESS
	MOD_EX_DataVAL		=0x03,		// ILLEGAL DATA VALUE
	MOD_EX_SLAVE_FAIL	=0x04,		// SLAVE DEVICE FAILURE
	MOD_EX_SLAVE_BUSY	=0x06,		// SLAVE DEVICE BUSY
};


enum
{
	MOD_NORMAL_ERR_NONE	= 0,
	MOD_NORMAL_RX_ERR,
};

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/


extern void MB_initAddrMap(void);

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#endif /* SRC_MODBUS_FUNC_H_ */
