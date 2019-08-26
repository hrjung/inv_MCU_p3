/*
 * dsp_comm.h
 *
 *  Created on: 2019. 6. 11.
 *      Author: hrjung
 */

#ifndef SRC_DSP_COMM_H_
#define SRC_DSP_COMM_H_


#define STATE_DEFAULT 	0
#define STATE_SUCCESS 	1
#define STATE_FAILED 	2

#define COMM_DEFAULT 	0
#define COMM_SUCCESS 	1
#define COMM_FAILED 	2
//#define COMM_NAK		3

#define SPI_REPEAT_CNT		5

#define COMM_ERR_COUNT_LIMIT	5

#define SPI_TEST_CMD_TEST_MODE  0
#define SPI_TEST_CMD_RESET      1
#define SPI_TEST_CMD_SPI_TEST	2	// SPI test command
#define SPI_TEST_CMD_DTM0_READ	3	// set DTM pin as 00
#define SPI_TEST_CMD_DTM1_READ	4 	// set DTM pin as 01
#define SPI_TEST_CMD_DTM2_READ	5 	// set DTM pin as 10
#define SPI_TEST_CMD_DTM3_READ	6	// set DTM pin as 11
#define SPI_TEST_CMD_MTD_READ	7


typedef enum
{
	SPICMD_CTRL_RUN		=0x0001,
	SPICMD_CTRL_STOP	=0x0002,
	SPICMD_CTRL_DIR_F	=0x0004,
	SPICMD_CTRL_DIR_R	=0x0008,
	SPICMD_PARAM_W		=0x0010,
	SPICMD_PARAM_R		=0x0020,
	SPICMD_REQ_ST		=0x0040,
	SPICMD_REQ_ERR		=0x0080,

	SPICMD_RESP_ACK		=0x0100,
	SPICMD_RESP_ST		=0x0200,
	SPICMD_RESP_ERR		=0x0400,
	SPICMD_RESP_PARAM	=0x0800,

	SPICMD_TEST_CMD		=0x1000,

} COMM_CMD_t;

extern int8_t COMM_convertValue(PARAM_IDX_t table_idx, uint16_t *buf);
extern int8_t COMM_sendMessage(COMM_CMD_t cmd, const uint16_t* data);

extern int8_t COMM_sendTestCmd(uint16_t cmd);

#endif /* SRC_DSP_COMM_H_ */
