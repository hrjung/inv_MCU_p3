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

} COMM_CMD_t;

#if 0
typedef enum {
	CTRL_RUN_INDEX =0,
	CTRL_STOP_INDEX,
	CTRL_DIR_F_INDEX,
	CTRL_DIR_R_INDEX,
	PARAM_W_INDEX,
	PARAM_R_INDEX,
	REQ_ST_INDEX,
	REQ_ERR_INDEX,

	RESP_ACK_INDEX,
	RESP_ST_INDEX,
	RESP_ERR_INDEX,
	RESP_PARAM_INDEX,

} COMM_CMD_INDEX_t;

typedef struct {
	int16_t		index;
	uint16_t	cmd;
	uint16_t	len;
} DSP_COMM_t ;
#endif

extern int8_t COMM_convertValue(PARAM_IDX_t table_idx, uint16_t *buf);
extern int8_t COMM_sendMessage(COMM_CMD_t cmd, const uint16_t* data);

#endif /* SRC_DSP_COMM_H_ */
