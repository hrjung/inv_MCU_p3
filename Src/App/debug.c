/*
 * debug.c
 *
 *  Created on: 2018. 11. 9.
 *      Author: hrjung
 */


/* Includes ------------------------------------------------------------------*/

#define DEBUG_GLOBALS
#include "includes.h"

#include "main.h"
#include "cmsis_os.h"

#include "debug.h"
#include "proc_uart.h"
#include "table.h"
//#include "task.h"

#include "drv_nvm.h"
#include "drv_ST25DV.h"
#include "drv_gpio.h"

#ifdef SUPPORT_UNIT_TEST
#include "../test/unity.h"
#endif

/* Private variables ---------------------------------------------------------*/

#define KEYPAD_SCAN_TIME_INTERVAL	10

//#define CMDLINE_MAX_ARGS        8
#define NUM_OF_DEBUGCHAR    	64

const char	*help_msg[] = {
	"Display Command Help",
	"Usage: HELP [COMMAND]",
	0
};

const char	*reset_msg[] = {
	"RESET",
	"Usage: RESET",
    "       RESET ALL [YES]: Factory default init except MAP",
	0
};

const char	*init_param_msg[] = {
	"INITP : set EEPROM not initialized",
	"Usage: INITP ",
	0
};

const char	*lparam_msg[] = {
	"LPARAM",
	"Usage: LPARAM index l_value",
	0
};

const char	*fparam_msg[] = {
	"FPARAM",
	"Usage: FPARAM index f_value",
	0
};

const char	*read_nv_msg[] = {
	"RNV",
	"Usage: RNV addr ",
	0
};

const char	*write_nv_msg[] = {
	"WNV",
	"Usage: WNV addr value",
	0
};

const char	*din_msg[] = {
	"DIN",
	"Usage: DIN ",
	0
};

const char	*dout_msg[] = {
	"DOUT",
	"Usage: DOUT enable(0,1) value(0,1)",
	0
};

const char	*ain_msg[] = {
	"AIN",
	"Usage: AIN ",
	0
};

#if 0
const char	*aout_msg[] = {
	"AOUT",
	"Usage: AOUT enable(0,1) value(0~1023)",
	0
};

const char	*uio_test_msg[] = {
	"UIOT",
	"Usage: UIOT 0/1",
	0
};
#endif

const char	*test_msg[] = {
	"TEST",
	"Usage: TEST f_value",
	0
};


#ifdef SUPPORT_UNIT_TEST
const char	*utest_msg[] = {
	"UNIT TEST",
	"Usage: UTEST ",
	0
};
#endif

static int display_BoardInfo(uint8_t dport);

STATIC int help_ser(uint8_t dport);
STATIC int reset_ser(uint8_t dport);
STATIC int init_param_ser(uint8_t dport);
STATIC int lparam_ser(uint8_t dport);
STATIC int fparam_ser(uint8_t dport);
STATIC int read_nv_ser(uint8_t dport);
STATIC int write_nv_ser(uint8_t dport);
STATIC int din_ser(uint8_t dport);
STATIC int dout_ser(uint8_t dport);
STATIC int ain_ser(uint8_t dport);
//STATIC int aout_ser(uint8_t dport);
//STATIC int uio_enable_ser(uint8_t dport);
STATIC int test_ser(uint8_t dport);
#ifdef SUPPORT_UNIT_TEST
STATIC int utest_ser(uint8_t dport);
#endif

const COMMAND	Cmd_List[] =
{
	{ 1,   	"HELP",			2,		help_ser,			help_msg	},
	{ 1,  	"RESET",		1,		reset_ser,			help_msg	},
	{ 1,  	"INITP",		1,		init_param_ser,		init_param_msg	},
	{ 1,  	"LPARAM",		3,		lparam_ser,			lparam_msg	},
	{ 1,  	"FPARAM",		3,		fparam_ser,			fparam_msg	},
	{ 1,  	"RNV",			2,		read_nv_ser,		read_nv_msg	},
	{ 1,  	"WNV",			3,		write_nv_ser,		write_nv_msg},
	{ 1,  	"DIN",			1,		din_ser,			din_msg		},
	{ 1,  	"DOUT",			2,		dout_ser,			dout_msg	},
	{ 1,  	"AIN",			1,		ain_ser,			ain_msg		},
//	{ 1,  	"AOUT",			2,		aout_ser,			aout_msg	},
//	{ 1,  	"UIOT",			2,		uio_enable_ser,		uio_test_msg},
	{ 1,  	"TEST",			3,		test_ser,			test_msg	},
#ifdef SUPPORT_UNIT_TEST	
	{ 1,  	"UTEST",		1,		utest_ser,			utest_msg	},
#endif	
	{ 0, 	(const char *)0,0,		0,			(const char **)0	}
};


uint8_t debugIoFlag=0;
int16_t	debug_adc_val=0;
int8_t debug_di_val=0;

/* Global variables ---------------------------------------------------------*/

//extern uint32_t AO_duty;
//extern uint8_t AO_enable;
//extern uint8_t din_val;
extern uint8_t	mb_slaveAddress;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc1;
extern osSemaphoreId debugSemaphoreIdHandle;
extern osTimerId keyScanTimerHandle;

extern uint8_t mdin_value[];
extern uint16_t ain_val[];
extern uint32_t ain_sum;
extern uint16_t adc_value;
extern volatile int8_t ADC_ConvCpltFlag, ADC_error;
extern int32_t table_nvm[];

//extern osThreadId defaultTaskHandle;
//extern osThreadId YstcNfcTaskHandle;
//extern osThreadId userIoTaskHandle;
//extern osThreadId YstcEventTaskHandle;
//extern osThreadId rs485TaskHandle;

extern uint32_t ystc_event_cnt, default_cnt, nfc_cnt, rs485_cnt, user_io_cnt;
extern uint32_t key_scan_cnt, nfc_app_cnt, ystc_trigger_cnt;
//extern uint32_t ystc_default_cnt, ystc_run_cnt;
//extern int ystc_state;
//extern int ystc_enter, ystc_exit;
//extern int run_enter, run_enter1, run_enter2, run_enter3, run_enter4,run_enter5;
//extern int run_exit, run_exit1, run_exit2,run_exit3, run_exit4,run_exit5;
//extern int rs_f1,rs_f2,rs_f3,rs_f4,rs_f5,rs_f6, i2c_st;
extern eTaskState eTaskGetState( TaskHandle_t xTask );
#ifdef SUPPORT_TASK_WATCHDOG
extern uint8_t watchdog_f;
#endif

//extern void set_AO_duty(uint8_t bool_use, uint32_t in);
//extern void set_DO_duty(uint8_t bool_use, uint8_t in);
extern uint16_t EXT_AI_readADC(void);
extern void MB_UART_init(uint32_t baudrate);
extern void UTIL_writeDout(uint8_t index, uint8_t onoff);
extern uint8_t NVM_clearInit(void); // to re-initialize EEPROM
extern uint16_t NVM_getSystemParamAddr(uint16_t index);

extern uint8_t ERR_getErrorState(void);
extern void test_setTableValue(PARAM_IDX_t idx, int32_t value);
extern int32_t table_getInitValue(PARAM_IDX_t index);
extern uint16_t table_getAddr(PARAM_IDX_t index);

#ifdef SUPPORT_UNIT_TEST
// Unit test function
//nvm_queue test
extern void test_nfc_q_basic(void);
extern void test_nfc_q_muliple(void);
extern void test_table_q_basic(void);
extern void test_table_q_muliple(void);
//error
extern void test_errorBasic(void);
//dsp_comm
extern void test_getRecvLength(void);
extern void test_generateMessage(void);
extern void test_parseValue(void);
extern void test_parseMessage(void);
extern void test_sendCommand(void);
//ext_io
extern void test_setupMultiFuncDin(void);
extern void test_convertMultiStep(void);
extern void test_handleDin(void);
extern void test_handleDout(void);
extern void test_getFreq(void);
extern void test_handleAin(void);
// table
extern void test_setValue(void);


// modbus
extern void test_modbusBasic(void);
extern void test_modbusAddress(void);
extern void test_modbusGetValue(void);
extern void test_modbusFC03(void);
extern void test_modbusFC03_multi(void);
extern void test_modbusFC04(void);
extern void test_modbusFC04_multi(void);
extern void test_modbusFC06(void);

#endif

/* Private function prototypes -----------------------------------------------*/
#if 0
void display_current_time(void)
{
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	printf("\r\n DATE = %04d-%02d-%02d %02d:%02d:%02d\r\n", \
			sDate.Year+2000, sDate.Month, sDate.Date, \
			sTime.Hours, sTime.Minutes, sTime.Seconds);
}
#endif

char _ToUpper(char c)
{
	if ('a' <= c && c <= 'z')
		return c-0x20;
	return c;
}

int get_cmd(uint8_t dport, char *Cmd)
{
	uint8_t	ch;
	int     pos=0;

	while(1)
	{
		while (!kgetc(dport,&ch)) osDelay(1);

		switch (ch)
		{
		case '\b' :
			if (pos > 0)
			{
				kputc(dport,'\b');
				pos--;
			}
			else
				kputc(dport,0x07);
			break;
		case '\r' :
		case '\n' :
			*(Cmd+pos) = 0x00;
			kputc(dport,(char)ch);
			return pos;
		default :
			kputc(dport,_ToUpper((char)ch));
			Cmd[pos++] = _ToUpper((char)ch);
			break;
		}

		if (pos > NUM_OF_DEBUGCHAR)
		{
			kputs(dport,"\nout of length(enter command)");
			return -1;
		}
	}
}

int get_arg(char *args)
{
	int8_t   bflag = 1;
	int     idx = 0;

	arg_c = 0;
	while(1)
	{
		switch(args[idx])
		{
		case '\0':
			return arg_c;
		case ' ':
			if(!bflag)
			{
				bflag = 1;
				args[idx] = 0;
			}
			idx++;
			break;
		default :
			if(bflag)
			{
				arg_v[arg_c] = &args[idx];
				arg_c++;
				bflag = 0;
			}
			idx++;
		}
	}
}

int exe_cmd(uint8_t dport)
{
	COMMAND *cmd = (COMMAND *)Cmd_List;

	for(; cmd->flag != 0; cmd++)
	{
		if(!strcmp((const char *)arg_v[0], cmd->cmd_str))
		{
			cmd->cmd(dport);
			return 1;
		}
	}
	kputs(dport, "\r\nNoCmd");
	return 0;
}

STATIC int help_ser(uint8_t dport)
{
    int i, j;

	if (arg_c == 1)
	{
	    //for (i=0; Cmd_List[i].cmd_str != (char *)0; i++)
		for (i=0; Cmd_List[i].flag != 0; i++)
	    {
			if ((i % 7) == 0)
				kputs(dport, "\r\n");

			kprintf(dport, "%10s\r\n", Cmd_List[i].cmd_str);
	    	//kputs(dport, Cmd_List[i].cmd_str);
    	}
    }
	else if (arg_c == 2)
	{
	    for (i=0; Cmd_List[i].cmd_str != (char *)0; i++)
	    {
	    	if (!strcmp(arg_v[1], Cmd_List[i].cmd_str))
	    	{
	    		for (j = 0; Cmd_List[i].help_str[j] != (char *)0; j++)
	    		{
	    			kprintf(dport, "\r\n%s", Cmd_List[i].help_str[j]);
	    		}
	    		kputs(dport, "\r\n");
    			return 0;
	    	}
    	}
    }
    kputs(dport, "\r\n");
    return 0;
}

STATIC int reset_ser(uint8_t dport)
{
	kputs(dport, "\r\n\r\n---Reset---\r\n\r\n");
	HAL_NVIC_SystemReset();
	return 0;
}


STATIC int init_param_ser(uint8_t dport)
{
	kputs(dport, "\r\n\r\n---set EEPROM not initialized");
	kputs(dport, "\r\n please reset device	\r\n\r\n");
	//EEPROM_setNotInitiated();
	return 0;
}

STATIC int lparam_ser(uint8_t dport)
{
	uint16_t index;
	int32_t l_val;
	int result;

	if(arg_c != 2 && arg_c != 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	index = (uint16_t)atoi(arg_v[1]);
	if(index >= PARAM_TABLE_SIZE)
	{
		kprintf(dport, "\r\n out of range index value");
		return -1;
	}
#if 0
	if(arg_c == 2)
	{
		l_val = table_database_getValue(index);
		kprintf(dport, "\r\n index=%d value = %d", index, l_val);
	}
	else if(arg_c == 3)
	{
		l_val = (int32_t)atoi(arg_v[2]);
		result = table_database_setValue_DBGI(index, l_val);
		if(result == -1)
		{
			kprintf(dport, "\r\n out of range index value");
			return -1;
		}
		else if(result == 0)
		{
			kprintf(dport, "\r\n parameter set error!");
			return -1;
		}
	}
#endif
	return 0;
}

STATIC int fparam_ser(uint8_t dport)
{
	uint16_t index;
	int32_t f_val;
	int result;

	if(arg_c != 2 && arg_c != 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	index = (uint16_t)atoi(arg_v[1]);
	if(index >= PARAM_TABLE_SIZE)
	{
		kprintf(dport, "\r\n out of range index value");
		return -1;
	}

	if(arg_c == 2)
	{
		//f_val = table_database_getValue(index);
		kprintf(dport, "\r\n index=%d fvalue = %f", index, (float)(f_val)/10.0);
	}
	else if(arg_c == 3)
	{
		f_val = (int32_t)atoi(arg_v[2]);
		//result = table_database_setValue_DBGF(index, f_val);
		if(result == -1)
		{
			kprintf(dport, "\r\n wrong parameter type");
			return -1;
		}
		else if(result == 0)
		{
			kprintf(dport, "\r\n parameter set error!");
			return -1;
		}
	}

	return 0;
}

STATIC int read_nv_ser(uint8_t dport)
{
	uint16_t addr;
	int32_t i2c_rvalue=0;
	uint8_t i2c_status;

	if(arg_c != 2)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	addr = (uint16_t)atoi(arg_v[1]);
	if(addr%4 != 0 || addr > 0x3FF)
	{
		kprintf(dport, "\r\n address error %d", addr);
		return -1;
	}

	if(arg_c == 2)
	{
		i2c_status = I2C_readData((uint8_t *)&i2c_rvalue, addr, 4);
		kprintf(dport, "\r\n read EEPROM addr=%d, value = %d, status=%d", addr, i2c_rvalue, i2c_status);
	}

	return 0;
}

STATIC int write_nv_ser(uint8_t dport)
{
	uint16_t addr;
	int32_t i2c_wvalue=0;
	uint8_t i2c_status;

	if(arg_c != 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	addr = (uint16_t)atoi(arg_v[1]);
	if(addr%4 != 0 || addr > 0x3FF)
	{
		kprintf(dport, "\r\n address error %d", addr);
		return -1;
	}

	i2c_wvalue = (int32_t)atoi(arg_v[2]);

	if(arg_c == 3)
	{
		i2c_status = I2C_writeData((uint8_t *)&i2c_wvalue, addr, 4);
		kprintf(dport, "\r\n write EEPROM addr=%d, value = %d, status=%d", addr, i2c_wvalue, i2c_status);

	}

	return 0;
}

STATIC int din_ser(uint8_t dport)
{
	if(arg_c != 1)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	UTIL_readDin();
	kprintf(dport, "\r\n set Din mdin %d, %d, %d", mdin_value[0], mdin_value[1], mdin_value[2]);

	return 0;
}

STATIC int dout_ser(uint8_t dport)
{
	uint8_t index, onoff;

	if(arg_c != 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	index = (uint8_t)atoi(arg_v[1]);
	if(index != 0 && index != 1) return 1;

	onoff = (uint8_t)atoi(arg_v[2]);
	if(onoff != 0 && onoff != 1) return 1;

	UTIL_writeDout(index, onoff);
	kprintf(dport, "\r\n set Dout index=%d, value=%d", index, onoff);

	return 0;
}

STATIC int ain_ser(uint8_t dport)
{
	int i;
	//uint16_t ain_val[EXT_AIN_SAMPLE_CNT], ain_sum;

	if(arg_c != 1)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	if(ADC_error)
		kprintf(dport, "\r\n ADC error");
	else
	{
		kprintf(dport, "\r\n ADC val = %d %d %d %d  %d %d %d %d", \
				ain_val[0],ain_val[1],ain_val[2],ain_val[3], ain_val[4],ain_val[5],ain_val[6],ain_val[7]);
		kprintf(dport, "\r\n ADC val = %d %d %d %d  %d %d %d %d", \
						ain_val[8],ain_val[9],ain_val[10],ain_val[11], ain_val[12],ain_val[13],ain_val[14],ain_val[15]);
		kprintf(dport, "\r\n set Ain sum=%d, value=%d %d", ain_sum, (int)adc_value, (int)EXT_AI_readADC());
		//kprintf(dport, "\r\n read Ain value=%d", (int)EXT_AI_readADC());
	}

//	ADC_ConvCpltFlag=0;
//	ADC_error = 0;
	return 0;
}

#if 0
STATIC int aout_ser(uint8_t dport)
{
	uint8_t enable;
	uint32_t duty;

	if(arg_c != 2 && arg_c != 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	enable = (uint8_t)atoi(arg_v[1]);
	if(enable != 0 && enable != 1) return 1;

	if(arg_c == 2)
	{
		set_AO_duty(enable, 0);
		kprintf(dport, "\r\n AO enable=%d ", enable);
	}
	else if(arg_c == 3)
	{
		duty = (uint32_t)atoi(arg_v[2]);
		if(duty >=0 && duty <= 100)
		{
			set_AO_duty(enable, duty);
			kprintf(dport, "\r\n set AO enable=%d, duty=%d ", enable, duty);
		}
		else
		{
			kprintf(dport, "\r\n ERROR! duty should be 0 ~ 100 ");
			return 1;
		}
	}

	return 0;
}


STATIC int uio_enable_ser(uint8_t dport)
{
	uint8_t on_off=0;

	if(arg_c != 2)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	on_off = (uint8_t)atoi(arg_v[1]);
	if(on_off != 0 && on_off != 1)
	{
		kprintf(dport, "\r\n only 0(off) and 1(on) available!");
		return -1;
	}

	debugIoFlag = on_off;

	kprintf(dport, "\r\n set debugIo %d", debugIoFlag);

	return 0;
}
#endif

STATIC int test_ser(uint8_t dport)
{
	int idx;
	int32_t value;
	uint8_t status;
	uint16_t addr;
	int test_case, arg1;

	if(arg_c > 4)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	test_case = atoi(arg_v[1]);
    if(test_case == 0)
    	test_case = arg_v[1][0];

    if(test_case == '0')
    {
//    	addr = (uint16_t)atoi(arg_v[2]);
//    	status = NVM_read(addr, &value);
//    	kprintf(dport, "\r\n NVM_read addr=%d, value=%d, status=%d", addr, (int)value, status);
    	idx = (int)atoi(arg_v[2]);
    	status = NVM_readParam((PARAM_IDX_t)idx, &value);
    	kprintf(dport, "\r\n NVM_readParam idx=%d, value=%d, status=%d", idx, (int)value, status);
    }
    else if(test_case == 1)
    {
//    	addr = (uint16_t)atoi(arg_v[2]);
//    	value = table_getInitValue(idx);
//    	status = NVM_write(addr, value);
//		kprintf(dport, "\r\n NVM_write addr=%d, value=%d status=%d", addr, value, status);
    	idx = (int)atoi(arg_v[2]);
    	value = table_getInitValue(idx);
    	status = NVM_writeParam((PARAM_IDX_t)idx, value);
    	kprintf(dport, "\r\n NVM_writeParam idx=%d, value=%d status=%d", idx, value, status);
    	status = NVM_setCRC();
    	kprintf(dport, "\r\n NVM_setCRC() status=%d", status);
    }
    else if(test_case == 2)
    {
    	status = NVM_clearInit();
		kprintf(dport, "\r\n re-initialize EEPROM, please reset...  %d\r\n", status);
    }
    else if(test_case == 3)
    {
    	int enable=0;

    	enable = (int)atoi(arg_v[2]);
    	if(enable)
    	{
    		test_setTableValue(ctrl_in_type, CTRL_IN_Analog_V);
    		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ain_val, EXT_AIN_SAMPLE_CNT);
    		kprintf(dport, "\r\n AIN test start");
    	}
    	else
    	{
    		test_setTableValue(ctrl_in_type, CTRL_IN_NFC);
    		kprintf(dport, "\r\n AIN test stop");
    	}
    }
    else if(test_case == 4)
    {
    	idx = (int)atoi(arg_v[2]);
    	value = table_getValue(idx);
    	kprintf(dport, "\r\n index=%d, value=%d", idx, (int)value);
    }
    else if(test_case == 5)
    {

    	idx = (uint16_t)atoi(arg_v[2]);
    	if(idx < PARAM_TABLE_SIZE)
    	{
			value = table_getValue(idx);
			kprintf(dport, "\r\n table[%d]=%d", idx, (int)value);
    	}
    	else
    		kprintf(dport, "\r\n index=%d error", idx);
    }
    else if(test_case == 6)
    {
		kprintf(dport, "\r\n error code = %d", (int)ERR_getErrorState());
    }
    else if(test_case == 7)
    {
		uint8_t txBuf[5];
		HAL_StatusTypeDef status;

		txBuf[0] = 0xAA; //
		txBuf[1] = 0x55;
		txBuf[2] = 0xCC;
		txBuf[3] = 0x33;
		txBuf[4] = 0x11;

		//HAL_GPIO_WritePin(MCU_nSCS_OUT2_GPIO_Port, MCU_nSCS_OUT2_Pin, GPIO_PIN_RESET); // OE decoder

		// send the address of indexed register
		status = HAL_SPI_Transmit(&hspi1, txBuf, 5, 100);
		osDelay(1);
		status = HAL_SPI_Transmit(&hspi1, txBuf, 5, 100);
		kprintf(dport, "\r\n SPI write, status= %d", status);
	}
    else if(test_case == 8) // show all EEPROM table
    {
    	uint8_t i, status=0;
    	uint16_t i2c_addr, i2c_len=4;
    	uint32_t i2c_value=0;

    	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
    	{
    		i2c_addr = NVM_getSystemParamAddr(i);
    		status = I2C_readData((uint8_t *)&i2c_value, i2c_addr, i2c_len);
    		kprintf(dport, "\r\n addr=%d, value=%d, status=%d, ", i2c_addr, i2c_value, status);
    		osDelay(5);
    	}

    	for(i=0; i<PARAM_TABLE_SIZE; i++)
    	{
    		i2c_addr = table_getAddr((PARAM_IDX_t)i);
    		status = I2C_readData((uint8_t *)&i2c_value, i2c_addr, i2c_len);
    		kprintf(dport, "\r\n idx=%d, value=%d, nvm=%d, status=%d, ", i, i2c_value, table_nvm[i], status);
    		osDelay(5);
    	}
    }
    else if(test_case == 9)
    {
    	uint16_t b_index;
    	uint32_t mb_baudrate[] = {2400, 4800, 9600, 19200, 38400};


    	b_index = (int)atoi(arg_v[2]);
    	MB_UART_init(mb_baudrate[b_index]);
		kprintf(dport, "\r\n MB_address=%d, baudrate=%d", mb_slaveAddress, mb_baudrate[b_index]);
    }
    else if(test_case == 'A')
    {
    	idx = (int)atoi(arg_v[2]);
    	value = (int32_t)atoi(arg_v[3]);
    	status = table_runFunc(idx, (int32_t)value, REQ_FROM_MODBUS);
    	kprintf(dport, "\r\n idx=%d, value=%d, status=%d, ", idx, value, status);
    }
    else if(test_case == 'B')
    {
    	arg1 = (int)atoi(arg_v[2]);
    	switch(arg1)
    	{
    	case 0 : UTIL_setLED(LED_COLOR_OFF, 0); break;
    	case 1 : UTIL_setLED(LED_COLOR_R, 0); break;
    	case 2 : UTIL_setLED(LED_COLOR_G, 0); break;
    	case 3 : UTIL_setLED(LED_COLOR_B, 0); break;
    	}

		kprintf(dport, "\r\n set LED %d", arg1);
    }
    else if(test_case == 'C') // temp test only
    {
    	idx = 0;
    	value = 300;
    	status = NVM_writeParam((PARAM_IDX_t)idx, (int32_t)300);
    	kprintf(dport, "\r\n NVM_writeParam idx=%d, value=%d status=%d", idx, value, status);
    	status = NVM_setCRC();
    	kprintf(dport, "\r\n NVM_setCRC() status=%d", status);
    }
#if 0
    else if(test_case == 'B')
    {
		kprintf(dport, "\r\n Task ystcEv:%d, userIO: %d, nfc: %d, 485: %d ", \
				ystc_event_cnt, user_io_cnt, nfc_cnt, rs485_cnt);
		kprintf(dport, "\r\n Timer keyscan: %d, nfcApp: %d, trg: %d, def:%d ", \
				key_scan_cnt, nfc_app_cnt, ystc_trigger_cnt, default_cnt);
		kprintf(dport, "\r\n Ystc run: %d, default: %d, ys_state=%d", \
				ystc_run_cnt, ystc_default_cnt, ystc_state);
		kprintf(dport, "\r\n enter:%d 1:%d, 2:%d 3:%d, 4:%d 5:%d", \
				run_enter,run_enter1,run_enter2, run_enter3,run_enter4,run_enter5);
		kprintf(dport, "\r\n exit:%d 1:%d, 2:%d 3:%d, 4:%d 5:%d", \
				run_exit,run_exit1,run_exit2, run_exit3,run_exit4,run_exit5);
		kprintf(dport, "\r\n rsf 1:%d, 2:%d 3:%d, 4:%d, 5:%d, 6:%d, st=%d", \
				rs_f1,rs_f2,rs_f3, rs_f4,rs_f5,rs_f6, i2c_st);
    }
    else if(test_case == 'C')
    {
#ifdef SUPPORT_TASK_WATCHDOG
    	kprintf(dport, "\r\n watchdog_f=%d", watchdog_f);
#endif
    	//to use osThreadGetState(), need to set INCLUDE_eTaskGetState 1 in FreeRTOS.h
//    	eTaskState ye_state, nf_state, rs_state, us_state;
//    	ye_state = osThreadGetState(YstcEventTaskHandle);
//    	nf_state = osThreadGetState(YstcNfcTaskHandle);
//    	rs_state = osThreadGetState(rs485TaskHandle);
//    	us_state = osThreadGetState(userIoTaskHandle);
//
//    	kprintf(dport, "\r\n state YE: %d, NF: %d, RS: %d, US: %d", ye_state, nf_state, rs_state, us_state);
    }
#endif
    else
    {
		kprintf(dport, "\r\n test_case(%d) is not available", test_case);
    }

	return 0;
}

#ifdef SUPPORT_UNIT_TEST
STATIC int utest_ser(uint8_t dport)
{
    kputs(dport, "--Unit Test Running\r\n");

	UNITY_BEGIN();

#if 1
	// add nvm_queue test
	RUN_TEST(test_nfc_q_basic);
	RUN_TEST(test_nfc_q_muliple);

	RUN_TEST(test_table_q_basic);
	RUN_TEST(test_table_q_muliple);

	// add DSP eror test
	RUN_TEST(test_errorBasic);

	// Dsp comm
	RUN_TEST(test_getRecvLength);
	RUN_TEST(test_generateMessage);
	RUN_TEST(test_parseValue);
	RUN_TEST(test_parseMessage);
	RUN_TEST(test_sendCommand);

	// ext_di_
	RUN_TEST(test_setupMultiFuncDin);
	RUN_TEST(test_convertMultiStep);
	RUN_TEST(test_handleDin);
	//ext_do
	RUN_TEST(test_handleDout);
	//ext_ai
	RUN_TEST(test_getFreq);
	RUN_TEST(test_handleAin);

	// table
	RUN_TEST(test_setValue);
#endif

	// modbus
	RUN_TEST(test_modbusBasic);
	RUN_TEST(test_modbusAddress);
	RUN_TEST(test_modbusGetValue);
	RUN_TEST(test_modbusFC03);
	RUN_TEST(test_modbusFC03_multi);
	RUN_TEST(test_modbusFC04);
	RUN_TEST(test_modbusFC04_multi);
	RUN_TEST(test_modbusFC06);
	//RUN_TEST(test_modbusFC16);

	UNITY_END();

	kputs(dport, " End of Unit Test!\r\n Reset Device!!\r\n");

	//UARTFlushTx(0);
	while(1); // cause device reset by watchdog

	return 0;
}
#endif

static int display_BoardInfo(uint8_t dport)
{
	kputs(dport, "\r\n*************************************************");
	kputs(dport, "\r\n    Nara Inverter MCU ");
	kputs(dport, "\r\n*************************************************\r\n");

	return 0;
}

void debugTaskFunc(void const * argument)
{

  osDelay(1000);

  kputs(PORT_DEBUG, "start debugTask\r\n");
  /* Infinite loop */

  display_BoardInfo(PORT_DEBUG);

  while(1)
  {
	memset((void *)Cmd, 0, CMD_BUF_SIZE);
	kputs(PORT_DEBUG, "\r\ndebug> ");

	if(get_cmd(PORT_DEBUG, (char *)Cmd) > 0)
	{
		if(get_arg((char *)Cmd) > 0)
		{
			kputs(PORT_DEBUG, "\r\n");
			exe_cmd(PORT_DEBUG);
		}
	}
	osDelay(1);
  }
}


#if 0
void ProcessDebugCommand(void)
{
    int ret;

    kputc('\n');
    kputs("debug>");

    if(get_cmd((char *)Cmd) > 0)
    {
    	if(get_arg((char *)Cmd) > 0)
    	{
    		kputc('\n');
    		exe_cmd();
    		//memset((void *)Cmd, 0, CMD_BUF_SIZE);
    	}
    }
}
#endif
