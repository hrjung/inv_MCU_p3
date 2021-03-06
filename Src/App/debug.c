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
#include "error.h"
#include "dsp_comm.h"
#include "handler.h"
//#include "task.h"
#include "nvm_queue.h"

#include "drv_nvm.h"
#include "drv_ST25DV.h"
#include "drv_gpio.h"
#include "ext_io.h"

#ifdef SUPPORT_UNIT_TEST
#include "../test/unity.h"
#endif

/* Private variables ---------------------------------------------------------*/


//#define CMDLINE_MAX_ARGS        8
#define NUM_OF_DEBUGCHAR    	80

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
	"INITP : initialize EEPROM as default",
	"Usage: INITP ",
	0
};

const char	*param_msg[] = {
	"PARAM",
	"Usage: PARAM index l_value",
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

#ifdef SUPPORT_PASSWORD
const char	*pass_msg[] = {
	"PASS",
	"Usage: PASS password",
	0
};

const char	*lock_msg[] = {
	"LOCK",
	"Usage: LOCK 0/1",
	0
};
#endif

const char	*test_msg[] = {
	"TEST",
	"Usage: TEST f_value",
	0
};

#ifdef SUPPORT_PRODUCTION_TEST_MODE
const char	*ptest_msg[] = {
	"PTEST",
	"Usage: PTEST ",
	0
};
#endif

#ifdef SUPPORT_UNIT_TEST
const char	*utest_msg[] = {
	"UNIT TEST",
	"Usage: UTEST ",
	0
};
#endif


const char	*version_msg[] = {
	"VERSION INFO",
	"Usage: VER ",
	0
};

static int display_BoardInfo(uint8_t dport);

STATIC int help_ser(uint8_t dport);
STATIC int reset_ser(uint8_t dport);
STATIC int init_param_ser(uint8_t dport);
STATIC int param_ser(uint8_t dport);
STATIC int read_nv_ser(uint8_t dport);
STATIC int write_nv_ser(uint8_t dport);
STATIC int din_ser(uint8_t dport);
STATIC int dout_ser(uint8_t dport);
STATIC int ain_ser(uint8_t dport);
//STATIC int aout_ser(uint8_t dport);
//STATIC int uio_enable_ser(uint8_t dport);
#ifdef SUPPORT_PASSWORD
STATIC int pass_ser(uint8_t dport);
STATIC int lock_ser(uint8_t dport);
#endif
STATIC int test_ser(uint8_t dport);
#ifdef SUPPORT_PRODUCTION_TEST_MODE
STATIC int ptest_ser(uint8_t dport);
#endif
#ifdef SUPPORT_UNIT_TEST
STATIC int utest_ser(uint8_t dport);
#endif
STATIC int ver_ser(uint8_t dport);

const COMMAND	Cmd_List[] =
{
	{ 1,   	"HELP",			2,		help_ser,			help_msg	},
	{ 1,  	"RESET",		1,		reset_ser,			reset_msg	},
	{ 1,  	"INITP",		2,		init_param_ser,		init_param_msg	},
	{ 1,  	"PARAM",		3,		param_ser,			param_msg	},
	{ 1,  	"RNV",			2,		read_nv_ser,		read_nv_msg	},
	{ 1,  	"WNV",			3,		write_nv_ser,		write_nv_msg},
	{ 1,  	"DIN",			1,		din_ser,			din_msg		},
	{ 1,  	"DOUT",			2,		dout_ser,			dout_msg	},
	{ 1,  	"AIN",			1,		ain_ser,			ain_msg		},
//	{ 1,  	"AOUT",			2,		aout_ser,			aout_msg	},
//	{ 1,  	"UIOT",			2,		uio_enable_ser,		uio_test_msg},
#ifdef SUPPORT_PASSWORD
	{ 1,  	"PASS",			2,		pass_ser,			pass_msg	},
	{ 1,  	"LOCK",			2,		lock_ser,			lock_msg	},
#endif
	{ 1,  	"TEST",			2,		test_ser,			test_msg	},
#ifdef SUPPORT_PRODUCTION_TEST_MODE
	{ 1,  	"PTEST",		2,		ptest_ser,			ptest_msg	},
#endif
#ifdef SUPPORT_UNIT_TEST	
	{ 1,  	"UTEST",		1,		utest_ser,			utest_msg	},
#endif	
	{ 1,  	"VER",			1,		ver_ser,			version_msg	},
	{ 0, 	(const char *)0,0,		0,			(const char **)0	}
};


uint8_t debugIoFlag=0;

int16_t test_run_stop_f=0;

int32_t table_dbg[PARAM_TABLE_SIZE];

int16_t dbg_warn_test=0;

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

extern int32_t mb_baudrate[];

extern int32_t i2c_rd_error;
extern int32_t i2c_wr_error;

//extern int32_t isMonitoring;
extern int16_t state_run_stop;
extern int16_t state_direction;
extern int16_t st_overload;
extern int16_t st_brake;

extern int16_t gear_ratio;
extern uint32_t motor_run_cnt;
extern uint32_t motor_run_hour;
extern uint32_t device_on_hour;
extern uint32_t device_min_cnt;
extern uint32_t run_minutes;
extern uint32_t motor_run_start_time;

extern uint8_t NFC_Access_flag;
extern uint32_t NFC_Access_cnt;

extern uint8_t mtd_pin;
extern uint8_t mdout_value[];
extern uint8_t mdin_value[];
extern uint16_t ain_val[];
extern uint32_t ain_sum;
//extern uint16_t adc_sample;
extern uint16_t adc_value;
extern volatile int8_t ADC_error;

//extern float freq_min, freq_max, V_ai_min, V_ai_max;

extern int32_t table_nvm[];
extern int32_t table_data[];
extern int32_t sys_data[];
extern Param_sys_t sys_table[];

extern uint8_t reset_cmd_send_f;
extern uint8_t reset_requested_f;

extern uint8_t param_init_requested_f;

#ifdef SUPPORT_TASK_WATCHDOG
extern uint8_t watchdog_f;
#endif

//extern void set_AO_duty(uint8_t bool_use, uint32_t in);
//extern void set_DO_duty(uint8_t bool_use, uint8_t in);
extern void EXT_DI_printStatus(void);
extern void EXT_DI_printConfig(void);
extern uint16_t EXT_AI_readADC(void);
extern int32_t EXT_AI_getFreq(uint16_t adc_val);
extern int32_t EXT_getAIValue(void);
//extern void EXT_AI_printConfig(void);

extern void MB_UART_init(uint32_t baudrate);
extern void MB_initTimer(int32_t b_index);
extern void UTIL_writeDout(uint8_t index, uint8_t onoff);
extern uint8_t NVM_clearInit(void); // to re-initialize EEPROM
extern uint16_t NVM_getSystemParamAddr(uint16_t index);
extern void NVM_setNfcMonitoring(void); // test only
extern int8_t NVM_isMonitoring(void);

extern int8_t main_SwReset(int flag);

extern void test_initDbgTableValue(void);
extern int8_t table_setValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern void test_setTableValue(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int32_t table_getInitValue(PARAM_IDX_t index);
extern uint16_t table_getAddr(PARAM_IDX_t index);
extern uint32_t table_calcCRC(void);
extern int8_t table_setCtrlIn(PARAM_IDX_t idx, int32_t value, int16_t option);
extern int8_t table_initializeBlankEEPROM(void);

#ifdef SUPPORT_PRODUCTION_TEST_MODE
extern uint8_t p_test_enabled;
#endif

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
extern void test_handleDinEmergency(void);
extern void test_handleDin(void);
extern void test_handleDout(void);
extern void test_getFreq(void);
extern void test_handleAin(void);
extern void test_handleDAin(void);
// table
extern void test_setFreqValue(void);
extern void test_setFreqRange(void);
extern void test_setValue(void);
extern void test_NvmAddr(void);
extern void test_ModbusAddr(void);

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
	int8_t status;
	int flag=0;

	flag = (int)atoi(arg_v[1]);

	kprintf(dport, "\r\n\r\n ---Reset--- flag=%d\r\n\r\n", flag);
	status = main_SwReset(flag);
	if(status==0)
		kputs(dport, "\r\n\r\n---Reset Error---\r\n\r\n");

	return 0;
}


STATIC int init_param_ser(uint8_t dport)
{
	uint8_t option=0;

	if(!table_isMotorStop())
	{
		kputs(dport, "\r\n\r\n--- please make sure motor is stopped");
		return -1;
	}

	option = (uint8_t)atoi(arg_v[1]);
	if(option > NVM_INIT_PARAM_TIME)
	{
		kputs(dport, "\r\n invalid value");
		return -1;
	}

	param_init_requested_f = option;

	kprintf(dport, "\r\n INITP parameter initialization option= %d", param_init_requested_f);

	return 0;
}

STATIC int param_ser(uint8_t dport)
{
	uint16_t index;
	int32_t l_val;
	int result, maj, min;

	if(arg_c != 2 && arg_c != 3)
	{
		maj = (int)((NVM_TABLE_VERSION&0xFF00)>>8);
		min = (int)(NVM_TABLE_VERSION&0xFF);
		kprintf(dport, "\r\n table version = %d.%d", maj, min);
		kprintf(dport, "\r\n Usage: PARAM index l_value");
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
		l_val = table_getValue(index);
		kprintf(dport, "\r\n index=%d value = %d", index, l_val);
	}
	else if(arg_c == 3)
	{
		l_val = (int32_t)atoi(arg_v[2]);
		result = table_runFunc(index, (int32_t)l_val, REQ_FROM_MODBUS);
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
	if(addr%4 != 0) // || addr > 0x3FF)
	{
		kprintf(dport, "\r\n address error %d", addr);
		return -1;
	}

	if(arg_c == 2)
	{
		i2c_status = I2C_readData((uint8_t *)&i2c_rvalue, addr, 4);
		kprintf(dport, "\r\n read EEPROM addr=%d, 0x%x, value=%d, status=%d", addr, addr, i2c_rvalue, i2c_status);
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
	if(addr%4 != 0) // || addr > 0x3FF)
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
	int i;

	if(arg_c != 1)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	for(i=0; i<20; i++) {UTIL_readDin(); osDelay(10);}

	EXT_DI_printConfig();
	EXT_DI_printStatus();

	return 0;
}

STATIC int dout_ser(uint8_t dport)
{
	uint8_t index;
	int32_t setting;
	int8_t status;

	if(arg_c == 2 || arg_c > 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		goto dout_err;
	}

	if(arg_c == 1) // show DOUT status
	{
		kprintf(dport, "\r\n show status Dout_config[0]=%d, [1]=%d", (int)table_getValue(multi_Dout_0_type), (int)table_getValue(multi_Dout_1_type));
		kprintf(dport, "\r\n show status Dout[0]=%d, [1]=%d, run_stop=%d", mdout_value[0], mdout_value[1], state_run_stop);
	}
	else // arg_c == 3
	{
		index = (uint8_t)atoi(arg_v[1]);
		if(index != 0 && index != 1) goto dout_err;

		setting = (int32_t)atoi(arg_v[2]);
		if(setting > DOUT_shaftbrake_on) goto dout_err;

		index = multi_Dout_0_type + index;
		status = table_setValue((PARAM_IDX_t)index, setting, REQ_FROM_TEST);
		kprintf(dport, "\r\n set Dout index=%d, value=%d, status=%d", index, setting, status);
	}

	return 0;

dout_err:
	kprintf(dport, "\r\n DOUT error");

	return 1;
}

STATIC int ain_ser(uint8_t dport)
{
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
		//kprintf(dport, "\r\n ADC val = %d %d ", ain_val[8],ain_val[9]);
		kprintf(dport, "\r\n ADC val = %d %d %d %d  %d %d %d %d", \
						ain_val[8],ain_val[9],ain_val[10],ain_val[11], ain_val[12],ain_val[13],ain_val[14],ain_val[15]);
		//kprintf(dport, "\r\n set Ain sum=%d, value=%d %d, V_val=%d, freq=%d", ain_sum, (int)adc_value, (int)adc_sample, EXT_getAIValue(), EXT_AI_getFreq(adc_value));
		kprintf(dport, "\r\n set Ain sum=%d, value=%d, V_val=%d, freq=%d", ain_sum, (int)adc_value, EXT_getAIValue(), EXT_AI_getFreq(adc_value));
	}
	return 0;
}


#ifdef SUPPORT_PASSWORD
extern int password_enabled;
STATIC int pass_ser(uint8_t dport)
{
	int32_t	new_pass, stored_pass;
	int8_t ret=0;

	if(arg_c == 1)
	{
		stored_pass = table_getValue(password_type);
		kprintf(dport, "\r\n password=%d, flag=%d", stored_pass, password_enabled);
		return -1;
	}
	else if(arg_c == 2)
	{
		new_pass = (int32_t)atoi(arg_v[1]);
		ret = table_runFunc(password_type, new_pass, REQ_FROM_MODBUS);
		if(ret == 0)
		{
			kprintf(dport, "\r\n password is not correct !!");
			return -1;
		}
		else
			kprintf(dport, "\r\n set password=%d enabled=%d", new_pass, password_enabled);
	}
	else
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	return 0;
}

STATIC int lock_ser(uint8_t dport)
{
	int32_t lock_val;
	int8_t ret=0;

	if(arg_c == 1)
	{
		lock_val = table_getValue(modify_lock_type);
		kprintf(dport, "\r\n lock=%d, flag=%d", lock_val, password_enabled);
		return -1;
	}
	else if(arg_c == 2)
	{
		lock_val = (int32_t)atoi(arg_v[1]);
		ret = table_runFunc(modify_lock_type, lock_val, REQ_FROM_MODBUS);
		if(ret == 0)
		{
			kprintf(dport, "\r\n password should be opened before change lock !!");
			return -1;
		}
		else
			kprintf(dport, "\r\n set lock=%d enabled=%d", lock_val, password_enabled);
	}
	else
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	return 0;
}
#endif

void test_DinConfig(void)
{
	int8_t t_status;
	uint8_t err_flag=0;

	// setup DI test,
	// DI0 : run/stop
	t_status = table_runFunc(multi_Din_0_type, (int32_t)DIN_run, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	// DI1 : Freq_low
	t_status = table_runFunc(multi_Din_1_type, (int32_t)DIN_freq_low, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	// DI2 : Freq_mid
	t_status = table_runFunc(multi_Din_2_type, (int32_t)DIN_freq_mid, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	// multi freq value setting: 20, 30, 40, 60Hz
	t_status = table_runFunc(multi_val_0_type, (int32_t)200, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	t_status = table_runFunc(multi_val_1_type, (int32_t)300, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	t_status = table_runFunc(multi_val_2_type, (int32_t)400, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	t_status = table_runFunc(multi_val_3_type, (int32_t)600, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	// set ctrl_in as DI control
	t_status = table_runFunc(ctrl_in_type, (int32_t)CTRL_IN_Digital, REQ_FROM_MODBUS);
	if(t_status == 0) err_flag++;

	kprintf(PORT_DEBUG, "\r\n setup DIN control  err=%d\r\n", err_flag);
}

STATIC int ver_ser(uint8_t dport)
{
	int maj, min;

	maj = (int)((NVM_TABLE_VERSION&0xFF00)>>8);
	min = (int)(NVM_TABLE_VERSION&0xFF);
	kprintf(dport, "\r\n table version = %d.%d", maj, min);
	kprintf(dport, "\r\n MCU v%d.%d", VERSION_MAJ, VERSION_MIN);

	return 0;
}

STATIC int test_ser(uint8_t dport)
{
	int idx;
	int32_t value;
	uint8_t status;
	int test_case, arg1;

	if(arg_c > 4)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		return -1;
	}

	test_case = atoi(arg_v[1]);
    if(test_case == 0)
    	test_case = arg_v[1][0];

    if(test_case == '0') // read EEPROM parameter
    {
//    	uint16_t addr;
//    	addr = (uint16_t)atoi(arg_v[2]);
//    	status = NVM_read(addr, &value);
//    	kprintf(dport, "\r\n NVM_read addr=%d, value=%d, status=%d", addr, (int)value, status);
    	idx = (int)atoi(arg_v[2]);
    	status = NVM_readParam((PARAM_IDX_t)idx, &value);
    	kprintf(dport, "\r\n NVM_readParam idx=%d, value=%d, status=%d", idx, (int)value, status);
    }
    else if(test_case == 1) // write parameter to EEPROM with CRC update
    {
//    	uint16_t addr;
//    	addr = (uint16_t)atoi(arg_v[2]);
//    	value = table_getInitValue(idx);
//    	status = NVM_write(addr, value);
//		kprintf(dport, "\r\n NVM_write addr=%d, value=%d status=%d", addr, value, status);

    	idx = (int)atoi(arg_v[2]);
    	value = table_getInitValue(idx);
    	status = NVM_writeParam((PARAM_IDX_t)idx, value);
    	kprintf(dport, "\r\n NVM_writeParam idx=%d, value=%d status=%d", idx, value, status);
    	NVM_setCRC();
    	kprintf(dport, "\r\n NVM_setCRC()");
    }
    else if(test_case == 2) // re-initialize EEPROM after reset
    {
    	table_initializeBlankEEPROM();
    	//NVM_clearInit();
		kprintf(dport, "\r\n re-initialize EEPROM, please reset...  \r\n");
    }
    else if(test_case == 3) // analog input test start
    {
    	int enable=0;

    	enable = (int)atoi(arg_v[2]);
    	if(enable)
    	{
    		test_setTableValue(ctrl_in_type, CTRL_IN_Analog_V, REQ_FROM_TEST);
    		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ain_val, EXT_AIN_SAMPLE_CNT);
    		kprintf(dport, "\r\n AIN test start");
    	}
    	else
    	{
    		test_setTableValue(ctrl_in_type, CTRL_IN_NFC, REQ_FROM_TEST);
    		kprintf(dport, "\r\n AIN test stop");
    	}
    }
    else if(test_case == 4) // set MCU error code to DSP  for testing
    {
    	int16_t value;

    	value = (int16_t)atoi(arg_v[2]);
    	switch(value)
    	{
    	case 0 :
    		dbg_warn_test = 0;
    		kputs(dport, "\r\n clear MCU WARNNING");
    		break;
    	case 1 :
    		dbg_warn_test = 1;
    		kputs(dport, "\r\n set MCU WARNNING TEST");
    		break;
    	case 2:
    		ERR_setErrorState(TRIP_REASON_MCU_ERR_TEST);
    		kputs(dport, "\r\n set MCU ERR TEST");
    		break;

    	case 3:
    		ERR_setErrorState(TRIP_REASON_MCU_COMM_FAIL);
    		kputs(dport, "\r\n set MCU COMM ERR TEST");
    		break;

    	case 4:
    		ERR_setErrorState(TRIP_REASON_NONE);
    		kputs(dport, "\r\n set ERR=0");
    		break;
    	}
    }
    else if(test_case == 5) // read table value
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
    else if(test_case == 6) // show error and counter, motor status ....
    {
		kprintf(dport, "\r\n error code=%d, i2c_rd_err=%d, i2c_wr_err=%d", (int)ERR_getErrorState(), i2c_rd_error, i2c_wr_error);
		kprintf(dport, "\r\n motor_on_cnt=%d, device_on_hour=%d, motor_run_hour=%d", (int)motor_run_cnt, (int)device_on_hour, (int)motor_run_hour);
		kprintf(dport, "\r\n monitor=%d, run_stop=%d", NVM_isMonitoring(), state_run_stop);
    }
    else if(test_case == 7) // change ctrl_in
    {
#if 1
    	uint32_t crtrl_in=0;
    	int8_t status=0;

    	crtrl_in = (int32_t)atoi(arg_v[2]);
    	status = table_setCtrlIn(ctrl_in_type, crtrl_in, REQ_FROM_MODBUS);
    	kprintf(dport, "\r\n set command crtrl_in = %d, status=%d", crtrl_in, status);

    	// for test motor run time, set dummy status value from DSP status
//    	test_run_stop_f = (int16_t)atoi(arg_v[2]);
//    	kprintf(dport, "\r\n set test run_state_f = %d", test_run_stop_f);
#else
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
#endif
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
    		kprintf(dport, "\r\n idx=%d, value=%d, data=%d, nvm=%d, status=%d, ", \
    				i, i2c_value, table_data[i], table_nvm[i], status);
    		osDelay(5);
    	}
    }
    else if(test_case == 9) // initialize modbus port with baudrate
    {
    	uint16_t b_index;
    	//uint32_t mb_baudrate[] = {2400, 4800, 9600, 19200, 38400, 115200};

    	b_index = (int)atoi(arg_v[2]);
   		MB_UART_init((uint32_t)b_index);
   		MB_initTimer((int32_t)b_index);
		kprintf(dport, "\r\n MB_address=%d, baudrate=%d", mb_slaveAddress, mb_baudrate[b_index]);
    }
    else if(test_case == 'A') // show Analog input setting
    {
    	kprintf(PORT_DEBUG, "\r\n AI setting f_min=%d, f_max=%d, V_min=%d, V_max=%d", \
    			table_data[v_in_min_freq_type], table_data[v_in_max_freq_type], table_data[v_in_min_type], table_data[v_in_max_type]);
    	//EXT_AI_printConfig();
    }
    else if(test_case == 'B') // 3 color LED test
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
    else if(test_case == 'C') // show EEPROM CRC value
    {
#if 0
    	uint32_t crc32_calc;

    	crc32_calc = table_calcCRC();
    	status = NVM_verifyCRC(crc32_calc);
#else
    	uint16_t crc16_calc;

    	crc16_calc = table_calcCRC();
    	status = NVM_verifyCRC(crc16_calc);
#endif
    	kprintf(dport, "\r\n verifyCRC status=%d", status);

    	arg1 = (int)atoi(arg_v[2]); // force update CRC
    	if(arg1 == 1)
    		NVM_setCRC();
    	else if(arg1 == 2) // clear CRC for test
    		status = NVM_write(sys_table[SYSTEM_PARAM_CRC_VALUE].addr, 0x0);
    }
    else if(test_case == 'D') // ask DSP error info
    {
    	uint16_t dummy[] = {0,0,0};
    	int8_t status;
    	int32_t err_code=0;

		status = COMM_sendMessage(SPICMD_REQ_ERR, dummy);
		if(status == COMM_SUCCESS)
		{
			err_code = table_getValue(err_code_1_type);
		}
		kprintf(dport, " read DSP error code=%d, dsp_err=%d", (int)err_code, (int)UTIL_isDspError());

    }
    else if(test_case == 'E') // show error data
    {
    	int i, idx[5] = {err_code_1_type, err_code_2_type, err_code_3_type, err_code_4_type, err_code_5_type};

    	kprintf(dport, " error code = %d, mtd_pin=%d", (int)ERR_getErrorState(), mtd_pin);
    	for(i=0; i<5; i++)
    	{
    		kprintf(dport, "\r\n [%d] err_code=%d, status=%d, current=%d, freq=%d", \
    				i+1, table_data[idx[i]], table_data[idx[i]+1], table_data[idx[i]+2], table_data[idx[i]+3]);
    	}
    }
    else if(test_case == 'F') // run parameter function of each parameter
    {
    	//idx = (int)atoi(arg_v[2]);
    	value = (int32_t)atoi(arg_v[2]);
    	status = table_runFunc(value_type, (int32_t)value, REQ_FROM_MODBUS);
    	kprintf(dport, "\r\n set command freq value=%d, status=%d, ", value, status);
    }
    else if(test_case == 'G') // test DOUT
    {
    	uint8_t index, onoff;

    	index = (uint8_t)atoi(arg_v[2]);
    	onoff = (uint8_t)atoi(arg_v[3]);

    	mdout_value[index] = onoff;
    	kprintf(dport, "\r\n set Dout index=%d, value=%d", index, onoff);
    }
    else if(test_case == 'I') // setting info
    {
    	// freq, ctrl_in
    	kprintf(dport, "\r\n freq=%d, ctrl_in=%d, baudrate=%d, motor_type=%d, gear_ratio=%d", \
    		table_data[value_type], table_data[ctrl_in_type], table_data[baudrate_type], table_data[motor_type_type], table_data[gear_ratio_type]);
    	kprintf(dport, "\r\n NFC_flag=%d, NFC_cnt=%d ", NFC_Access_flag, NFC_Access_cnt);

    }
    else if(test_case == 'L') // show table_data and table_nvm to compare
    {
    	int i;

    	for(i=0; i<PARAM_TABLE_SIZE; i++)
    	{
    		kprintf(dport, "\r\n %d, table_data=%d, table_nvm=%d %d", i, table_data[i], table_nvm[i], (table_data[i] == table_nvm[i]));
    	}
    }
    else if(test_case == 'N') // show sys_table list
    {
    	int i;
    	uint8_t status=0;
    	uint16_t addr=0;
    	int32_t value=0;

    	for(i=0; i<SYSTEM_PARAM_SIZE; i++)
    	{
    		addr = NVM_getSystemParamAddr(i);
    		status = NVM_read(addr, &value);
    		kprintf(dport, "\r\n sys_data[%d]=%d, addr=%d, nvm=%d, update=%d, status=%d", \
    				i, sys_data[i], sys_table[i].addr, value, NVM_isSysParamNeedUpdate(i), status);
    		//kprintf(dport, "\r\n update_index=%d", NVM_getSysParamUpdateIndex());
    	}
    }
    else if(test_case == 'Q') // verify EEPROM size
    {
    	uint8_t status, errflag=0;
    	uint16_t test_addr; // out of range
    	int32_t test_val = 0x55, value=0;

    	test_addr = (uint8_t)atoi(arg_v[2]);
    	if(test_addr%4 == 0)
    	{
			status = NVM_write(test_addr, test_val);
			if(status == 0) errflag++;

			status = NVM_read(test_addr, &value);
			if(status == 0 || value != test_val) errflag++;

			kprintf(PORT_DEBUG, "0: err=%d\r\n", errflag);
    	}
    	else
    		kprintf(PORT_DEBUG, "test_addr=%d  error\r\n", test_addr);
    }
    else if(test_case == 'R') // send Run/Stop command to DSP
    {
    	int8_t status;
    	uint16_t dummy[3] = {0,0,0};

    	if(state_run_stop == CMD_RUN)
    	{
    		status = COMM_sendMessage(SPICMD_CTRL_STOP, dummy);
    		kprintf(dport, "\r\n send SPICMD_CTRL_STOP, status=%d", status);
    	}
    	else
    	{
    		status = COMM_sendMessage(SPICMD_CTRL_RUN, dummy);
    		kprintf(dport, "\r\n send SPICMD_CTRL_RUN, status=%d", status);
    	}
    }
    else if(test_case == 'S') // show status info
    {
    	int i;

    	for(i=run_status1_type; i<PARAM_TABLE_SIZE; i++)
    	{
    		kprintf(dport, "\r\n %d, status value table_data=%d", i, table_data[i]);
    	}

		kprintf(dport, "\r\n status run_stop=%d, dir=%d", state_run_stop, state_direction);
    	kprintf(dport, "\r\n status overload=%d, brake=%d, gear_ratio=%d", st_overload, st_brake, gear_ratio);
    	kprintf(dport, "\r\n status di_val=%d, do_val=%d, ai_val=%d", EXT_getDIValue(), EXT_getDOValue(), EXT_getAIValue());
    	kprintf(dport, "\r\n dev_cnt=%d, run_count=%d on_hour=%d, run_hour=%d", \
    			device_min_cnt, motor_run_cnt, device_on_hour, motor_run_hour);
    	kprintf(dport, "\r\n r_start=%d, minutes=%d", motor_run_start_time, run_minutes);
    }
    else if(test_case == 'V') // show status in EEPROM
    {
    	uint8_t i, status=0;
    	uint16_t i2c_addr, i2c_len=4;
    	uint32_t i2c_value=0;

    	for(i=run_status1_type; i<PARAM_TABLE_SIZE; i++)
    	{
    		i2c_addr = table_getAddr((PARAM_IDX_t)i);
    		status = I2C_readData((uint8_t *)&i2c_value, i2c_addr, i2c_len);
    		kprintf(dport, "\r\n idx=%d, value=%d, nvm=%d, status=%d, ", i, i2c_value, table_nvm[i], status);
    		osDelay(5);
    	}
    }
    else if(test_case == 'W')
    {
    	int idx, errflag=0;
    	int8_t status=NVM_OK;
    	uint16_t addr;

    	table_dbg[model_type] = 3;
    	table_dbg[motor_type_type] = 3;
    	table_dbg[gear_ratio_type] = 120;

    	table_dbg[err_code_1_type] = 10;
    	table_dbg[err_status_1_type] = 4;
    	table_dbg[err_current_1_type] = 17;
    	table_dbg[err_freq_1_type] = 100;

    	table_dbg[err_code_2_type] = 2;
    	table_dbg[err_status_2_type] = 1;
    	table_dbg[err_current_2_type] = 19;
    	table_dbg[err_freq_2_type] = 200;

    	table_dbg[err_code_3_type] = 7;
    	table_dbg[err_status_3_type] = 2;
    	table_dbg[err_current_3_type] = 22;
    	table_dbg[err_freq_3_type] = 300;

    	table_dbg[err_code_4_type] = 9;
    	table_dbg[err_status_4_type] = 3;
    	table_dbg[err_current_4_type] = 21;
    	table_dbg[err_freq_4_type] = 400;

    	table_dbg[err_code_5_type] = 11;
    	table_dbg[err_status_5_type] = 4;
    	table_dbg[err_current_5_type] = 34;
    	table_dbg[err_freq_5_type] = 500;

    	table_dbg[run_status1_type] = 4;
    	table_dbg[run_status2_type] = 1;
    	table_dbg[I_rms_type] = 18;
    	table_dbg[run_freq_type] = 310;
    	table_dbg[dc_voltage_type] = 520;
    	table_dbg[torque_value_type] = 50;
    	table_dbg[torque_percent_type] = 95;
    	table_dbg[ipm_temperature_type] = 401;
    	table_dbg[mtr_temperature_type] = 3;
    	table_dbg[di_status_type] = 1;
    	table_dbg[do_status_type] = 2;
    	table_dbg[ai_status_type] = 3;
    	table_dbg[motor_on_cnt_type] = 20;
    	table_dbg[elapsed_hour_type] = 30;
    	table_dbg[operating_hour_type] = 50;

    	for(idx=model_type; idx<=operating_hour_type; idx++)
    	{
    		addr = table_getAddr((PARAM_IDX_t)idx);
    		status = NVM_write(addr, table_dbg[idx]); // clear all flag
    		if(status==NVM_NOK) errflag++;

#ifdef SUPPORT_TASK_WATCHDOG
    		watchdog_f= 0x1F; // WATCHDOG_ALL; to kick watchdog in case of no NFC B/D
#endif
    	}
    	kprintf(dport, "\r\n EEPROM write done err=%d", errflag);
    }
    else if(test_case == 'Y')
    {
    	uint8_t i, status=0;
    	uint16_t i2c_addr, i2c_len=4;
    	uint32_t i2c_value=0;

    	for(i=model_type; i<=operating_hour_type; i++)
    	{
			i2c_addr = table_getAddr((PARAM_IDX_t)i);
			status = I2C_readData((uint8_t *)&i2c_value, i2c_addr, i2c_len);
			kprintf(dport, "\r\n idx=%d, value=%d, dbg=%d, status=%d, ", i, i2c_value, table_dbg[i], status);
			osDelay(5);
#ifdef SUPPORT_TASK_WATCHDOG
    		watchdog_f= 0x1F; // WATCHDOG_ALL; to kick watchdog in case of no NFC B/D
#endif
    	}
    }
#ifdef SUPPORT_PASSWORD
    else if(test_case == 'U') // lock/unock with password
    {
    	int32_t lock, password=0;

    	if(arg_c == 2) // show password, lock status
    	{
    		password = table_getValue(password_type);
    		lock = table_getValue(modify_lock_type);
    		kprintf(dport, "\r\n password = %d, lock status = %d", password, lock);
    	}
    	else if(arg_c > 3) // >TEST U pass lock : set lock value with password
    	{
    		password = (int32_t)atoi(arg_v[2]);
    		lock = (int32_t)atoi(arg_v[3]);
    		if(password == table_getValue(password_type))
    		{
    			status = table_setValue(modify_lock_type, lock, REQ_FROM_MODBUS);
    			kprintf(dport, "\r\n set lock status = %d", lock);
    		}
    		else
    			kprintf(dport, "\r\n wrong password Error!!");
    	}
    }
#endif
    else if(test_case == 'T') // enable monitoring
    {
    	kprintf(dport, "\r\n set monitoring flag !!");
    	NVM_setNfcMonitoring();

    }
#ifdef SUPPORT_PRODUCTION_TEST_MODE
    else if(test_case == 'P') // Production test start
    {
    	p_test_enabled=1;

    	kprintf(dport, "\r\n production test start");
    }
#endif
#if 0
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

#ifdef SUPPORT_PRODUCTION_TEST_MODE
int8_t relay_test_state=0;
STATIC int ptest_ser(uint8_t dport)
{
	uint16_t test_cmd=0;
	int8_t status;

	if(arg_c > 3)
	{
		kprintf(dport, "\r\nInvalid number of parameters");
		goto ptest_err;
	}

	test_cmd = atoi(arg_v[1]);

	switch(test_cmd)
	{
	case 0:
		status = COMM_sendTestCmd(SPI_TEST_DSP_TEST_START);
		relay_test_state=1;
		kprintf(dport, "\r\n send SPI_TEST_DSP_TEST_START, status=%d", status);
		break;

	case 1:
		status = COMM_sendTestCmd(SPI_TEST_DSP_RELAY_OK);
		relay_test_state=2;
		kprintf(dport, "\r\n send SPI_TEST_DSP_RELAY_OK, status=%d", status);
		break;

	case 2:
		status = COMM_sendTestCmd(SPI_TEST_DSP_RELAY_NOK);
		relay_test_state=3;
		kprintf(dport, "\r\n send SPI_TEST_DSP_RELAY_NOK, status=%d", status);
		break;

	case 3:
		status = COMM_sendTestCmd(SPI_TEST_DSP_MOTOR_RUN);
		kprintf(dport, "\r\n send SPI_TEST_DSP_MOTOR_RUN, status=%d", status);
		break;
	}

	return 0;

ptest_err:
	kprintf(dport, "\r\n P_TEST error");

	return 1;
}
#endif


#ifdef SUPPORT_UNIT_TEST
STATIC int utest_ser(uint8_t dport)
{
    kputs(dport, "--Unit Test Running\r\n");

	UNITY_BEGIN();

#if 0
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
#endif

#if 0
	// ext_di
	RUN_TEST(test_setupMultiFuncDin);
	RUN_TEST(test_convertMultiStep);
	RUN_TEST(test_handleDinEmergency);
	RUN_TEST(test_handleDin);

	//ext_do
	RUN_TEST(test_handleDout);

	//ext_ai
	RUN_TEST(test_getFreq);
	RUN_TEST(test_handleAin);
#endif
	RUN_TEST(test_handleDAin);


#if 0
	// table
	RUN_TEST(test_NvmAddr);
	RUN_TEST(test_ModbusAddr);
	RUN_TEST(test_setFreqRange);
	RUN_TEST(test_setFreqValue);
	RUN_TEST(test_setValue);
#endif

#if 0
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
#endif

	UNITY_END();

	kputs(dport, " End of Unit Test!\r\n Reset Device!!\r\n");

	//UARTFlushTx(0);
	//while(1); // cause device reset by watchdog

	return 0;
}
#endif

static int display_BoardInfo(uint8_t dport)
{
	kputs  (dport, "\r\n*************************************************");
	kprintf(dport, "\r\n    Nara Inverter MCU v%d.%d Develop", VERSION_MAJ, VERSION_MIN);
	kputs  (dport, "\r\n*************************************************\r\n");

	return 0;
}

void debugTaskFunc(void const * argument)
{

  osDelay(2000);

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
