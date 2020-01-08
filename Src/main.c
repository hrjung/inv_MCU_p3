/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "includes.h"
#include "proc_uart.h"
#include "build_defs.h"

#include "table.h"
#include "ext_io.h"
#include "error.h"
#include "dsp_comm.h"
#include "modbus_func.h"
#include "modbus_queue.h"
#include "nvm_queue.h"
#include "handler.h"

#include "drv_gpio.h"
#include "drv_nvm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum {
	MAIN_IDLE_STATE = 0,
	MAIN_DSP_STATE,
	MAIN_EXTIO_STATE,
	MAIN_MODBUS_STATE,
	MAIN_TABLE_STATE,
	MAIN_ERROR_STATE,
} MAIN_state_st;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId NfcNvmTaskHandle;
osThreadId userIoTaskHandle;
osThreadId mainHandlerTaskHandle;
osThreadId mbus485TaskHandle;
osThreadId opt485TaskHandle;
osMessageQId NFC_eventQHandle;
osTimerId YstcTriggerTimerHandle;
osTimerId YstcUpdateTimerHandle;
osTimerId userIoTimerHandle;
osTimerId NfcAppTimerHandle;
osTimerId AccReadTimerHandle;
osTimerId OperationTimerHandle;
osSemaphoreId debugSemaphoreIdHandle;
osSemaphoreId mbus485SemaphoreIdHandle;
osSemaphoreId rs485SemaphoreIdHandle;
osSemaphoreId I2CSemaphoreIdHandle;
/* USER CODE BEGIN PV */
osThreadId debugTaskHandle;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void const * argument);
void NfcNvmTaskFunc(void const * argument);
void userIoTaskFunc(void const * argument);
void mainHandlerTaskFunc(void const * argument);
void mbus485TaskFunc(void const * argument);
void opt485TaskFunc(void const * argument);
void YstcTriggerTimerCallback(void const * argument);
void YstcUpdateTimerCallback(void const * argument);
void userIoTimerCallback(void const * argument);
void NfcAppTimerCallback(void const * argument);
void AccReadTimerCallback(void const * argument);
void OperationTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */

// timer setup
#define UIO_UPDATE_TIME_INTERVAL	10 // 10ms
#define ACC_READ_TIME_INTERVAL		1000 // 1sec
#define DSP_STATUS_TIME_INTERVAL	1000 // 1sec
#define OPERATION_TIME_INTERVAL		60000 // 1 min

#define WATCHDOG_NFC		0x01
#define WATCHDOG_MAIN		0x02
#define WATCHDOG_USERIO		0x04
#define WATCHDOG_MODBUS		0x08
#define WATCHDOG_RS485		0x10
#define WATCHDOG_ALL		0x1F

uint32_t default_cnt=0, main_cnt=0, nfc_cnt=0, mbus_cnt=0, rs485_cnt=0, user_io_cnt=0;

uint8_t main_handler_f=0;

uint8_t NFC_Access_flag=0;
uint8_t DSP_status_read_flag=0;
uint8_t reset_cmd_send_f = 0;

volatile int8_t ADC_ConvCpltFlag=0, ADC_error=0;

uint16_t exec_do_cnt=0;

uint16_t user_io_handle_cnt=0;
uint8_t user_io_handle_f = 0;

// device working hour
uint32_t motor_run_hour=0;
uint32_t device_on_hour=0;
uint32_t run_minutes=0;

uint32_t timer_100ms=0;
uint32_t device_min_cnt=0;
uint32_t dev_start_time=0;


#ifdef SUPPORT_TASK_WATCHDOG
uint8_t watchdog_f = 0;
#endif

extern LED_status_t LED_state[]; // 3 color LED state

extern uint8_t reset_enabled_f;
//extern uint8_t reset_requested_f;

extern uint8_t param_init_requested_f;

extern uint16_t jig_test_enabled_f;

extern void gen_crc_table(void);
extern void MB_init(void);
extern void MB_TaskFunction(void);
extern int8_t MB_handlePacket(void);

extern int8_t NVM_setMotorRunCount(uint32_t run_count);
extern int8_t COMM_sendMotorType(void);

// opt485_task.c
extern void OPT_init(void);
extern void OPT_TaskFunction(void);

#ifdef SUPPORT_PRODUCTION_TEST_MODE
extern int JIG_isTestEnabled(void);
extern void JIG_test_state(void);
#endif

extern void debugTaskFunc(void const * argument);
//extern void rs485TaskFunction(void);
//extern void kputs2(const char *pStr);
//extern void kprintf2(const char *fmt, ...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *data, int len)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);

	return (status == HAL_OK ? len : 0);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		osSemaphoreRelease(debugSemaphoreIdHandle);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		uart_errCnt[0]++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADC_ConvCpltFlag = 1;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	ADC_error=1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //int8_t status=0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  initUarts(); // debug UART
  HAL_TIM_Base_Start_IT(&htim3); // Analog Out timer, no used for P3
  HAL_TIM_Base_Start_IT(&htim10);

  printf("\r\n======== MCU Started ==========");
  printf("\r\n** Compiled :    %4d/%02d/%02d   **\r\n\r\n ", BUILD_YEAR, BUILD_MONTH, BUILD_DAY);
 
  // LED on
  HAL_GPIO_WritePin(STATUS_MCU_GPIO_Port, STATUS_MCU_Pin, GPIO_PIN_SET);

  UTIL_setMTDpin(0); // set MTD pin no error

  // ADC self calibration
  while(!(HAL_ADCEx_Calibration_Start(&hadc1)==HAL_OK));


  gen_crc_table();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of debugSemaphoreId */
  osSemaphoreDef(debugSemaphoreId);
  debugSemaphoreIdHandle = osSemaphoreCreate(osSemaphore(debugSemaphoreId), 1);

  /* definition and creation of mbus485SemaphoreId */
  osSemaphoreDef(mbus485SemaphoreId);
  mbus485SemaphoreIdHandle = osSemaphoreCreate(osSemaphore(mbus485SemaphoreId), 1);

  /* definition and creation of rs485SemaphoreId */
  osSemaphoreDef(rs485SemaphoreId);
  rs485SemaphoreIdHandle = osSemaphoreCreate(osSemaphore(rs485SemaphoreId), 1);

  /* definition and creation of I2CSemaphoreId */
  osSemaphoreDef(I2CSemaphoreId);
  I2CSemaphoreIdHandle = osSemaphoreCreate(osSemaphore(I2CSemaphoreId), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of YstcTriggerTimer */
  osTimerDef(YstcTriggerTimer, YstcTriggerTimerCallback);
  YstcTriggerTimerHandle = osTimerCreate(osTimer(YstcTriggerTimer), osTimerPeriodic, NULL);

  /* definition and creation of YstcUpdateTimer */
  osTimerDef(YstcUpdateTimer, YstcUpdateTimerCallback);
  YstcUpdateTimerHandle = osTimerCreate(osTimer(YstcUpdateTimer), osTimerPeriodic, NULL);

  /* definition and creation of userIoTimer */
  osTimerDef(userIoTimer, userIoTimerCallback);
  userIoTimerHandle = osTimerCreate(osTimer(userIoTimer), osTimerPeriodic, NULL);

  /* definition and creation of NfcAppTimer */
  osTimerDef(NfcAppTimer, NfcAppTimerCallback);
  NfcAppTimerHandle = osTimerCreate(osTimer(NfcAppTimer), osTimerOnce, NULL);

  /* definition and creation of AccReadTimer */
  osTimerDef(AccReadTimer, AccReadTimerCallback);
  AccReadTimerHandle = osTimerCreate(osTimer(AccReadTimer), osTimerPeriodic, NULL);

  /* definition and creation of OperationTimer */
  osTimerDef(OperationTimer, OperationTimerCallback);
  OperationTimerHandle = osTimerCreate(osTimer(OperationTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(userIoTimerHandle, UIO_UPDATE_TIME_INTERVAL); // 10ms UserIo update
  osTimerStart(AccReadTimerHandle, ACC_READ_TIME_INTERVAL); // 1 sec to read Accelerometer
  osTimerStart(YstcTriggerTimerHandle, 100); //100ms

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of NFC_eventQ */
  osMessageQDef(NFC_eventQ, 128, uint8_t);
  NFC_eventQHandle = osMessageCreate(osMessageQ(NFC_eventQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of NfcNvmTask */
  osThreadDef(NfcNvmTask, NfcNvmTaskFunc, osPriorityNormal, 0, 256);
  NfcNvmTaskHandle = osThreadCreate(osThread(NfcNvmTask), NULL);

  /* definition and creation of userIoTask */
  osThreadDef(userIoTask, userIoTaskFunc, osPriorityBelowNormal, 0, 128);
  userIoTaskHandle = osThreadCreate(osThread(userIoTask), NULL);

  /* definition and creation of mainHandlerTask */
  osThreadDef(mainHandlerTask, mainHandlerTaskFunc, osPriorityNormal, 0, 512);
  mainHandlerTaskHandle = osThreadCreate(osThread(mainHandlerTask), NULL);

  /* definition and creation of mbus485Task */
  osThreadDef(mbus485Task, mbus485TaskFunc, osPriorityNormal, 0, 256);
  mbus485TaskHandle = osThreadCreate(osThread(mbus485Task), NULL);

  /* definition and creation of opt485Task */
  osThreadDef(opt485Task, opt485TaskFunc, osPriorityBelowNormal, 0, 256);
  opt485TaskHandle = osThreadCreate(osThread(opt485Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(debugTask, debugTaskFunc, osPriorityLow, 0, 512);
  debugTaskHandle = osThreadCreate(osThread(debugTask), NULL);


  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 osDelay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
#ifndef SUPPORT_TASK_WATCHDOG
	return ; // not use watchdog in unit test
#endif

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 2;
  sTime.Minutes = 37;
  sTime.Seconds = 10;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 7;
  DateToUpdate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  // 50us timer for modbus
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 64;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 64;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 49;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DO_2_Pin|DO_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AUX_OUT_Pin|Modbus_RTS_Pin|MTD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R_LED_Pin|B_LED_Pin|G_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_MCU_Pin|TH_MCU_1_Pin|TH_MCU_2_Pin|MCU_nSCS_OUT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DO_2_Pin R_LED_Pin B_LED_Pin G_LED_Pin 
                           DO_1_Pin */
  GPIO_InitStruct.Pin = DO_2_Pin|R_LED_Pin|B_LED_Pin|G_LED_Pin 
                          |DO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_2_Pin DI_3_Pin DI_1_Pin */
  GPIO_InitStruct.Pin = DI_2_Pin|DI_3_Pin|DI_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DTM1_Pin DTM2_Pin */
  GPIO_InitStruct.Pin = DTM1_Pin|DTM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_OUT_Pin */
  GPIO_InitStruct.Pin = AUX_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Modbus_RTS_Pin */
  GPIO_InitStruct.Pin = Modbus_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Modbus_RTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_MCU_Pin TH_MCU_1_Pin TH_MCU_2_Pin MCU_nSCS_OUT3_Pin */
  GPIO_InitStruct.Pin = STATUS_MCU_Pin|TH_MCU_1_Pin|TH_MCU_2_Pin|MCU_nSCS_OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_WIP_Pin */
  GPIO_InitStruct.Pin = RF_WIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_WIP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MTD1_Pin */
  GPIO_InitStruct.Pin = MTD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MTD1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void main_kickWatchdogNFC(void)
{
	watchdog_f |= WATCHDOG_NFC;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  osDelay(800);
  kputs(PORT_DEBUG, "start DefaultTask\r\n");
  /* Infinite loop */
  for(;;)
  {

#ifdef SUPPORT_TASK_WATCHDOG
	if(main_handler_f == 0)
	{
		HAL_IWDG_Refresh(&hiwdg); // kick
	}
	else
	{
		if((watchdog_f&WATCHDOG_ALL) == WATCHDOG_ALL)
		{
			watchdog_f = 0;
			HAL_IWDG_Refresh(&hiwdg); // kick
		}
	}
#endif

	default_cnt++;
    osDelay(50);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_NfcNvmTaskFunc */
/**
* @brief Function implementing the NfcNvmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NfcNvmTaskFunc */
void NfcNvmTaskFunc(void const * argument)
{
  /* USER CODE BEGIN NfcNvmTaskFunc */
  int sys_index=SYSTEM_PARAM_SIZE;
  int32_t tag_end=0;
  int8_t status, nvm_backup=0;
  static uint32_t prev_time_tick;
  static uint32_t bk_cnt=0;

  // wait until mainHandler initialized
  while(main_handler_f==0) osDelay(1);

  osDelay(10);
  kputs(PORT_DEBUG, "start NfcNvmTask\r\n");
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
#ifdef SUPPORT_TASK_WATCHDOG
	  watchdog_f |= WATCHDOG_NFC;
#endif

#ifndef SUPPORT_UNIT_TEST
	  // read NFC tag flag
	  tag_end=0;
	  status = NVM_getNfcStatus(&tag_end);
	  if(status == 0) {kputs(PORT_DEBUG, "nfc tag error\r\n"); continue;}

	  if(NFC_Access_flag)
	  {
		  UTIL_setLED(LED_COLOR_B, 0);
		  osDelay(10);
		  continue; // it can skip below EEPROM access until NFC untagged
	  }
//	  else
//		  UTIL_setLED(LED_COLOR_G, 0);

	  // if tagged, wait tag_end
	  if(tag_end == 1)
	  {
		  // if tag end, update NVM -> table
		  //	check parameter table
		  //	check system parameter
		  osDelay(5);
		  if(NVM_isNfcMonitoring())
		  {
			  UTIL_setLED(LED_COLOR_B, 0);
		  }
		  else
		  {
#ifdef SUPPORT_PASSWORD
			  if(table_isLocked()) // in case of lock, parameter should be restored
			  {
				  osDelay(5);
				  status = HDLR_restoreNVM();
				  if(status == 0) {kputs(PORT_DEBUG, "locked!! nfc tag restore error\r\n"); }

				  UTIL_setLED(LED_COLOR_G, 0);
			  }
			  else
			  {
#endif
				  status = HDLR_updatebyNfc(); // EEPROM updated -> table update
				  if(status == 0)
				  {
					  kputs(PORT_DEBUG, "nfc tag update error\r\n");
					  UTIL_setLED(LED_COLOR_B, 1);
				  }
				  else
					  UTIL_setLED(LED_COLOR_G, 0);
			  }

			  kputs(PORT_DEBUG, "nfc tag processed!\r\n");
		  }
	  }
	  else if(tag_end != 0 && tag_end != 1) // NFC write is incomplete -> restore
	  {
		  // tag error : restore NVM <- table
		  osDelay(5);
		  status = HDLR_restoreNVM();
		  if(status == 0) {kputs(PORT_DEBUG, "nfc tag restore error\r\n"); }

		  kputs(PORT_DEBUG, "nfc tag imcomplete!\r\n");

		  UTIL_setLED(LED_COLOR_G, 0);
	  }
	  else
		  UTIL_setLED(LED_COLOR_G, 0);

	  // no tag state,
	  //	handle NVM update request
	  if(!NVMQ_isEmptyNfcQ())
	  {
		  status = HDLR_updateParamNVM();
		  if(status == 0) kputs(PORT_DEBUG, "HDLR_updateParamNVM ERROR\r\n");

		  UTIL_setLED(LED_COLOR_G, 0);
	  }

	  if(ERR_getErrorState() == TRIP_REASON_MCU_INIT) continue;

	  // update system_parameter to NVM
	  sys_index = NVM_getSysParamUpdateIndex();
	  if(sys_index != SYSTEM_PARAM_SIZE)
	  {
		  int8_t status=0;

		  status = HDLR_updateSysParam(sys_index);
		  if(status == 0) kprintf(PORT_DEBUG, "HDLR_updateSysParam index=%d ERROR\r\n", sys_index);

		  UTIL_setLED(LED_COLOR_G, 0);
	  }


	  bk_cnt++;
	  if(bk_cnt%50 == 0) // every 500ms
	  {
#ifdef SUPPORT_INIT_PARAM
		  if(param_init_requested_f || NVM_isInitNvmNfc()) // need NVM initialize ?
		  {
			  UTIL_setLED(LED_COLOR_B, 0);

			  status = HDLR_initNVM();
			  if(status == 0)
				  kputs(PORT_DEBUG, "HDLR_initNVM ERROR\r\n");
			  else
			  {
				  NVM_clearInitParamCmd();
				  param_init_requested_f = 0;
			  }

			  UTIL_setLED(LED_COLOR_G, 0);
		  }
#endif

#ifdef SUPPORT_PARAMETER_BACKUP
		  if(HDLR_isBackupEnabled())
		  {
			  nvm_backup = HDLR_getBackupFlag();
			  kprintf(PORT_DEBUG, "HDLR_backupParameter cmd=%d\r\n", nvm_backup);
			  if(nvm_backup == MB_BACKUP_SAVE) // backup
			  {
				  status = HDLR_backupParameter();
				  if(status == 0) kputs(PORT_DEBUG, "HDLR_backupParameter ERROR\r\n");
			  }
			  else if(nvm_backup == MB_BACKUP_RESTORE && HDLR_isBackupAvailable()) // restore
			  {
				  status = HDLR_restoreParameter(); // restore NVM
				  if(status == 0) kputs(PORT_DEBUG, "HDLR_restoreParameter ERROR\r\n");

				  NVM_setCRC(); // set new CRC
			  }

			  UTIL_setLED(LED_COLOR_G, 0);
			  HDLR_clearBackupFlagModbus(); // clear flag
		  }
#endif
	  }

	  // time info update
	  if(prev_time_tick != device_min_cnt) // every minute
	  {
		  status = HDLR_updateTime(device_min_cnt);
		  if(status == 0) kputs(PORT_DEBUG, "HDLR_updateTime ERROR\r\n");

		  prev_time_tick = device_min_cnt;
	  }

#endif

  }
  /* USER CODE END NfcNvmTaskFunc */
}

/* USER CODE BEGIN Header_userIoTaskFunc */
/**
* @brief Function implementing the userIoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_userIoTaskFunc */
void userIoTaskFunc(void const * argument)
{
  /* USER CODE BEGIN userIoTaskFunc */

  while(main_handler_f == 0) osDelay(5);

  osDelay(100);
  kputs(PORT_DEBUG, "start userIoTask\r\n");
  /* Infinite loop */
  for(;;)
  {

#ifdef SUPPORT_TASK_WATCHDOG
	watchdog_f |= WATCHDOG_USERIO;
#endif
    osDelay(50);
  }
  /* USER CODE END userIoTaskFunc */
}

/* USER CODE BEGIN Header_mainHandlerTaskFunc */

int8_t main_SwReset(void)
{
	int8_t status;
	reset_cmd_send_f = 1;

	status = COMM_sendTestCmd(SPI_TEST_CMD_RESET);

	if(status)
	{
		osDelay(1000); // wait DSP reset
		HAL_NVIC_SystemReset();
	}

	return status;
}


int8_t mainHandlerState(void)
{
	static MAIN_state_st sct_state = MAIN_IDLE_STATE; // initial state
	int8_t status;
	int32_t ctrl_in;
	static uint16_t event_cnt=0;

	switch(sct_state)
	{
	case MAIN_IDLE_STATE:
		//if(ERR_isErrorState()) sct_state = MAIN_ERROR_STATE;
		//else
		if(DSP_status_read_flag && reset_cmd_send_f==0) sct_state = MAIN_DSP_STATE;
		else if(user_io_handle_f && ERR_isErrorState()==0) sct_state = MAIN_EXTIO_STATE;
		else
		{
			event_cnt++;
			if(event_cnt%2 == 0) sct_state = MAIN_MODBUS_STATE;
			else sct_state = MAIN_TABLE_STATE;
		}
		break;

	case MAIN_DSP_STATE:
		// read DSP error flag and status info
		HDLR_handleDspError();

		HDLR_readDspStatus();

		DSP_status_read_flag = 0; // clear flag

		sct_state = MAIN_IDLE_STATE;

		break;

	case MAIN_EXTIO_STATE:
		if(ERR_isErrorState()) {sct_state = MAIN_IDLE_STATE; break;}

		EXI_DI_handleEmergency(); // handle trip/emergency DI

		ctrl_in = table_getCtrllIn();

		if(ctrl_in != CTRL_IN_NFC) // if run/stop from NFC, restore to idle
		{
			HDLR_restoreRunStopFlagNFC();
		}

		switch(ctrl_in)
		{
		case CTRL_IN_NFC:
			status = HDLR_handleRunStopFlagNFC();
			break;

		case CTRL_IN_Digital:
			status = EXI_DI_handleDin(ctrl_in);
			break;

		case CTRL_IN_Analog_V:
			status = EXT_AI_handleAin();
			break;

#ifdef SUPPORT_DI_AI_CONTROL
		case CTRL_IN_Din_Ain:
			status = EXT_handleDAin(ctrl_in);
			break;
#endif

		case CTRL_IN_Modbus: // only run/stop command
			status = HDLR_handleRunStopFlagModbus();
			break;
		}
		user_io_handle_f = 0;
		sct_state = MAIN_IDLE_STATE;

		break;

	case MAIN_MODBUS_STATE:
		MB_handlePacket();
		sct_state = MAIN_IDLE_STATE;
		break;

	case MAIN_TABLE_STATE:
		if(!NVMQ_isEmptyTableQ())
		{
		  status = table_updatebyTableQ();
		  if(status == 0) kputs(PORT_DEBUG, "table_updatebyTableQ ERROR\r\n");
		}

		if(reset_enabled_f) main_SwReset(); // SW reset

		sct_state = MAIN_IDLE_STATE;
		break;

	case MAIN_ERROR_STATE:

		break;
	}

	return 1;
}


/**
* @brief Function implementing the mainHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainHandlerTaskFunc */
void mainHandlerTaskFunc(void const * argument)
{
  /* USER CODE BEGIN mainHandlerTaskFunc */
  int8_t com_status, nv_status;
//	uint16_t addr=408;
//	int32_t i2c_rvalue=0;
//	uint8_t i2c_status;

  osDelay(10);
  kputs(PORT_DEBUG, "start mainHandler task\r\n");

#ifdef SUPPORT_PRODUCTION_TEST_MODE
  jig_test_enabled_f = JIG_isTestEnabled();
  if(jig_test_enabled_f)
  {
	  while(1)
	  {
		  JIG_test_state();
		  HAL_IWDG_Refresh(&hiwdg);
		  osDelay(10);
	  }
  }
#endif

//	i2c_status = I2C_readData((uint8_t *)&i2c_rvalue, addr, 4);
//	kprintf(PORT_DEBUG, "read EEPROM addr=%d, value=%d, status=%d", addr, i2c_rvalue, i2c_status);

  UTIL_setLED(LED_COLOR_G, 0);
#ifndef SUPPORT_UNIT_TEST
  nv_status = table_initNVM();
  if(nv_status == 0)
  {
	  kprintf(PORT_DEBUG, "nv_status = %d error!\r\n", nv_status);
	  nv_status = table_initNVM(); // try again
  }

  if(nv_status == 0)
	  ERR_setErrorState(TRIP_REASON_MCU_INIT);

  com_status = COMM_sendMotorType();
  if(com_status == 0)
	  ERR_setErrorState(TRIP_REASON_MCU_COMM_FAIL);

  kprintf(PORT_DEBUG, "nv_status = %d, com_status=%d\r\n", nv_status, com_status);
#endif

  // init queue
  MBQ_init();
  NVMQ_init();

  main_handler_f = 1;

  //TODO : hrjung init parameters setting for ctrl_in
  table_initParam();

//  nv_status = NVM_clearRunStopFlag(); // set idle flag
//  if(nv_status == 0)
//  {
//	  kprintf(PORT_DEBUG, "NFC idle flag set error=%d\r\n", nv_status);
//  }

  osTimerStart(OperationTimerHandle, OPERATION_TIME_INTERVAL); // 1 min to inverter operation time
  osTimerStart(YstcUpdateTimerHandle, DSP_STATUS_TIME_INTERVAL); // 1 sec read DSP status

  /* Infinite loop */
  for(;;)
  {

#ifdef SUPPORT_TASK_WATCHDOG
	watchdog_f |= WATCHDOG_MAIN;
#endif
	main_cnt++;
	osDelay(5);

#ifndef SUPPORT_UNIT_TEST
	mainHandlerState();
#endif

  }
  /* USER CODE END mainHandlerTaskFunc */
}

/* USER CODE BEGIN Header_mbus485TaskFunc */

/**
* @brief Function implementing the mbus485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mbus485TaskFunc */
void mbus485TaskFunc(void const * argument)
{
  /* USER CODE BEGIN mbus485TaskFunc */

	while(main_handler_f == 0) osDelay(5);

	osDelay(80);

	MB_init();

	// init modbus_handler
	MB_initAddrMap();

	kputs(PORT_DEBUG, "mbus485Task started\r\n");


	/* Infinite loop */
	for(;;)
	{
#ifndef SUPPORT_UNIT_TEST
		MB_TaskFunction();
#endif

		mbus_cnt++;

#ifdef SUPPORT_TASK_WATCHDOG
		watchdog_f |= WATCHDOG_MODBUS;
#endif
		osDelay(10);
	}
  /* USER CODE END mbus485TaskFunc */
}

/* USER CODE BEGIN Header_opt485TaskFunc */
/**
* @brief Function implementing the opt485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_opt485TaskFunc */
void opt485TaskFunc(void const * argument)
{
  /* USER CODE BEGIN opt485TaskFunc */
  while(main_handler_f == 0) osDelay(5);

  osDelay(100);

  OPT_init();

  kputs(PORT_DEBUG, "opt485Task started\r\n");
  /* Infinite loop */
  for(;;)
  {
	  OPT_TaskFunction();

	  rs485_cnt++;

#ifdef SUPPORT_TASK_WATCHDOG
	  watchdog_f |= WATCHDOG_RS485;
#endif
	  osDelay(10);
  }
  /* USER CODE END opt485TaskFunc */
}

/* YstcTriggerTimerCallback function */
void YstcTriggerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN YstcTriggerTimerCallback */
  timer_100ms++;
  /* USER CODE END YstcTriggerTimerCallback */
}

/* YstcUpdateTimerCallback function */
void YstcUpdateTimerCallback(void const * argument)
{
  /* USER CODE BEGIN YstcUpdateTimerCallback */
	DSP_status_read_flag = 1;

	HAL_GPIO_TogglePin(STATUS_MCU_GPIO_Port, STATUS_MCU_Pin); // STATUS-LED toggle
  /* USER CODE END YstcUpdateTimerCallback */
}

/* userIoTimerCallback function */
void userIoTimerCallback(void const * argument)
{
  /* USER CODE BEGIN userIoTimerCallback */
	int32_t ctrl_in;

#ifndef SUPPORT_UNIT_TEST
	// set DTM pin to check DSP error
	UTIL_readDspErrorPin();

	UTIL_readDin();

	ctrl_in = table_getCtrllIn();

	if(ctrl_in == CTRL_IN_Analog_V
#ifdef SUPPORT_DI_AI_CONTROL
		|| ctrl_in == CTRL_IN_Din_Ain
#endif
	)
	{
		// read ADC
		if(ADC_ConvCpltFlag)
		{
			UTIL_getAdcSamples();

			ADC_ConvCpltFlag=0;
			UTIL_startADC();
		}
	}
	else
		; // do nothing



	if(exec_do_cnt%10 == 0) // 10ms * 10 cycle -> 100ms
	{
		EXT_DO_handleDout();
		if(exec_do_cnt >= 1000) exec_do_cnt=0;
	}
	exec_do_cnt++;


	if(user_io_handle_cnt++ > 20) // 200 ms, set flag for EXT IO handler
	{
		user_io_handle_cnt=0;
		user_io_handle_f = 1;
	}
#endif
  /* USER CODE END userIoTimerCallback */
}

/* NfcAppTimerCallback function */
void NfcAppTimerCallback(void const * argument)
{
  /* USER CODE BEGIN NfcAppTimerCallback */
	NFC_Access_flag = 0; // NFC access end

  /* USER CODE END NfcAppTimerCallback */
}

/* AccReadTimerCallback function */
void AccReadTimerCallback(void const * argument)
{
  /* USER CODE BEGIN AccReadTimerCallback */

  UTIL_handleLED(); // handling 3 color LED

  //HAL_GPIO_TogglePin(STATUS_MCU_GPIO_Port, STATUS_MCU_Pin); // STATUS-LED toggle
  /* USER CODE END AccReadTimerCallback */
}

/* OperationTimerCallback function */
void OperationTimerCallback(void const * argument)
{
  /* USER CODE BEGIN OperationTimerCallback */
	device_min_cnt++;

  /* USER CODE END OperationTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  ;
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
