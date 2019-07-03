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

#include "drv_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
osMessageQId YSTC_eventQHandle;
osTimerId YstcTriggerTimerHandle;
osTimerId YstcUpdateTimerHandle;
osTimerId userIoTimerHandle;
osTimerId NfcAppTimerHandle;
osTimerId AccReadTimerHandle;
osSemaphoreId debugSemaphoreIdHandle;
osSemaphoreId rs485SemaphoreIdHandle;
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

/* USER CODE BEGIN PFP */

#define UIO_UPDATE_TIME_INTERVAL	10 // 10ms
#define ACC_READ_TIME_INTERVAL		1000 // 1sec

#define WATCHDOG_NFC		0x01
#define WATCHDOG_MAIN		0x02
#define WATCHDOG_USERIO		0x04
#define WATCHDOG_MODBUS		0x08
#define WATCHDOG_RS485		0x10
#define WATCHDOG_ALL		0x1F

uint32_t default_cnt=0, main_cnt=0, nfc_cnt=0, mbus_cnt=0, rs485_cnt=0, user_io_cnt=0;

extern uint16_t adc_value;

volatile int8_t ADC_ConvCpltFlag=0, ADC_error=0;
uint16_t ain_val[EXT_AIN_SAMPLE_CNT];
uint32_t ain_sum=0;
uint16_t exec_do_cnt=0;

#ifdef SUPPORT_TASK_WATCHDOG
uint8_t watchdog_f = 0;
#endif

extern void MB_init(void);
extern void MB_TaskFunction(void);

extern void debugTaskFunc(void const * argument);
//extern void rs485TaskFunction(void);
extern void kputs2(const char *pStr);
extern void kprintf2(const char *fmt, ...);


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
  HAL_TIM_Base_Start_IT(&htim3); // Analog Out timer
  HAL_TIM_Base_Start_IT(&htim10);
  printf("\r\n======== Started ==========");
  printf("\r\n** Compiled :    %4d/%02d/%02d   **\r\n\r\n ", BUILD_YEAR, BUILD_MONTH, BUILD_DAY);
 
  // LED on
  HAL_GPIO_WritePin(STATUS_MCU_GPIO_Port, STATUS_MCU_Pin, GPIO_PIN_SET);

  // ADC self calibration
  while(!(HAL_ADCEx_Calibration_Start(&hadc1)==HAL_OK));


  // TODO : initialize NVM


  HAL_IWDG_Refresh(&hiwdg); // kick


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of debugSemaphoreId */
  osSemaphoreDef(debugSemaphoreId);
  debugSemaphoreIdHandle = osSemaphoreCreate(osSemaphore(debugSemaphoreId), 1);

  /* definition and creation of rs485SemaphoreId */
  osSemaphoreDef(rs485SemaphoreId);
  rs485SemaphoreIdHandle = osSemaphoreCreate(osSemaphore(rs485SemaphoreId), 1);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(userIoTimerHandle, UIO_UPDATE_TIME_INTERVAL); // 10ms UserIo update
  osTimerStart(AccReadTimerHandle, ACC_READ_TIME_INTERVAL); // 1sec to read Accelerometer
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of YSTC_eventQ */
  osMessageQDef(YSTC_eventQ, 128, uint8_t);
  YSTC_eventQHandle = osMessageCreate(osMessageQ(YSTC_eventQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
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
  osThreadDef(mbus485Task, mbus485TaskFunc, osPriorityIdle, 0, 256);
  mbus485TaskHandle = osThreadCreate(osThread(mbus485Task), NULL);

  /* definition and creation of opt485Task */
  osThreadDef(opt485Task, opt485TaskFunc, osPriorityIdle, 0, 256);
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
	 osDelay(10);
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
  huart2.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOC, R_LED_Pin|G_LED_Pin|B_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_MCU_Pin|TH_MCU_1_Pin|TH_MCU_2_Pin|MCU_nSCS_OUT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DO_2_Pin R_LED_Pin G_LED_Pin B_LED_Pin 
                           DO_1_Pin */
  GPIO_InitStruct.Pin = DO_2_Pin|R_LED_Pin|G_LED_Pin|B_LED_Pin 
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
  osDelay(10);
  kputs(PORT_DEBUG, "start DefaultTask\r\n");
  /* Infinite loop */
  for(;;)
  {

#ifdef SUPPORT_TASK_WATCHDOG
	if((watchdog_f&WATCHDOG_ALL) == WATCHDOG_ALL)
	{
		watchdog_f = 0;
		HAL_IWDG_Refresh(&hiwdg); // kick
	}
#else
	HAL_IWDG_Refresh(&hiwdg); // kick
#endif

	default_cnt++;
    osDelay(100);
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

  // TODO : wait until EEPROM initialized
  //while(EEPROM_initialized_f==0);

  osDelay(10);
  kputs(PORT_DEBUG, "start NfcNvmTask\r\n");
  /* Infinite loop */
  for(;;)
  {
	  // read NFC tag flag

	  // if tagged, wait tag_end


	  // if tag end, update NVM -> table
	  //	check parameter table
	  //	check system parameter


	  // else if tag error, restore NVM <- table


	  // no tag state,
	  //	handle NVM update request



#ifdef SUPPORT_TASK_WATCHDOG
	  watchdog_f |= WATCHDOG_NFC;
#endif
	  osDelay(10);
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
  osDelay(200);
  kputs(PORT_DEBUG, "start userIoTask\r\n");
  /* Infinite loop */
  for(;;)
  {

#ifdef SUPPORT_TASK_WATCHDOG
	watchdog_f |= WATCHDOG_USERIO;
#endif
    osDelay(100);
  }
  /* USER CODE END userIoTaskFunc */
}

/* USER CODE BEGIN Header_mainHandlerTaskFunc */
/**
* @brief Function implementing the mainHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainHandlerTaskFunc */
void mainHandlerTaskFunc(void const * argument)
{
  /* USER CODE BEGIN mainHandlerTaskFunc */
  /* Infinite loop */
  osDelay(50);
  kputs(PORT_DEBUG, "YstcEvent task started\r\n");
  /* Infinite loop */
  for(;;)
  {


#ifdef SUPPORT_TASK_WATCHDOG
	watchdog_f |= WATCHDOG_MAIN;
#endif
	main_cnt++;
	osDelay(5);
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

	MB_init();
	osDelay(5);

	kputs(PORT_DEBUG, "rs485Task started\r\n");


	/* Infinite loop */
	for(;;)
	{
	  MB_TaskFunction();
	  //rs485TaskFunction();

	  mbus_cnt++;

#ifdef SUPPORT_TASK_WATCHDOG
	  watchdog_f |= WATCHDOG_MODBUS;
#endif
	  osDelay(5);
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
  /* Infinite loop */
  for(;;)
  {
	  //rs485TaskFunction();

	  rs485_cnt++;

#ifdef SUPPORT_TASK_WATCHDOG
	  watchdog_f |= WATCHDOG_RS485;
#endif
	  osDelay(5);
  }
  /* USER CODE END opt485TaskFunc */
}

/* YstcTriggerTimerCallback function */
void YstcTriggerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN YstcTriggerTimerCallback */
  
  /* USER CODE END YstcTriggerTimerCallback */
}

/* YstcUpdateTimerCallback function */
void YstcUpdateTimerCallback(void const * argument)
{
  /* USER CODE BEGIN YstcUpdateTimerCallback */
  
  /* USER CODE END YstcUpdateTimerCallback */
}

/* userIoTimerCallback function */
void userIoTimerCallback(void const * argument)
{
  /* USER CODE BEGIN userIoTimerCallback */
	int i;
	int32_t ctrl_in;

#ifndef SUPPORT_UNIT_TEST
	ctrl_in = table_getCtrllIn();
	if(ctrl_in == CTRL_IN_Digital) // UIO_UPDATE_TIME_INTERVAL 10ms
	{
		UTIL_readDin();
	}
	else if(ctrl_in == CTRL_IN_Analog_V)
	{
		// read ADC
		if(ADC_ConvCpltFlag)
		{

			for(i=0; i<EXT_AIN_SAMPLE_CNT; i++) ain_val[i] = (ain_val[i]&0x0FFF); // only 12bit available

			ain_sum=0;
			for(i=0; i<EXT_AIN_SAMPLE_CNT; i++) ain_sum += (uint32_t)ain_val[i];
			adc_value = (uint16_t)(ain_sum/EXT_AIN_SAMPLE_CNT);

			ADC_ConvCpltFlag=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ain_val, EXT_AIN_SAMPLE_CNT);
		}

	}
	else
		; // do nothing

#if 0
	if(exec_do_cnt%10 == 0) // 10ms * 10 cycle
	{
		EXT_DO_handleDout();
		if(exec_do_cnt >= 1000) exec_do_cnt=0;
	}
	exec_do_cnt++;
#endif


#endif
  /* USER CODE END userIoTimerCallback */
}

/* NfcAppTimerCallback function */
void NfcAppTimerCallback(void const * argument)
{
  /* USER CODE BEGIN NfcAppTimerCallback */
  
  /* USER CODE END NfcAppTimerCallback */
}

/* AccReadTimerCallback function */
void AccReadTimerCallback(void const * argument)
{
  /* USER CODE BEGIN AccReadTimerCallback */
  HAL_GPIO_TogglePin(STATUS_MCU_GPIO_Port, STATUS_MCU_Pin); // STATUS-LED toggle
  /* USER CODE END AccReadTimerCallback */
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
