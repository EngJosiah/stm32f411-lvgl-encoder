/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "../ILI9341/XPT2046_touch.h"
#include "../ILI9341/ILI9341_STM32_Driver.h"
#include "../lvgl/lvgl.h"
#include "../lvgl/lv_conf.h"
#include "stdio.h"
#include "../GSM/gsm.h"
#include "../GPS/gps.h"
#include "../FLASH/flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint16_t x;
	uint16_t y;
	uint8_t touched;
}tp_t;

tp_t TP;

typedef struct{
	char tx_buffer[256];//tosend over uart
	char queue_data[256];// queue for uart
	char rx_data[64];
	char rx_buffer[64];
	uint8_t busy;
	uint8_t new_tx_data;
	uint8_t new_rx_data;
} rs485_t;

rs485_t rs_port;

typedef struct{
	bool started;
	bool stopped;
	bool onStart;
	bool onStop;
	uint32_t start_time;
	uint32_t stop_time;
	uint32_t active_time;
}timing_vending_t;

typedef struct{
	float s_price, s_litres, s_grams;
	float d_litres, d_price, d_grams;
	float offset, kg;
	bool pause;
	uint32_t d_timer;
}dispenser_t;


timing_vending_t my_timer;
dispenser_t my_dispenser;

extern settings_t settings;
extern gsm_t gsm;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define top_bar_color_1 0x4b5320
#define top_bar_color_2 0x78866b
#define top_bar_color_3 0xbdb76b
#define top_bar_color_4 0x555d50
#define top_bar_color_5 0x18453b

#define main_body_color_1  0xf8c058
#define main_body_color_2  0xcd722d
#define main_body_color_3  0xae0d13
#define main_body_color_4  0x5c310d
#define main_body_color_5  0x875a28
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timerTask */
osThreadId_t timerTaskHandle;
const osThreadAttr_t timerTask_attributes = {
  .name = "timerTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gsmTask */
osThreadId_t gsmTaskHandle;
const osThreadAttr_t gsmTask_attributes = {
  .name = "gsmTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for serialTask */
osThreadId_t serialTaskHandle;
const osThreadAttr_t serialTask_attributes = {
  .name = "serialTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* USER CODE BEGIN PV */

//******************LVGL
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 5];
static lv_color_t buf2[LV_HOR_RES_MAX * 5];

lv_disp_drv_t disp_drv;
uint8_t my_buf[LV_HOR_RES_MAX * 5 * 2];
lv_indev_drv_t indev_drv;
lv_indev_t *  touch;

lv_style_t style;

char buffer[30];
char time_text[20];
char temp_buffer[256];

static lv_obj_t * kb;
lv_obj_t * text_area;

const char * kb_map[] = {
		  "1","2", "3", "4","\n",
		  "5", "6", "7", "8", "\n",
		  "9", "0", ".", LV_SYMBOL_BACKSPACE ,"\n",
		  LV_SYMBOL_OK,LV_SYMBOL_CLOSE,"",
};

const lv_btnmatrix_ctrl_t kb_ctrl[] = {
	   LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT,
	   LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT,
	   LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT,
	   LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_NO_REPEAT, LV_BTNMATRIX_CTRL_HIDDEN,
};

RTC_TimeTypeDef Time;
RTC_DateTypeDef Date;

volatile uint32_t pulseCount = 0, coinCount = 0;
volatile bool coin_in = false;

bool gps_data = false, time_is_set = false;

uint32_t led_timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartTimerTask(void *argument);
void StartGsmTask(void *argument);
void StartSerialTask(void *argument);
void StartGpsTask(void *argument);

/* USER CODE BEGIN PFP */
static void tft_flush(lv_disp_drv_t *data_regv, const lv_area_t *area, lv_color_t *color_p);

void welcomeScreen();
void homeScreen();
void setting();
void settingsPasscode();
void settingsPassword();
void readTime();
void setTime();
char getInput(char *label, bool mode);
bool touch_read(lv_indev_drv_t * drv, lv_indev_data_t * data);

void kb_event_cb(lv_obj_t * keyboard, lv_event_t e);

void delay(uint16_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of timerTask */
  timerTaskHandle = osThreadNew(StartTimerTask, NULL, &timerTask_attributes);

  /* creation of gsmTask */
  gsmTaskHandle = osThreadNew(StartGsmTask, NULL, &gsmTask_attributes);

  /* creation of serialTask */
  serialTaskHandle = osThreadNew(StartSerialTask, NULL, &serialTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 1024;
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
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_SET;
  /*if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }*/
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  /*if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }*/
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
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
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6,GPS.rxBuffer,sizeof(GPS.rxBuffer));
  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin|LCD_LED_Pin|LCD_DC_Pin|COOLING_Pin
                          |GSM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_LED_Pin LCD_DC_Pin COOLING_Pin
                           GSM_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_LED_Pin|LCD_DC_Pin|COOLING_Pin
                          |GSM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CTRL_Pin LOCK_Pin */
  GPIO_InitStruct.Pin = CTRL_Pin|LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HX711_SCK_Pin HX711_DI_Pin */
  GPIO_InitStruct.Pin = HX711_SCK_Pin|HX711_DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		coinCount++;
		coin_in = true;
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		pulseCount++;
	}
}

void delay(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim11, 0);
	while(__HAL_TIM_GET_COUNTER(&htim11)<us);
}

void openLock(){
    HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, GPIO_PIN_SET);//OPEN LOCK
    osDelay(50);
    HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, GPIO_PIN_RESET);//OPEN LOCK
    return;
}

bool checkDoorLock(){
	/*if(HAL_GPIO_ReadPin(LOCK_STATUS_GPIO_Port, LOCK_STATUS_Pin) == GPIO_PIN_RESET){
		settings.door_locked = true;
		return true;
	}
	else {
		settings.door_locked = false;
		return false;
	}*/
	return false;
}
void kb_event_cb(lv_obj_t * keyboard, lv_event_t e)
{

    lv_keyboard_def_event_cb(kb, e);
    if(e == LV_EVENT_CANCEL) {
    	//strcpy(buffer, lv_textarea_get_text(text_area));
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_del(kb);
        kb = NULL;
    }
    else if(e == LV_EVENT_APPLY){
    	sprintf((char*)buffer, "%s",lv_textarea_get_text(text_area));
    	//strcpy((char *)buffer, lv_textarea_get_text(text_area));
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_del(kb);
        kb = NULL;
    }
}


char getInput(char *label, bool mode){
	lv_obj_t * input_screen = lv_obj_create(NULL, NULL);
	lv_scr_load(input_screen);
	lv_obj_set_style_local_bg_color(input_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);


	lv_group_t * input_screen_group = lv_group_create();
	lv_indev_set_group(touch, input_screen_group);

	lv_obj_t * time_label = lv_label_create(input_screen, NULL);
	lv_obj_align(time_label, input_screen, LV_ALIGN_IN_TOP_RIGHT, -30, 0);
	sprintf(time_text,"%02d:%02d:%02d", Time.Hours,Time.Minutes, Time.Seconds);
	lv_label_set_text(time_label,time_text);
	//lv_obj_set_style_local_bg_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);

	 //Create the one-line mode text area
	text_area = lv_textarea_create(lv_scr_act(),NULL);
	if(!mode) lv_textarea_set_max_length(text_area, 4);
	lv_textarea_set_pwd_mode(text_area, !mode);
	lv_textarea_set_cursor_hidden(text_area, false);
	lv_textarea_set_accepted_chars(text_area, "0123456789.");
	lv_textarea_set_one_line(text_area, true);
	lv_textarea_set_text(text_area, "");
	lv_obj_align(text_area, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);

	//create a label
	lv_obj_t * input_label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(input_label, label);
	lv_obj_align(input_label, text_area, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

	kb = lv_keyboard_create(lv_scr_act(), NULL);
	lv_obj_set_size(kb,  LV_HOR_RES-5, 185);
	lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUM);
	lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_NUM, kb_map);
	lv_keyboard_set_ctrl_map(kb, LV_KEYBOARD_MODE_NUM,kb_ctrl);
	lv_keyboard_set_textarea(kb, text_area);
	lv_obj_align(kb, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
	lv_group_add_obj(input_screen_group, kb);

    lv_task_handler();
    osDelay(300);
    lv_obj_set_event_cb(kb, kb_event_cb);
    while(1){
    	osDelay(10);
		//sprintf(time_text,"%02u:%02u:%02u", Time.Hours,Time.Minutes, Time.Seconds);
		if(kb == NULL) break;
		lv_label_set_text(time_label,time_text);
		lv_task_handler();

    }

	lv_group_del(input_screen_group);
	lv_obj_del(input_screen);

    return strlen((char*)buffer);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    lv_disp_flush_ready(&disp_drv);
}

void rs485(char *data){
	if(rs_port.busy == 1){
		strcpy(rs_port.queue_data, data);
		rs_port.new_tx_data = 1;
		return;
	}
	else{
		strcpy(rs_port.queue_data, data);
		rs_port.new_tx_data = 1;
	}
	if (rs_port.new_tx_data == 1) {
		strcpy(rs_port.tx_buffer, rs_port.queue_data);
		rs_port.new_tx_data = 0;
		HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, SET);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)rs_port.tx_buffer, strlen(rs_port.tx_buffer));
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)//CALLED ON CPLT TRANSMISSION
{
	if (huart == &huart1) {
		HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, RESET);
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)rs_port.rx_buffer, 64);
		rs_port.busy = 0;
	}
	if (huart == &huart2){
		  HAL_UART_Receive_DMA(&huart2, (uint8_t *)gsm.port.rx_buffer, 256);//start DMA Receiver
	}
}
static void tft_flush(lv_disp_drv_t *data_regv, const lv_area_t *area, lv_color_t *color_p)
{
    /*Return if the area is out the screen*/
    if(area->x2 < 0) return;
    if(area->y2 < 0) return;
    if(area->x1 > LV_HOR_RES_MAX - 1) return;
    if(area->y1 > LV_VER_RES_MAX - 1) return;

    uint16_t x=0, y=0, z= 0;
    for(y=area->y1;y<=area->y2;y++)
	{
		for(x=area->x1;x<=area->x2;x++)
		{
			my_buf[z] = (color_p->full >> 8);
			z++;
			my_buf[z] = (color_p->full & 0xFF);
			z++;
			color_p++;
		}
	}
    ILI9341_Set_Address(area->x1, area->y1, area->x2, area->y2);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit_DMA(&hspi1, my_buf, z);
}
bool touch_read(lv_indev_drv_t * drv, lv_indev_data_t*data){
	if(TP.touched == 1){
		TP.touched = 0;
		data->point.x = TP.x;
	    data->point.y = TP.y;
	    data->state = LV_INDEV_STATE_PR;
	}
	else{
		data->state = LV_INDEV_STATE_REL;
	}
  //youtu.be/NjuvMQmZria
  return false; /*No buffering now so no more data read*/
}


void welcomeScreen(){
	lv_obj_t * welcome_screen  = lv_obj_create(NULL, NULL); // create screen
	lv_scr_load(welcome_screen);
	lv_obj_set_style_local_bg_color(welcome_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_SILVER);


	lv_group_t * welcome_screen_group = lv_group_create();
	lv_indev_set_group(touch, welcome_screen_group);

	lv_obj_t * top_bar = lv_obj_create(welcome_screen, NULL);
	lv_obj_set_size(top_bar, 320, 30);
	lv_obj_set_style_local_bg_color(top_bar, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(top_bar_color_1));
	lv_obj_align(top_bar, welcome_screen, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_set_click(top_bar, false);

	lv_obj_t * time_label = lv_label_create(top_bar, NULL);
	lv_label_set_text(time_label,time_text);
	lv_obj_align(time_label, top_bar, LV_ALIGN_IN_RIGHT_MID, -10, 0);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_obj_t * main_body = lv_obj_create(welcome_screen, NULL);
	lv_obj_set_size(main_body, 320, 210);
	lv_obj_set_style_local_bg_color(main_body, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(main_body_color_1));
	lv_obj_align(main_body, top_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
	lv_obj_set_click(main_body, false);

	lv_obj_t * brand_text = lv_label_create(main_body, NULL);
	lv_label_set_text(brand_text, "Macangira Africa");
	lv_obj_set_style_local_text_color(brand_text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_obj_set_style_local_text_font(brand_text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_32);
	lv_obj_align(brand_text, main_body, LV_ALIGN_CENTER, 0, -35);

    lv_obj_t * welcome_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(welcome_button, main_body, LV_ALIGN_IN_BOTTOM_MID, 0, -40);
	lv_group_add_obj(welcome_screen_group, welcome_button);


	lv_obj_t * welcome_button_label = lv_label_create(welcome_button, NULL);         // Add a label to the button
	lv_label_set_text(welcome_button_label, "WELCOME");

	lv_task_handler();

	while(1){
		osDelay(5);
		lv_label_set_text(time_label,time_text);
		lv_task_handler();
		HAL_IWDG_Refresh(&hiwdg);
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(welcome_button)){
			lv_group_del( welcome_screen_group);
			lv_obj_del(welcome_screen);
			break;
		}
	}

}

void homeScreen(){
	homeScreen_redraw:;
	lv_obj_t * home_screen  = lv_obj_create(NULL, NULL); // create screen
	lv_scr_load(home_screen);
	lv_obj_set_style_local_bg_color(home_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_SILVER);
	lv_obj_set_click(home_screen, false);


	lv_group_t * home_screen_group = lv_group_create();
	lv_indev_set_group(touch, home_screen_group);

	lv_obj_t * top_bar = lv_obj_create(home_screen, NULL);
	lv_obj_set_size(top_bar, 320, 30);
	lv_obj_set_style_local_bg_color(top_bar, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(top_bar_color_2));
	lv_obj_align(top_bar, home_screen, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_set_click(top_bar, false);

	lv_obj_t * time_label = lv_label_create(top_bar, NULL);
	lv_label_set_text(time_label,time_text);
	lv_obj_align(time_label, top_bar, LV_ALIGN_IN_RIGHT_MID, -10, 0);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_obj_t * main_body = lv_obj_create(home_screen, NULL);
	lv_obj_set_size(main_body, 320, 210);
	lv_obj_set_style_local_bg_color(main_body, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(main_body_color_2));
	lv_obj_align(main_body, top_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
	lv_obj_set_click(main_body, false);

	/*lv_obj_t * brand_text = lv_label_create(main_body, NULL);
	lv_label_set_text(brand_text, "Macangira Africa");
	lv_obj_set_style_local_text_color(brand_text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_obj_set_style_local_text_font(brand_text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_32);
	lv_obj_align(brand_text, main_body, LV_ALIGN_CENTER, 0, -35);*/

    lv_obj_t * open_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(open_button, main_body, LV_ALIGN_IN_LEFT_MID, 20, 0);
	lv_group_add_obj(home_screen_group, open_button);


	lv_obj_t * open_button_label = lv_label_create(open_button, NULL);         // Add a label to the button
	lv_label_set_text(open_button_label, "OPEN");

    lv_obj_t * admin_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(admin_button, main_body, LV_ALIGN_IN_RIGHT_MID, -20, 0);
	lv_group_add_obj(home_screen_group, admin_button);


	lv_obj_t * admin_button_label = lv_label_create(admin_button, NULL);         // Add a label to the button
	lv_label_set_text(admin_button_label, "ADMIN");

	lv_task_handler();

	while(1){
		osDelay(5);
		lv_label_set_text(time_label,time_text);
		lv_task_handler();
		HAL_IWDG_Refresh(&hiwdg);
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(open_button)){
			if(getInput("Password", true) > 0){
				if(strcmp(buffer,settings.passcode) != 0){
					lv_group_del( home_screen_group);
					lv_obj_del(home_screen);
					goto homeScreen_redraw;
				}
				buffer[0] = 0;
			}
			lv_scr_load(home_screen);
			lv_task_handler();
		    lv_obj_t * mbox1 = lv_msgbox_create(main_body, NULL);
		    lv_obj_set_width(mbox1, 200);
		    lv_obj_align(mbox1, main_body, LV_ALIGN_CENTER, 0, 0); /*Align to the corner*/
		    lv_msgbox_start_auto_close(mbox1, 4000);
		    lv_msgbox_set_anim_time(mbox1, 400);
			if (settings.active) {
				lv_msgbox_set_text(mbox1, "OPENING");
				lv_task_handler();
				openLock();
				for (int i = 0; i < 60; i++) {
					lv_task_handler();
					osDelay(5);
				}
			}
			else{
				lv_msgbox_set_text(mbox1, "KINDLY PAY");
				lv_task_handler();
				for (int i = 0; i < 60; i++) {
					lv_task_handler();
					osDelay(5);
				}
				continue;
			}
			lv_obj_del(mbox1);
		}
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(admin_button)){
			lv_group_del( home_screen_group);
			lv_obj_del(home_screen);
			if(getInput("Password", true) > 0){
				if(strcmp(buffer,settings.password) == 0){
					setting();
				}
				buffer[0] = 0;
			}
			goto homeScreen_redraw;
		}
	}
}


void setting(){
	settings_redraw:;
	lv_obj_t * settings_screen  = lv_obj_create(NULL, NULL);
    lv_scr_load(settings_screen);
	lv_obj_set_style_local_bg_color(settings_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_group_t * settings_screen_group = lv_group_create();
	lv_indev_set_group(touch, settings_screen_group);

	lv_obj_t * top_bar = lv_obj_create(settings_screen, NULL);
	lv_obj_set_size(top_bar, 320, 30);
	lv_obj_set_style_local_bg_color(top_bar, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(top_bar_color_5));
	lv_obj_align(top_bar, settings_screen, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_set_click(top_bar, false);

	lv_obj_t * time_label = lv_label_create(top_bar, NULL);
	lv_label_set_text(time_label,time_text);
	lv_obj_align(time_label, top_bar, LV_ALIGN_IN_RIGHT_MID, -10, 0);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_obj_t * main_body = lv_obj_create(settings_screen, NULL);
	lv_obj_set_size(main_body, 320, 210);
	lv_obj_set_style_local_bg_color(main_body, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(main_body_color_3));
	lv_obj_align(main_body, top_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
	lv_obj_set_click(main_body, false);

    lv_obj_t * passcode_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(passcode_button, main_body, LV_ALIGN_IN_LEFT_MID, 20, 0);
	lv_group_add_obj(settings_screen_group, passcode_button);
	lv_obj_t * passcode_button_label = lv_label_create(passcode_button, NULL);         // Add a label to the button
	lv_label_set_text(passcode_button_label, "PASSCODE");

    lv_obj_t * password_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(password_button, main_body, LV_ALIGN_IN_RIGHT_MID, -20, 0);
	lv_group_add_obj(settings_screen_group, password_button);
	lv_obj_t * password_button_label = lv_label_create(password_button, NULL);         // Add a label to the button
	lv_label_set_text(password_button_label, "PASSWORD");

    lv_obj_t * cancel_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(cancel_button, main_body, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
	lv_group_add_obj(settings_screen_group, cancel_button);
	lv_obj_t * cancel_button_label = lv_label_create(cancel_button, NULL);         // Add a label to the button
	lv_label_set_text(cancel_button_label, LV_SYMBOL_CLOSE);

	lv_task_handler();
	osDelay(300);

	while(1){
		lv_task_handler();
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(cancel_button)){
			lv_group_del(settings_screen_group);
			lv_obj_del(settings_screen);
			break;
		}
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(passcode_button)){
			lv_group_del(settings_screen_group);
			lv_obj_del(settings_screen);
			settingsPasscode();
			goto settings_redraw;
		}//password
		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(password_button)){
			lv_group_del(settings_screen_group);
			lv_obj_del(settings_screen);
			settingsPassword();
			goto settings_redraw;
		}

		lv_label_set_text(time_label,time_text);
		//lv_task_handler();
		osDelay(5);
	}
}


void settingsPassword(){
	char* password = settings.password;
	password_redraw:;
	char local_buffer[20];
	lv_obj_t * password_screen  = lv_obj_create(NULL, NULL);
    lv_scr_load(password_screen);
	lv_obj_set_style_local_bg_color(password_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_group_t * password_screen_group = lv_group_create();
	lv_indev_set_group(touch, password_screen_group);

	lv_obj_t * top_bar = lv_obj_create(password_screen, NULL);
	lv_obj_set_size(top_bar, 320, 30);
	lv_obj_set_style_local_bg_color(top_bar, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(top_bar_color_3));
	lv_obj_align(top_bar, password_screen, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_set_click(top_bar, false);

	lv_obj_t * time_label = lv_label_create(top_bar, NULL);
	lv_label_set_text(time_label,time_text);
	lv_obj_align(time_label, top_bar, LV_ALIGN_IN_RIGHT_MID, -10, 0);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_obj_t * main_body = lv_obj_create(password_screen, NULL);
	lv_obj_set_size(main_body, 320, 210);
	lv_obj_set_style_local_bg_color(main_body, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(main_body_color_4));
	lv_obj_align(main_body, top_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
	lv_obj_set_click(main_body, false);


	lv_obj_t *current_value_ta = lv_textarea_create(main_body, NULL);
    lv_obj_set_size(current_value_ta, 200, 50);
    lv_obj_align(current_value_ta, main_body, LV_ALIGN_IN_TOP_MID, 0, 40);
    sprintf(local_buffer,"%s", password);
    lv_textarea_set_text(current_value_ta, local_buffer);    /*Set text*/
    lv_textarea_set_cursor_hidden(current_value_ta, true);
    lv_textarea_set_text_align(current_value_ta, LV_LABEL_ALIGN_CENTER);
	lv_obj_set_style_local_text_color(current_value_ta, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
	lv_obj_set_style_local_text_font(current_value_ta, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_32);

	//create a label
	lv_obj_t * current_label = lv_label_create(main_body, NULL);
	lv_label_set_text(current_label, "PASSWORD:");
	lv_obj_align(current_label, current_value_ta, LV_ALIGN_OUT_TOP_MID, 0, 0);

    lv_obj_t * save_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(save_button, main_body, LV_ALIGN_IN_RIGHT_MID, -20, 20);
	lv_group_add_obj(password_screen_group, save_button);
	lv_obj_t * save_button_label = lv_label_create(save_button, NULL);         // Add a label to the button
	lv_label_set_text(save_button_label, LV_SYMBOL_SAVE);

    lv_obj_t * edit_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_t * edit_button_label = lv_label_create(edit_button, NULL);         // Add a label to the button
	lv_obj_align(edit_button, main_body, LV_ALIGN_IN_LEFT_MID, 20, 20);
	lv_label_set_text(edit_button_label, LV_SYMBOL_EDIT);

    lv_obj_t * back_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(back_button, main_body, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
	lv_group_add_obj(password_screen_group, back_button);
	lv_obj_t * back_button_label = lv_label_create(back_button, NULL);         // Add a label to the button
	lv_label_set_text(back_button_label, "BACK");


	lv_task_handler();
	osDelay(300);

	while(1){
		lv_label_set_text(time_label,time_text);
		lv_task_handler();


		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(back_button)){
			lv_group_del(password_screen_group);
			lv_obj_del(password_screen);
			break;
		}

		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(save_button)){
			strcpy(settings.password,password);
			FlashWrite();
		    lv_obj_t * mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
		    lv_msgbox_set_text(mbox1, "saved!");
		    lv_obj_set_width(mbox1, 200);
		    lv_msgbox_start_auto_close(mbox1, 1000);
		    lv_msgbox_set_anim_time(mbox1, 100);
		    lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0); /*Align to the corner*/
		}

		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(edit_button)){

			lv_group_del(password_screen_group);
			lv_obj_del(password_screen);

			if(getInput("new password", true) > 0){
				password = (char *)buffer;
				buffer[0] = 0;
			}
			goto password_redraw;
		}
		osDelay(5);
	}
}


void settingsPasscode(){
	char* passcode = settings.passcode;
	passcode_redraw:;
	char local_buffer[20];
	lv_obj_t * passcode_screen  = lv_obj_create(NULL, NULL);
    lv_scr_load(passcode_screen);
	lv_obj_set_style_local_bg_color(passcode_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_group_t * passcode_screen_group = lv_group_create();
	lv_indev_set_group(touch, passcode_screen_group);

	lv_obj_t * top_bar = lv_obj_create(passcode_screen, NULL);
	lv_obj_set_size(top_bar, 320, 30);
	lv_obj_set_style_local_bg_color(top_bar, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(top_bar_color_4));
	lv_obj_align(top_bar, passcode_screen, LV_ALIGN_IN_TOP_MID, 0, 0);
	lv_obj_set_click(top_bar, false);

	lv_obj_t * time_label = lv_label_create(top_bar, NULL);
	lv_label_set_text(time_label,time_text);
	lv_obj_align(time_label, top_bar, LV_ALIGN_IN_RIGHT_MID, -10, 0);
	lv_obj_set_style_local_text_color(time_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

	lv_obj_t * main_body = lv_obj_create(passcode_screen, NULL);
	lv_obj_set_size(main_body, 320, 210);
	lv_obj_set_style_local_bg_color(main_body, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(main_body_color_5));
	lv_obj_align(main_body, top_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
	lv_obj_set_click(main_body, false);


	lv_obj_t *current_value_ta = lv_textarea_create(main_body, NULL);
    lv_obj_set_size(current_value_ta, 200, 50);
    lv_obj_align(current_value_ta, main_body, LV_ALIGN_IN_TOP_MID, 0, 40);
    sprintf(local_buffer,"%s", passcode);
    lv_textarea_set_text(current_value_ta, local_buffer);    /*Set text*/
    lv_textarea_set_cursor_hidden(current_value_ta, true);
    lv_textarea_set_text_align(current_value_ta, LV_LABEL_ALIGN_CENTER);
	lv_obj_set_style_local_text_color(current_value_ta, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
	lv_obj_set_style_local_text_font(current_value_ta, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_32);

	//create a label
	lv_obj_t * current_label = lv_label_create(main_body, NULL);
	lv_label_set_text(current_label, "PASS CODE:");
	lv_obj_align(current_label, current_value_ta, LV_ALIGN_OUT_TOP_MID, 0, 0);

    lv_obj_t * save_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(save_button, main_body, LV_ALIGN_IN_RIGHT_MID, -20, 20);
	lv_group_add_obj(passcode_screen_group, save_button);
	lv_obj_t * save_button_label = lv_label_create(save_button, NULL);         // Add a label to the button
	lv_label_set_text(save_button_label, LV_SYMBOL_SAVE);

    lv_obj_t * edit_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_t * edit_button_label = lv_label_create(edit_button, NULL);         // Add a label to the button
	lv_obj_align(edit_button, main_body, LV_ALIGN_IN_LEFT_MID, 20, 20);
	lv_label_set_text(edit_button_label, LV_SYMBOL_EDIT);

    lv_obj_t * back_button = lv_btn_create(main_body, NULL);     //Add a button to the current screen
	lv_obj_align(back_button, main_body, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
	lv_group_add_obj(passcode_screen_group, back_button);
	lv_obj_t * back_button_label = lv_label_create(back_button, NULL);         // Add a label to the button
	lv_label_set_text(back_button_label, "BACK");


	lv_task_handler();
	osDelay(300);

	while(1){
		lv_label_set_text(time_label,time_text);
		lv_task_handler();


		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(back_button)){
			lv_group_del(passcode_screen_group);
			lv_obj_del(passcode_screen);
			break;
		}

		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(save_button)){
			strcpy(settings.passcode, passcode);
			FlashWrite();
		    lv_obj_t * mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
		    lv_msgbox_set_text(mbox1, "saved!");
		    lv_obj_set_width(mbox1, 200);
		    lv_msgbox_start_auto_close(mbox1, 1000);
		    lv_msgbox_set_anim_time(mbox1, 100);
		    lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0); /*Align to the corner*/
		}

		if(LV_BTN_STATE_PRESSED == lv_btn_get_state(edit_button)){

			lv_group_del(passcode_screen_group);
			lv_obj_del(passcode_screen);

			if(getInput("new pass code", true) > 0){
				passcode = (char *)buffer;
				buffer[0] = 0;
			}
			goto passcode_redraw;
		}
		osDelay(5);
	}
}


/*
void clock_calender(){
	clock_redraw:;
	lv_obj_t * clock_screen = lv_obj_create(NULL, NULL);
	lv_scr_load(clock_screen);
	lv_obj_set_style_local_bg_color(clock_screen, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    lv_obj_t  * calendar = lv_calendar_create(lv_scr_act(), NULL);
    lv_obj_set_size(calendar, 235, 235);
    lv_obj_align(calendar, clock_screen, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_event_cb(calendar, event_handler);

    //Make the date number smaller to be sure they fit into their area
    lv_obj_set_style_local_text_font(calendar, LV_CALENDAR_PART_DATE, LV_STATE_DEFAULT, lv_theme_get_font_small());

    //Set today's date
    lv_calendar_date_t today;
    today.year = 2021;
    today.month = 10;
    today.day = 26;

    lv_calendar_set_today_date(calendar, &today);
    lv_calendar_set_showed_date(calendar, &today);

}
*/

/*
static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        lv_calendar_date_t * date = lv_calendar_get_pressed_date(obj);
        if(date) {
            printf("Clicked date: %02d.%02d.%d\n", date->day, date->month, date->year);
        }
    }
}

static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_roller_get_selected_str(obj, buf, sizeof(buf));
        printf("Selected month: %s\n", buf);
    }
}


void lv_ex_roller_1(void)
{
    lv_obj_t *roller1 = lv_roller_create(lv_scr_act(), NULL);
    lv_roller_set_options(roller1,
                        "January\n"
                        "February\n"
                        "March\n"
                        "April\n"
                        "May\n"
                        "June\n"
                        "July\n"
                        "August\n"
                        "September\n"
                        "October\n"
                        "November\n"
                        "December",
                        LV_ROLLER_MODE_INFINITE);

    lv_roller_set_visible_row_count(roller1, 4);
    lv_obj_align(roller1, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(roller1, event_handler);
}
*/

void readTime(){
	  HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	  sprintf(time_text,"%02d:%02d:%02d", Time.Hours,Time.Minutes, Time.Seconds);
	  //sprintf(time_text,"%02d:%02d:%02d", gsm.time.hour,gsm.time.minute, gsm.time.second);
}
void setTime(){
	  Time.Hours = gsm.time.hour;
	  Time.Minutes = gsm.time.minute;
	  Time.Seconds = gsm.time.second;
	  if (HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  //sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  Date.Month = gsm.time.month;
	  Date.Date = gsm.time.day;
	  Date.Year = gsm.time.year;

	  if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/*
 * Description	Resource	Path	Location	Type
make: *** [Core/Src/subdir.mk:42: Core/Src/main.o] Error 1	VMOS_TOUCH		 	C/C++ Problem
 *
 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	ILI9341_Init();
	ILI9341_Set_Rotation(3);
	ILI9341_Set_Address(0, 0, 320, 240);

	lv_init();
	lv_disp_buf_init(&disp_buf, buf, buf2, LV_HOR_RES_MAX * 5);


	   lv_disp_drv_init(&disp_drv);
	   disp_drv.flush_cb = tft_flush;
	   disp_drv.buffer = &disp_buf;
	   lv_disp_drv_register(&disp_drv);


	   lv_indev_drv_init(&indev_drv);  /*Basic initialization*/
	   indev_drv.type = LV_INDEV_TYPE_POINTER; /*See below.*/
	   indev_drv.read_cb = touch_read; /*See below.*/
	   touch = lv_indev_drv_register(&indev_drv);  /*Register the driver in LittlevGL*/

	   initFlash();
	   coinCount = 0;
	   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	   HAL_TIM_Base_Start_IT(&htim11);

	   welcomeScreen();
  /* Infinite loop */
  for(;;)
  {
	homeScreen();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTimerTask */
/**
* @brief Function implementing the timerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTimerTask */
void StartTimerTask(void *argument)
{
  /* USER CODE BEGIN StartTimerTask */
  /* Infinite loop */
  for(;;)
  {
	osDelay(100);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    readTime();
  }
  /* USER CODE END StartTimerTask */
}

/* USER CODE BEGIN Header_StartGsmTask */
/**
* @brief Function implementing the gsmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGsmTask */
void StartGsmTask(void *argument)
{
  /* USER CODE BEGIN StartGsmTask */
	gsmInit();
	waitForRegister(20);
	uint32_t timer_15s = HAL_GetTick();
	//uint32_t timer_20s = HAL_GetTick();
	uint32_t timer_60s = HAL_GetTick();
	for(int i =1; i<26;i++){
		gsm.msg.slot = i;
		gsmDeleteSMS();
	}
	gsm.msg.slot = 0;
  /* Infinite loop */
  for(;;)
  {
	    osDelay(1000);
		if(gsm.msg.send == 1){
			gsm.msg.send = 0;
			gsmSendSMS();
		}

		if (HAL_GetTick() - timer_15s > 15000) {
			timer_15s = HAL_GetTick();
			if(gsm.registered != 1){
				gsmNetworkStatus();
			}

			if((gsm.registered == 1) && (gsm.time.set != 1)){
				getNetworkTime();
				if(gsm.time.set == 1){
					setTime();
				}
			}

			if(gsm.imei != 1){
				gsmGetImei();
			}

		    if(gsm.msg.slot > 0){
				gsmReadSMS();
				//gsm.msg.slot = 0;
			}
		}
		if(HAL_GetTick() - timer_60s > 60000){
			timer_60s = HAL_GetTick();
		}

  }
  /* USER CODE END StartGsmTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief Function implementing the serialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void *argument)
{
  /* USER CODE BEGIN StartSerialTask */
  /* Infinite loop */
  for(;;)
  {
	osThreadFlagsWait(_SERIAL_PORT_EVENT, osFlagsWaitAny, osWaitForever);
	osThreadFlagsClear(_SERIAL_PORT_EVENT);
    checkGsmPort();
    osDelay(1);
  }
  /* USER CODE END StartSerialTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(XPT2046_TouchPressed()){
		if(XPT2046_TouchGetCoordinates(&TP.y, &TP.x) == true)TP.touched = 1;
    }
    if(GPS.rx_event == 1){
    	GPS_Process();
    	if((time_is_set == false) && (GPS.GPGGA.LatitudeDecimal != 0.0f) && GPS.GPGGA.LongitudeDecimal != 0.0f){
    		Time.Hours = (GPS.GPGGA.UTC_Hour+3)%24;
    		Time.Minutes = GPS.GPGGA.UTC_Min;
    		Time.Seconds = GPS.GPGGA.UTC_Sec;
    		HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
    		time_is_set = true;
    	}
    	GPS.rx_event = 0;
    }
  }
  /* USER CODE END StartGpsTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	lv_tick_inc(1);
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
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
