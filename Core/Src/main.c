/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * The work of Sandra Snyman (20826702) for Design E314, 2020.
  *
  *
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sinewave.h"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

enum states{Stop, Play, Record};
enum states myState;

extern volatile uint8_t toggle;

uint8_t button1 = 0;   	//PA6
uint8_t button2 = 0;	//PB6
uint8_t button3 = 0;	//PC7
uint8_t buttonrec = 0;	//PA9
uint8_t buttonstop = 0;	//PA7

uint32_t buttonStopTick = 0;


uint16_t sine440[1024] = {0};
uint16_t sine523[1024] = {0};
uint16_t sinesum[1024] = {0};

uint8_t track[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	huart2.gState = HAL_UART_STATE_READY;
	HAL_UART_Transmit_DMA(&huart2, track+512, 512);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	huart2.gState = HAL_UART_STATE_READY;
	HAL_UART_Transmit_DMA(&huart2, track, 512);
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
  wave_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_ENABLE(&htim2);
   __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
   __HAL_TIM_ENABLE(&htim3);
   //HAL_TIM_Base_Start(&htim3);
   HAL_TIM_Base_Start(&htim4);




  uint8_t begin = 0; //used in every state for every track to trigger the actions that need to happen only once
  uint32_t refTick = 0;
  uint8_t playing = 0;

  uint32_t button1Tick = 0; //counter to save how long button has been high
  uint32_t button2Tick = 0;
  uint32_t button3Tick = 0;

  uint8_t startup_msg[10] = {127, 128, '2', '0', '8', '2', '6', '7', '0', '2'};
  uint8_t record_msg[10] = {127, 128, 'R', 'e', 'c', 'o', 'r', 'd', '_', '_'};
  uint8_t playback_msg[10] = {127, 128, 'P', 'l', 'a', 'y', '_', '_', '_', '_'};
  uint8_t stop_msg[10] = {127, 128, 'S', 't', 'o', 'p', '_', '_', '_', '_'};

  wave_fillbuffer(sine440, 1, 1024);
  wave_fillbuffer(sine523, 2, 1024);
  wave_fillbuffer(sinesum, 3, 1024);

  myState = Stop;
  //STOP is state 1, PLAY is state 2, RECORD is state 3
  HAL_UART_Transmit(&huart2, startup_msg, 10, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  refTick = HAL_GetTick();
	  if (myState == Stop) {
		  allLEDSoff();

		  //if button 1 is high, add to counter
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
			  button1Tick += (HAL_GetTick() - refTick);
			  if (button1Tick >= 10) button1 = 1;
		  }
		  else button1Tick = 0;

		  //if button 2 is high, add to counter
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
			  button2Tick += (HAL_GetTick() - refTick);
			  if (button2Tick >= 10) button2 = 1;
		  }
		  else button2Tick = 0;

		  //if button 3 is high, add to counter
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {
			  button3Tick += (HAL_GetTick() - refTick);
			  if (button3Tick >= 10) button3 = 1;
		  }
		  else button3Tick = 0;

		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) && (button1 || button2 || button3)){
			  myState = Record;
			  begin = 1;
			  buttonstop = 0;
		  }
		  else if ((button1)||(button2)||(button3)) {
			  myState = Play;
			  begin = 1;
			  buttonstop = 0;
		  }
		  if (buttonstop)
			  myState = Stop;
	  }

	  if (myState == Play) {
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)) {
			  buttonStopTick += (HAL_GetTick() - refTick);
			  if (buttonStopTick >= 10) buttonstop = 1;
		  }
		  else {
			  buttonStopTick = 0;
		  }

		  if (button1 && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
			    if (begin && !playing) {
			    	//if this is the beginning of the playback, send UART message and start timer
			    	playback_msg[9] = '1';
			    	HAL_UART_Transmit(&huart2, playback_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in playback time
					 HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sine440, 1024, DAC_ALIGN_12B_R);
					 playing = 1;
			    }
			   if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
				   //if not exceeded max length, and stop is not pressed, flash LED
				   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, toggle);
			   }
			   else {
				   button1 = 0;
				   HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				   myState = Stop;
				   playing = 0;
				   HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
			   }
		  }

		  else if (button2 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
			    if (begin && !playing) {
			    	//if this is the beginning of the playback, send UART message and start timer
			    	playback_msg[9] = '2';
			    	HAL_UART_Transmit(&huart2, playback_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in playback time
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sine523, 1024, DAC_ALIGN_12B_R);
					playing = 1;
			    }
			   if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
				   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, toggle);
			   }
			   else {
				   button2 = 0;
				   HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				   myState = Stop;
				   playing = 0;
				   HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
			   }
		  }
		  else if (button3 && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {
			    if (begin && !playing) {
			    	//if this is the beginning of the playback, send UART message and start timer
			    	playback_msg[9] = '3';
			    	HAL_UART_Transmit(&huart2, playback_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in playback time
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sinesum, 1024, DAC_ALIGN_12B_R);
					playing = 1;
			    }
			  if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
				   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, toggle);
			   }	//LED 3
			  else {
				  button3 = 0;
				  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				  myState = Stop;
				  playing = 0;
				  HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
			  }
		  }
		  else myState = Stop;
	  }

	  if (myState == Record) {
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)) {
			  buttonStopTick += (HAL_GetTick() - refTick);
			  if (buttonStopTick >= 10) buttonstop = 1;
		  }
		  else {
			  buttonStopTick = 0;
		  }
		  if (button1 && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
			    if (begin) {
			    	//if this is the beginning of the recording, turn REC LED on, send UART message and start timer
			    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
			    	record_msg[9] = '1';
			    	HAL_UART_Transmit(&huart2, record_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in recording time
					HAL_ADC_Start_DMA(&hadc2, (uint32_t*) track, 1024);
			    }
			  //While stop button has not been pressed, flash LED
			  if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, toggle);
			  }
			  else {
				  button1 = 0;
				  myState = Stop;
				  HAL_UART_DMAStop(&huart2);
				  HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
				  HAL_ADC_Stop_DMA(&hadc2);

			  }
		  }
		  else if (button2 && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
			    if (begin) {
			    	//if this is the beginning of the recording, turn REC LED on, send UART message and start timer
			    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
			    	record_msg[9] = '2';
			    	HAL_UART_Transmit(&huart2, record_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in recording time
			    }
				   if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
					   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, toggle);
				   }
				   else {
					   button2 = 0;
					   myState = Stop;
					   HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
				   }
		  }
		  else if (button3 && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {
			    if (begin) {
			    	//if this is the beginning of the recording, turn REC LED on, send UART message and start timer
			    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
			    	record_msg[9] = '3';
			    	HAL_UART_Transmit(&huart2, record_msg, 10, 1000);
					begin = 0; //to ensure this block is executed only once in recording time
			    }
				   if (!(buttonstop && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))) {
					   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, toggle);
				   }
				   else {
					   button3 = 0;
					   myState = Stop;
					   HAL_UART_Transmit(&huart2, stop_msg, 10, 1000);
				   }
			  }

	  }

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1905;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1905;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void allLEDSoff() {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); //1
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); //3
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0); //REC
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
	if (button1) {
		HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*) sine440, 1024, DAC_ALIGN_12B_R);
		wave_fillbuffer(sine440+512, 1, 512);
	}
	if (button2) {
		HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*) sine523, 1024, DAC_ALIGN_12B_R);
		wave_fillbuffer(sine523+512, 2, 512);
	}
	if (button3) {
		HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*) sinesum, 1024, DAC_ALIGN_12B_R);
		wave_fillbuffer(sinesum+512, 3, 512);
	}
}
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
	if (button1) {
		wave_fillbuffer(sine440, 1, 512);
	}
	if (button2) {
		wave_fillbuffer(sine523, 2, 512);
	}
	if (button3) {
		wave_fillbuffer(sinesum, 3, 512);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
