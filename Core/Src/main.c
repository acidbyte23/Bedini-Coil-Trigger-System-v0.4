/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
// Calculation Settings
const float voltageR1 = 100000.0;
const float voltageR2 = 5656.0;
const uint32_t MAX_BATTERY_VOLTAGE = ((((voltageR1 + voltageR2) / voltageR2) * 3.3) * 100.0); // ~3.3 volt = (100k -> 5k) = resolution of ~mV
const uint32_t MAX_BATTERY_CURRENT = 1000000; // to be calculated in uA
const uint32_t MAX_PULSE_DELAY = 5000; // some value making 1 = 1 uS
const uint32_t MAX_PULSE_WIDTH = 5000; // some value making 1 = 1 uS 
const uint32_t STEP_MULTIPLIER_DELAY = 1;
const uint32_t STEP_MULTIPLIER_WIDTH = 1;

// Pulse motor settings
const int MAGNETS_ON_ROTOR = 10;
volatile uint32_t rpmSetPulse = 1500;
volatile int modeWidthPulse = 0;

// PID Settings
int MIN_BANDWIDTH_PID = 100;
int MAX_BANDWIDTH_PID = 100;
int MAX_TUNE_PID = 150;
int MIN_TUNE_PID = 0;
int DEADBAND_PID = 0;
volatile uint32_t	multiplierPid;
volatile uint32_t rpmDifference[3];

// Analog Inputs
const int SHIFT_ARRAY = 20;
volatile uint32_t analogInputs[4];
volatile uint32_t analogInAvg[SHIFT_ARRAY][4];
const int analogChCounts = sizeof ( analogInputs) / sizeof (analogInputs[0]);
volatile int analogConvComplete = 0;


// Proram Vars
volatile uint32_t delayPulse = 0;
volatile uint32_t widthPulse = 0;
volatile uint32_t comparePulse[3];
volatile uint32_t rpmPulse = 0;
volatile int multiplierPulse = 100;
volatile uint32_t voltageAvg;
volatile uint32_t currentAvg;
volatile uint32_t delayAvg;
volatile uint32_t widthAvg;
volatile uint32_t voltageAvgCalc;
volatile uint32_t currentAvgCalc;
volatile uint32_t delayAvgCalc;
volatile uint32_t widthAvgCalc;

volatile uint32_t voltageBattery = 0;
volatile uint32_t currentBattery = 0;

volatile int pulseTrigger;
volatile int motorRunState = 0;


// WIFI SSID and PASSWORD
uint8_t SSID[] = "Free Energy";
uint8_t PASS[] = "Nternet23!";

// uart data stuf
uint32_t dataCounter;
uint8_t data1[8];
uint32_t prevTimerCount;
uint32_t dataCycleTime = 1000;
uint8_t Rx_data[8];
uint8_t dataCommand;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	// Init reference timer
	HAL_TIM_Base_Start(&htim1);
	HAL_UART_Receive_IT (&huart5, Rx_data, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 2)){
			if(pulseTrigger == 2){
				comparePulse[2] = (comparePulse[0] + comparePulse[1]) / 2;
				if(comparePulse[2] > 0){
					rpmPulse = (((1000000.0 / (float)comparePulse[2]) * 60.0) / (float)MAGNETS_ON_ROTOR);
				}
				pulseTrigger = 0;
			}

			if((TIM1->CNT >= delayPulse) && (pulseTrigger == 1)){
				TIM3->ARR = widthPulse / 3;
				__HAL_TIM_ENABLE(&htim3);
				pulseTrigger = 2;
			}
		}
		

		if((pulseTrigger == 0) && (TIM1->CNT >= (prevTimerCount + dataCycleTime))){
			prevTimerCount = TIM1->CNT;
			
			switch(dataCounter){
				case 0:
					//Send motor data over uart
					data1[0]= 0x70;
					data1[1]= 0x10;
					data1[2]=(voltageBattery >> 24);
					data1[3]=(voltageBattery >> 16);
					data1[4]=(voltageBattery >> 8);
					data1[5]=(voltageBattery);
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				case 1:
					//Send motor data over uart
					data1[0]= 0x70;
					data1[1]= 0x20;
					data1[2]=(currentBattery >> 24);
					data1[3]=(currentBattery >> 16);
					data1[4]=(currentBattery >> 8);
					data1[5]=(currentBattery);
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				case 2:
					//Send motor data over uart
					data1[0]= 0x70;
					data1[1]= 0x30;
					data1[2]=(rpmPulse >> 24);
					data1[3]=(rpmPulse >> 16);
					data1[4]=(rpmPulse >> 8);
					data1[5]=(rpmPulse);
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				case 3:
					//Send motor data over uart
					data1[0]= 0x70;
					data1[1]= 0x40;
					data1[2]=(widthPulse >> 24);
					data1[3]=(widthPulse >> 16);
					data1[4]=(widthPulse >> 8);
					data1[5]=(widthPulse);
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				case 4:
					//Send motor data over uart
					data1[0]= 0x70;
					data1[1]= 0x50;
					data1[2]=(delayPulse >> 24);
					data1[3]=(delayPulse >> 16);
					data1[4]=(delayPulse >> 8);
					data1[5]=(delayPulse);
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			}
			dataCounter++;
			
			if(dataCounter >= 5){
					dataCounter = 0;
			} 
		} 
		
		if(dataCommand == 1){
			
		}
		
		while( pulseTrigger == 0){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analogInputs, analogChCounts);
			while(analogConvComplete == 0){
			if(comparePulse[2] > 0){
				rpmPulse = (((1000000.0 / (float)comparePulse[2]) * 60.0) / (float)MAGNETS_ON_ROTOR);
			}
			}
			analogConvComplete = 0;
			

			for(int i = (SHIFT_ARRAY - 1);i >= 0;i--){
				for(int ii = 0;ii <= 3;ii++){
					analogInAvg[i][ii] = analogInAvg[(i-1)][ii];
				}
			}
			for(int i = 0;i <= 3;i++){
				analogInAvg[0][i] = analogInputs[i];
			}
						
			
			for(int i = (SHIFT_ARRAY - 1);i >= 0;i--){
				for(int ii = 0;ii <= 3;ii++){
					analogInAvg[i][ii] = analogInAvg[(i-1)][ii];
				}
			}
			for(int i = 0;i <= 3;i++){
				analogInAvg[0][i] = analogInputs[i];
			}
			
			voltageAvgCalc = 0;
			
			for(int i = 0; i < SHIFT_ARRAY; i++){
				voltageAvgCalc += analogInAvg[i][0];
			}
						
			voltageAvg = voltageAvgCalc / SHIFT_ARRAY;
			
			currentAvgCalc = 0;
			
			for(int i = 0; i < SHIFT_ARRAY; i++){
				currentAvgCalc += analogInAvg[i][1];
			}
						
			currentAvg = currentAvgCalc / SHIFT_ARRAY;
			
			delayAvgCalc = 0;
			
			for(int i = 0; i < SHIFT_ARRAY; i++){
				delayAvgCalc += analogInAvg[i][2];
			}
						
			delayAvg = delayAvgCalc / SHIFT_ARRAY;
			
			widthAvgCalc = 0;
			
			for(int i = 0; i < SHIFT_ARRAY; i++){
				widthAvgCalc += analogInAvg[i][3];
			}
						
			widthAvg = widthAvgCalc / SHIFT_ARRAY;
			//voltageAvg = (analogInAvg[0][0] + analogInAvg[1][1] + analogInAvg[2][0] + analogInAvg[3][0] + analogInAvg[4][0] + analogInAvg[5][0] + analogInAvg[6][0] + analogInAvg[7][0]) / 8;
			//currentAvg = (analogInAvg[0][1] + analogInAvg[1][1] + analogInAvg[2][1] + analogInAvg[3][1] + analogInAvg[4][1] + analogInAvg[5][1] + analogInAvg[6][1] + analogInAvg[7][1]) / 8;
			//delayAvg = (analogInAvg[0][2] + analogInAvg[1][2] + analogInAvg[2][2] + analogInAvg[3][2] + analogInAvg[4][2] + analogInAvg[5][2] + analogInAvg[6][2] + analogInAvg[7][2]) / 8;
			//widthAvg = (analogInAvg[0][3] + analogInAvg[1][3] + analogInAvg[2][3] + analogInAvg[3][3] + analogInAvg[4][3] + analogInAvg[5][3] + analogInAvg[6][3] + analogInAvg[7][3]) / 8;
			
			
			voltageBattery = (((float)MAX_BATTERY_VOLTAGE / 4095.0) * (float)voltageAvg); // to be verified
			currentBattery = (((float)MAX_BATTERY_CURRENT / 4095.0) * (float)currentAvg); // to be verified
			//delayPulse = (((MAX_PULSE_DELAY / 4095) * analogInputs[2]) * STEP_MULTIPLIER_DELAY);//analogInputs[2]); // to be verified 1000 = 1mS  * multiplier
			delayPulse =(((float) comparePulse[2] / 1000.0) * ((900.0 / 4095.0) * (float)delayAvg)); // rpm percentage delay
			
			if(modeWidthPulse == 1){
				// timed width
				widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)widthAvg) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
			}
			else{
				// duty cycle
				widthPulse = (((float)comparePulse[2] / 10000.0) * ((5000.0 / 4095.0) * (((float)widthAvg / 100.0) * multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
			}
			//comparePulse = 0;
			

			// PID
			if(!(GPIOB->IDR &(1<<2))){
				rpmDifference[1] = rpmDifference[0];
				rpmDifference[2] = (rpmDifference[0] + rpmDifference[1]) / 2;
				if(rpmPulse < (rpmSetPulse - MIN_BANDWIDTH_PID)){
					multiplierPid = MAX_TUNE_PID;
					rpmDifference[0] = rpmSetPulse - rpmPulse;
				}
				else if(rpmPulse > (rpmSetPulse + MAX_BANDWIDTH_PID)){
					multiplierPid = MIN_TUNE_PID;
					rpmDifference[0] = rpmPulse - rpmSetPulse;
				}
				else if((rpmPulse > (rpmSetPulse - MIN_BANDWIDTH_PID)) && (rpmPulse < (rpmSetPulse - DEADBAND_PID))){
					rpmDifference[0] = rpmSetPulse - rpmPulse;
					multiplierPid = 100 + rpmDifference[2]	;
					
					if(multiplierPid > MAX_TUNE_PID){
						multiplierPid = MAX_TUNE_PID;
					}
				}
				else if((rpmPulse < (rpmSetPulse + MAX_BANDWIDTH_PID)) && (rpmPulse > (rpmSetPulse + DEADBAND_PID))){
					rpmDifference[0] = rpmPulse - rpmSetPulse;
					multiplierPid = 100 - rpmDifference[2];
					
					if(multiplierPid > MAX_TUNE_PID){
						multiplierPid = MIN_TUNE_PID;
					}
				}
			}
			else{
				multiplierPid = 150;
			}
			
			
			if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 0)){
				motorRunState = 1;
			}
			else if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 1)){
				TIM3->ARR = 650000;
				__HAL_TIM_ENABLE(&htim3);
				
				
				comparePulse[2] = (comparePulse[0] + comparePulse[1]) / 2;
				if(comparePulse[2] > 0){
					rpmPulse = (((1000000.0 / (float)comparePulse[2]) * 60.0) / (float)MAGNETS_ON_ROTOR);
				}
			}
			
			else if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 2)){}
			else{
				motorRunState = 0;
			}
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 180-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

	TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_3,TIM_CCx_ENABLE);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : Hall_Trigger_In_Pin */
  GPIO_InitStruct.Pin = Hall_Trigger_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Hall_Trigger_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Pid_Active_Pin Motor_Enable_Pin */
  GPIO_InitStruct.Pin = Pid_Active_Pin|Motor_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	analogConvComplete = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  HAL_UART_Receive_IT(&huart5, Rx_data, 4); 
	dataCommand = 1;
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
