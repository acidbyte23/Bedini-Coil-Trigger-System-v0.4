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
//#define UART_DATA // enable uart data comms
//#define WIFI_UART_DATA // enable uart over wifi
//#define BLUE_UART_DATA // enable uart over bluetooth
//#define RPM_PID // enable DIY PID
#define REAL_RPM_PID // enable REAL PID
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
volatile int modeDelayPulse = 0;

#if defined(RPM_PID) && !defined(REAL_RPM_PID)
	const uint32_t PID_CYCLE_COUNTER = 1000;
	
	// PID Settings
	int MIN_BANDWIDTH_PID = 100;
	int MAX_BANDWIDTH_PID = 100;
	int MAX_TUNE_PID = 150;
	int MIN_TUNE_PID = 0;
	int DEADBAND_PID = 0;
	const int MAX_SHIFT_PID = 3;
	float gainPid = 1.0;
	volatile uint32_t rpmDifference[MAX_SHIFT_PID];
	uint32_t rpmDifferenceAvg;
	uint32_t prevTimerPIDCount;
#elif defined(REAL_RPM_PID) && !defined(RPM_PID)
	
	//PID constants
	double kp = 2;
	double ki = 5;
	double kd = 1;
 
	float error;
	float lastError;
	float cumError, rateError;
	
	int PID_MIN = 1;
	int PID_MAX = 150;
#elif !defined(RPM_PID) && !defined(REAL_RPM_PID)
#else
    #error "User can only select 1 PID configuration"
#endif


uint32_t ANALOG_CYCLE_COUNTER = 1000;

// Analog Inputs
const int SHIFT_ARRAY = 20;
const int VOLTAGE_SHIFT = 5;
const int CURRENT_SHIFT = 5;
const int DELAY_SHIFT = 20;
const int WIDTH_SHIFT = 20;
volatile uint32_t analogInputs[4];
volatile uint32_t analogInAvg[SHIFT_ARRAY][4];
const int analogChCounts = sizeof ( analogInputs) / sizeof (analogInputs[0]);
volatile int analogConvComplete = 0;
uint32_t prevTimerAnalogCount;

// Proram Vars
volatile uint32_t	multiplierPid;
volatile uint32_t delayPulse = 0;
volatile uint32_t widthPulse = 0;
int MAX_COMPARE_PULSE = 5;
const int MAXSET_COMPARE_PULSE = 20;
volatile uint32_t comparePulse[MAXSET_COMPARE_PULSE];
volatile uint32_t comparePulseAvg;
volatile uint32_t rpmPulse = 0;
volatile int multiplierPulse = 100;
volatile uint32_t voltageAvgCalc;
volatile uint32_t currentAvgCalc;
volatile uint32_t delayAvgCalc;
volatile uint32_t widthAvgCalc;
volatile uint32_t voltageAvg;
volatile uint32_t currentAvg;
volatile uint32_t delayAvg;
volatile uint32_t widthAvg;
volatile uint32_t voltageBattery = 0;
volatile uint32_t currentBattery = 0;
volatile int pulseTrigger;
volatile int motorRunState = 0;
volatile uint32_t comparePulseAvgTemp;
volatile uint32_t totalPulseTime;

#if defined(UART_DATA) && !defined(WIFI_UART_DATA) && !defined(BLUE_UART_DATA)
	const uint32_t DATA_CYCLE_COUNTER = 1000;
	
	// uart data stuf
	uint32_t dataCounter;
	uint8_t data1[8];
	uint32_t prevTimerDataCount;
	uint8_t Rx_data[16];
	uint8_t dataCommand;

	// Serial vars
	int SerialEnable;
	int enablePulseSerial; 	
	int analogValueMode;
	int modeWidthPulseSerial;
	int modeDelayPulseSerial;
	uint32_t pulseWidthSerial;
	uint32_t delayPulseSerial;
	
	#if defined(RPM_PID)
		int pidEnabledSerial;
		uint32_t deadbandPidSerial;
		uint32_t minBandwidthPidSerial;
		uint32_t maxBandwidthPidSerial;
		uint32_t minTunePidSerial;
		uint32_t maxTunePidSerial;
		uint32_t rpmPulseSerial;
	#elif defined(REAL_RPM_PID)   
		//TODO
	#endif
	
#elif defined(WIFI_UART_DATA) && !defined(UART_DATA) && !defined(BLUE_UART_DATA)
	const uint32_t DATA_CYCLE_COUNTER = 1000;
	
	// uart data stuf
	uint32_t dataCounter;
	uint8_t data1[8];
	uint32_t prevTimerDataCount;
	uint8_t Rx_data[16];
	uint8_t dataCommand;

	// WIFI AT COMMANDS 
	uint8_t SET_MODE_WIFI[] = "AT+CWMODE=1";
	uint8_t CONNECT_WIFI[] = "AT+CWJAP=\"Free Energy\",\"Internet23!\""; // connect to wifi access point
	uint8_t OK_WIFI[] = "AT";	// AT command
	uint8_t IP_ADRES_WIFI[] = "AT+CIPSTA=192.168.2.200";  // set ip adress
	uint8_t MULTI_CONNECT_WIFI[] = "AT+CIPMUX=1"; // allow multiple connections
	uint8_t RUN_SERVER_WIFI[] = "AT+CIPSERVER=1,369"; // run tcp server on port 369
	uint8_t OPEN_PORT_WIFI[] = "AT+CIPSTART=0,\"UDP\",\"192.168.2.5\",369,369,2"; // open the port 369 at 192.168.2.5
	uint8_t SEND_MSG_8_WIFI[] = "AT+CIPSEND=0,8,\"192.168.2.5\",8100"; // send msg at port 369 to 192.168.2.5 with length of 8
	uint8_t SEND_MSG_16_WIFI[] = "AT+CIPSEND=0,16,\"192.168.2.5\",8100"; // send msg at port 369 to 192.168.2.5 with length of 16

	// wifi vars
	int wifiEnable;
	int enablePulseWifi; 	
	int analogValueMode;
	int modeWidthPulseWifi;
	int modeDelayPulseWifi;
	uint32_t pulseWidthWifi;
	uint32_t delayPulseWifi;
	
	#if defined(RPM_PID)
		int pidEnabledWifi;
		uint32_t deadbandPidWifi;
		uint32_t minBandwidthPidWifi;
		uint32_t maxBandwidthPidWifi;
		uint32_t minTunePidWifi;
		uint32_t maxTunePidWifi;
		uint32_t rpmPulseWifi;
	#elif defined(REAL_RPM_PID)  
		//TODO 
	#endif
	
#elif defined(BLUE_UART_DATA) && !defined(UART_DATA) && !defined(WIFI_UART_DATA)
#elif !defined(UART_DATA) && !defined(WIFI_UART_DATA) && !defined(BLUE_UART_DATA)
#else
	#error "User can only select 1 UART configuration"
#endif

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

#if defined(RPM_PID)
	void runPID(void);
#elif defined(REAL_RPM_PID)   
	void runPID(void);
#endif

void handleMotorRunstate(void);
void readAnalogConversion(void);
#if defined(UART_DATA) || defined(WIFI_UART_DATA) || defined(BLUE_UART_DATA)
	void readBusConversion(void);
#endif
#if defined(UART_DATA)
void uartDataReceive(void);
void uartDataTransmit(void);
#elif defined(WIFI_UART_DATA)
void uartDataReceive(void);
void uartDataTransmit(void);
void initSerialWifi(void);
#elif defined(BLUE_UART_DATA)
void uartDataReceive(void);
void uartDataTransmit(void);
void initSerialBLuetooth(void);
#endif
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
	// Init uart receive interrupt
#if defined(UART_DATA)
	HAL_UART_Receive_IT (&huart5, Rx_data, 4);
#elif defined(WIFI_UART_DATA)
	HAL_UART_Receive_IT (&huart5, Rx_data, 4);
	initSerialWifi();
#elif defined(BLUE_UART_DATA)
	HAL_UART_Receive_IT (&huart5, Rx_data, 4);
		initSerialBluetooth();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// check if motor is enabled and the motor is in running state
		if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 2)){
			// check if the pulse has passed
			if(pulseTrigger == 2){
				// take 2 pulses and calculate average
				for(int i = (MAX_COMPARE_PULSE - 1); i >= 0; i--){
					comparePulse[i] = comparePulse[(i-1)];
				}
				comparePulse[0] = totalPulseTime;
				
				comparePulseAvgTemp = 0;
				
				for(int i = 0; i < (MAX_COMPARE_PULSE); i++){
					comparePulseAvgTemp += comparePulse[i];
				}
				
				if (comparePulseAvgTemp > 0){
					comparePulseAvg = (float)comparePulseAvgTemp / (float)MAX_COMPARE_PULSE;
					rpmPulse = (((1000000.0 / (float)comparePulseAvg) * 60.0) / (float)MAGNETS_ON_ROTOR);
				}
				else{
					rpmPulse = 0;
				}
				
				#if defined(REAL_RPM_PID)       
					// PID
					if(!(GPIOB->IDR &(1<<2))){
						runPID();
					}
					else{
						multiplierPid = 150;
					}
				#endif	
				
				// reset pulse state
				pulseTrigger = 0;
			}

			// if time passed delay of pulse and a pulse has to be fired
			if((TIM1->CNT >= delayPulse) && (pulseTrigger == 1)){
				// divide pulsewidth by 4 and pass to timer register
				TIM3->ARR = widthPulse / 3;
				// enable one shot timer
				__HAL_TIM_ENABLE(&htim3);
				// set new pulse state
				pulseTrigger = 2;
			}
		}

#if defined(UART_DATA)
		// Check uart for data
		if((dataCommand == 1)){
			uartDataReceive();
		}
		
		// if there is no pulse running and timer has passed the data cyclus time
		if((pulseTrigger == 0) && (TIM1->CNT >= (prevTimerDataCount + DATA_CYCLE_COUNTER))){
			// save new timer value
			prevTimerDataCount = TIM1->CNT;
			
			uartDataTransmit();
		
		}

		if((prevTimerDataCount > (comparePulse[0] - DATA_CYCLE_COUNTER)) || (prevTimerDataCount > (65534 - DATA_CYCLE_COUNTER))){
			prevTimerDataCount = 0;
		}
#elif defined(WIFI_UART_DATA)
		// Check uart for data
		if((wifiEnable == 1) && (dataCommand == 1)){
			uartDataReceive();
		}
		
		// if there is no pulse running and timer has passed the data cyclus time
		if((wifiEnable == 1) && (pulseTrigger == 0) && (TIM1->CNT >= (prevTimerDataCount + DATA_CYCLE_COUNTER))){
			// save new timer value
			prevTimerDataCount = TIM1->CNT;
			
			uartDataTransmit();
		
		}

		if((prevTimerDataCount > (comparePulse[0] - DATA_CYCLE_COUNTER)) || (prevTimerDataCount > (65534 - DATA_CYCLE_COUNTER))){
			prevTimerDataCount = 0;
		}
#elif defined(BLUE_UART_DATA)
		// Check uart for data
		if((bluetoothEnable == 1) && (dataCommand == 1)){
			uartDataReceive();
		}
		
		// if there is no pulse running and timer has passed the data cyclus time
		if((bluetoothEnable == 1) && (pulseTrigger == 0) && (TIM1->CNT >= (prevTimerDataCount + DATA_CYCLE_COUNTER))){
			// save new timer value
			prevTimerDataCount = TIM1->CNT;
			
			uartDataTransmit();
		
		}

		if((prevTimerDataCount > (comparePulse[0] - DATA_CYCLE_COUNTER)) || (prevTimerDataCount > (65534 - DATA_CYCLE_COUNTER))){
			prevTimerDataCount = 0;
		}
#endif
			
		// read analog, do pid, handle motor state ever x cyclus time
		if( /*pulseTrigger == 0 &&*/ (TIM1->CNT >= (prevTimerAnalogCount + ANALOG_CYCLE_COUNTER))){
			// set new cycle time
			prevTimerAnalogCount = TIM1->CNT;
			
			// Handle Analog Values
			
#if defined(UART_DATA) || defined(WIFI_UART_DATA) || defined(BLUE_UART_DATA)
			if(analogValueMode == 0x01){
				readBusConversion();
			}	


			else{
				readAnalogConversion();
			}
#else
			readAnalogConversion();
#endif
		}
		
		// Handle motor runstate
			handleMotorRunstate();
		
		if((prevTimerAnalogCount > (comparePulse[0] - ANALOG_CYCLE_COUNTER)) || (prevTimerAnalogCount > (65534 - ANALOG_CYCLE_COUNTER))){
			prevTimerAnalogCount = 0;
		}
		
#if defined(RPM_PID)
		if(TIM1->CNT >= (prevTimerPIDCount + PID_CYCLE_COUNTER)){
			prevTimerPIDCount = TIM1->CNT;
			
			// PID
			if(!(GPIOB->IDR &(1<<2))){
				runPID();
			}
			else{
				multiplierPid = 150;
			}
		}
		
		if((prevTimerPIDCount > (comparePulse[0] - PID_CYCLE_COUNTER)) || (prevTimerPIDCount > (65534 - PID_CYCLE_COUNTER))){
			prevTimerPIDCount = 0;
		}
#elif !defined(RPM_PID) && !defined(REAL_RPM_PID)
	multiplierPid = 150;
#endif
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
  sConfig.Channel = ADC_CHANNEL_11;
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
	//11500 for serial and wifi
	//38400 for bluetooth
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

	// Connect to the wifi network
#if defined(WIFI_UART_DATA)
	initSerialWifi();
#elif defined(BLUE_UART_DATA)
	initSerialBluetooth();
#endif
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	analogConvComplete = 1;
}

#if defined(UART_DATA) || defined(WIFI_UART_DATA) || defined(BLUE_UART_DATA)
	void readAnalogConversion(){
		// start a adc poll
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analogInputs, analogChCounts);
			
		// hold program till conversion is complete
		while(analogConvComplete == 0){			}
			
		// reset analog conversion flag
		analogConvComplete = 0;
			
		analogInAvg[0][0] = analogInputs[0];
		analogInAvg[0][1] = analogInputs[1];

	
		// shift analog input array
		for(int i = (SHIFT_ARRAY - 1);i >= 0;i--){
			for(int ii = 0;ii <= 1;ii++){
				analogInAvg[i][ii] = analogInAvg[(i-1)][ii];
			}
		}
			
		// reset voltage average
		voltageAvgCalc = 0;
			
		// add all voltage registers together
		for(int i = 0; i < (VOLTAGE_SHIFT); i++){
			voltageAvgCalc += analogInAvg[i][0];
		}
							
		// divide by the registers
		voltageAvg = voltageAvgCalc / VOLTAGE_SHIFT;
			
		// reset current average
		currentAvgCalc = 0;
			
		// add all current registers together
		for(int i = 0; i < (CURRENT_SHIFT); i++){
			currentAvgCalc += analogInAvg[i][1];
		}
			
		// divide by the registers		
		currentAvg = currentAvgCalc / CURRENT_SHIFT;
		
		// calculate battery voltage
		if (voltageAvg > 0){
			voltageBattery = (((float)MAX_BATTERY_VOLTAGE / 4095.0) * (float)voltageAvg); // to be verified
		}
		else{
			voltageBattery = 0;
		}
			
		// calculate battery current
		if (currentAvg > 0){
			currentBattery = (((float)MAX_BATTERY_CURRENT / 4095.0) * (float)currentAvg); // to be verified
		}
		else{
			currentBattery = 0;
		}
			
#if defined(UART_DATA)
		// calculate delay pulse
		// select pulse delay mode
		if(modeDelayPulse == 1){
			delayPulse = ((((float)MAX_PULSE_DELAY / 4095.0) * (float)delayPulseSerial) * (float)STEP_MULTIPLIER_DELAY);//analogInputs[2]); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			delayPulse =(((float) comparePulseAvg / 1000.0) * ((900.0 / 4095.0) * (float)delayPulseSerial)); // rpm percentage delay
		}
		
		// select pulse width mode
		if(modeWidthPulse == 1){
			// timed width
			widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)modeWidthPulseSerial) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			// duty cycle
			widthPulse = (((float)comparePulseAvg / 10000.0) * ((5000.0 / 4095.0) * (((float)modeWidthPulseSerial / 100.0) * (float)multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
		}
#elif defined(WIFI_UART_DATA)
		// calculate delay pulse
		// select pulse delay mode
		if(modeDelayPulse == 1){
			delayPulse = ((((float)MAX_PULSE_DELAY / 4095.0) * (float)delayPulseWifi) * (float)STEP_MULTIPLIER_DELAY);//analogInputs[2]); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			delayPulse =(((float) comparePulseAvg / 1000.0) * ((900.0 / 4095.0) * (float)delayPulseWifi)); // rpm percentage delay
		}
		
		// select pulse width mode
		if(modeWidthPulse == 1){
			// timed width
			widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)modeWidthPulseWifi) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			// duty cycle
			widthPulse = (((float)comparePulseAvg / 10000.0) * ((5000.0 / 4095.0) * (((float)modeWidthPulseWifi / 100.0) * (float)multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
		}
#elif defined(BLUE_UART_DATA)
		// calculate delay pulse
		// select pulse delay mode
		if(modeDelayPulse == 1){
			delayPulse = ((((float)MAX_PULSE_DELAY / 4095.0) * (float)delayPulseBluetooth) * (float)STEP_MULTIPLIER_DELAY);//analogInputs[2]); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			delayPulse =(((float) comparePulseAvg / 1000.0) * ((900.0 / 4095.0) * (float)delayPulseBluetooth)); // rpm percentage delay
		}
		
		// select pulse width mode
		if(modeWidthPulse == 1){
			// timed width
			widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)modeWidthPulseBluetooth) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			// duty cycle
			widthPulse = (((float)comparePulseAvg / 10000.0) * ((5000.0 / 4095.0) * (((float)modeWidthPulseBluetooth / 100.0) * (float)multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
		}
#endif		
	}
#endif

#if defined(UART_DATA)
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
	{
		HAL_UART_Receive_IT(&huart5, Rx_data, 4); 
		dataCommand = 1;
	}

	void uartDataReceive(){
		// if the uart command was a register write action
		if(Rx_data[0] == 0x80){
			switch(Rx_data[1]){
				case 0x01: // enable bus control
					analogValueMode = Rx_data[2]; // 0=analog 1=busvalue
					break;
				
				case 0x02: // enable motor
					enablePulseSerial = Rx_data[2];
					break;
		
	#if defined(RPM_PID)
				case 0x03: // enable pid
					pidEnabledSerial = Rx_data[2]; // 0= disabled 1=standard coded pid values 2=bus controlled pid values
					break;
				case 0x05: // set rpm
					rpmPulseSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
	#elif defined(REAL_RPM_PID)
				case 0x03: // enable pid
					pidEnabledSerial = Rx_data[2]; // 0= disabled 1=standard coded pid values 2=bus controlled pid values
					break;
				case 0x05: // set rpm
					rpmPulseSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
	#endif		
			
				case 0x06: // set mode width pulse
					modeWidthPulseSerial = Rx_data[2];
					break;
			
				case 0x07: // set mode delay pulse
					modeDelayPulseSerial = Rx_data[2];
					break;
		
				case 0x40: // set pulse width
					pulseWidthSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x50: // set delay pulse
					delayPulseSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
	#if defined(RPM_PID)				
				case 0x60: // set pid deadband
					deadbandPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x61: // set pid min bandwidth
					minBandwidthPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x62: // set pid max bandwidth
					maxBandwidthPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x63: // set pid min tune
					minTunePidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x64: // set pid max tune
					maxTunePidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
		#elif defined(REAL_RPM_PID)				
				case 0x60: // set pid deadband
					deadbandPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x61: // set pid min bandwidth
					minBandwidthPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x62: // set pid max bandwidth
					maxBandwidthPidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x63: // set pid min tune
					minTunePidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
				
				case 0x64: // set pid max tune
					maxTunePidSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					break;
		#endif
			}
		}
			
		// set all databits to 0
		for(int i = 0; i < 8; i++){
				Rx_data[i] = 0x00;
		}
		
		// reset the data command flag
		dataCommand = 0;
	}

	void uartDataTransmit(){
		// select a bit of data
		switch(dataCounter){
			case 0:
				//Send motor data over uart
				// battery voltage
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
				// battery current
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
				// motor rpm
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
				// calculated pulse width
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
				// calculate pulse delay
				data1[0]= 0x70;
				data1[1]= 0x50;
				data1[2]=(delayPulse >> 24);
				data1[3]=(delayPulse >> 16);
				data1[4]=(delayPulse >> 8);
				data1[5]=(delayPulse);
				HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
				break;
		}
			
		// increase data counter
		dataCounter++;
			
		// handle counter overflow
		if(dataCounter >= 5){
			dataCounter = 0;
		} 
	}
#elif defined(WIFI_UART_DATA)
#elif defined(BLUE_UART_DATA)
#endif

	
#if defined(RPM_PID)
	void runPID(){
			for(int i = (MAX_SHIFT_PID - 1); i >= 0; i--){
				rpmDifference[i] = rpmDifference[(i - 1)];
			}
				
			rpmDifferenceAvg = 0;
				
			for(int i = 0; i <  (MAX_SHIFT_PID); i++){
				rpmDifferenceAvg += rpmDifference[i];
			}
				
			rpmDifferenceAvg = (float)rpmDifferenceAvg  / (float)MAX_SHIFT_PID;
				
			if(rpmPulse < (rpmSetPulse - MIN_BANDWIDTH_PID)){
				rpmDifference[0] = rpmSetPulse - rpmPulse;
			}
			else if(rpmPulse > (rpmSetPulse + MAX_BANDWIDTH_PID)){
				multiplierPid = MIN_TUNE_PID;
				rpmDifference[0] = rpmPulse - rpmSetPulse;
			}
			else if((rpmPulse > (rpmSetPulse - MIN_BANDWIDTH_PID)) && (rpmPulse < (rpmSetPulse - DEADBAND_PID))){
				rpmDifference[0] = rpmSetPulse - rpmPulse;
				multiplierPid = (100.0 + (float)rpmDifferenceAvg) * gainPid;
					
				if(multiplierPid > MAX_TUNE_PID){
					multiplierPid = MAX_TUNE_PID;
				}
			}
			else if((rpmPulse < (rpmSetPulse + MAX_BANDWIDTH_PID)) && (rpmPulse > (rpmSetPulse + DEADBAND_PID))){
				rpmDifference[0] = rpmPulse - rpmSetPulse;
				multiplierPid = (100.0 - (float)rpmDifferenceAvg) / gainPid;
				
			if(multiplierPid > MAX_TUNE_PID){
				multiplierPid = MIN_TUNE_PID;
			}
		}
	}
#elif defined(REAL_RPM_PID)
	void runPID(){ 
		error = (float)rpmSetPulse - (float)rpmPulse; // determine error
		cumError += error; // compute integral
			rateError = (error - lastError);
		
		multiplierPid = kp*error + ki*cumError + kd*rateError;   
		lastError = error; 
		
		if(multiplierPid > PID_MAX){
			multiplierPid = PID_MAX;
		}
		else if(multiplierPid < PID_MIN){
			multiplierPid = PID_MIN;
		}
	}
#endif
	
void handleMotorRunstate(){
	if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 0)){
#if defined(REAL_RPM_PID)
		error = 0;
		cumError = 0;
		rateError = 0;
#endif
		motorRunState = 1;
	}
	else if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 1)){
		TIM3->ARR = 650000;
		__HAL_TIM_ENABLE(&htim3);
			
		for(int i = (MAX_COMPARE_PULSE - 1); i >= 0; i--){
			comparePulse[i] = comparePulse[(i-1)];
		}
			
		comparePulseAvg = 0;
				
		for(int i = 0; i < (MAX_COMPARE_PULSE); i++){
			comparePulseAvg += comparePulse[i];
		}
				
		comparePulseAvg = (float)comparePulseAvg / (float)MAX_COMPARE_PULSE;
				
		if(comparePulseAvg > 0){
			rpmPulse = (((1000000.0 / (float)comparePulse[0]) * 60.0) / (float)MAGNETS_ON_ROTOR);
		}
	}
	else if(!(GPIOB->IDR &(1<<4)) && (motorRunState == 2)){
		// check if motor is still running
		if(TIM1->CNT > 65000){
			motorRunState = 0;
			comparePulse[0] = 0;
		}
		
		//for(int i = (MAX_COMPARE_PULSE - 1); i >= 0; i--){
		//	comparePulse[i] = comparePulse[(i-1)];
		//}
			
		//comparePulseAvg = 0;
				
		//for(int i = 0; i < (MAX_COMPARE_PULSE); i++){
		//	comparePulseAvg += comparePulse[i];
		//}
				
		//comparePulseAvg = (float)comparePulseAvg / (float)MAX_COMPARE_PULSE;
				
		//if(comparePulseAvg > 0){
		//	rpmPulse = (((1000000.0 / (float)comparePulse[0]) * 60.0) / (float)MAGNETS_ON_ROTOR);
		//}
	}
	else{
		motorRunState = 0;
	}
}

void readAnalogConversion(){
	// start a adc poll
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analogInputs, analogChCounts);
			
	// hold program till conversion is complete
	while(analogConvComplete == 0){			}
			
	// reset analog conversion flag
	analogConvComplete = 0;
			
	analogInAvg[0][0] = analogInputs[0];
	analogInAvg[0][1] = analogInputs[1];
	analogInAvg[0][2] = analogInputs[2];
	analogInAvg[0][3] = analogInputs[3];
	
	// shift analog input array
	for(int i = (SHIFT_ARRAY - 1);i >= 0;i--){
		for(int ii = 0;ii <= 3;ii++){
			analogInAvg[i][ii] = analogInAvg[(i-1)][ii];
		}
	}
			
	// reset voltage average
	voltageAvgCalc = 0;
			
	// add all voltage registers together
	for(int i = 0; i < (VOLTAGE_SHIFT); i++){
		voltageAvgCalc += analogInAvg[i][0];
	}
							
	// divide by the registers
	voltageAvg = voltageAvgCalc / VOLTAGE_SHIFT;
			
	// reset current average
	currentAvgCalc = 0;
			
	// add all current registers together
	for(int i = 0; i < (CURRENT_SHIFT); i++){
		currentAvgCalc += analogInAvg[i][1];
	}
			
	// divide by the registers		
	currentAvg = currentAvgCalc / CURRENT_SHIFT;
			
	// reset delay average
	delayAvgCalc = 0;
			
	// add all delay registers
	for(int i = 0; i < (DELAY_SHIFT); i++){
		delayAvgCalc += analogInAvg[i][2];
	}
					
	// divide by the registers	
	delayAvg = delayAvgCalc / DELAY_SHIFT;
			
	// reset width average
	widthAvgCalc = 0;
			
	// add all width registers
	for(int i = 0; i < (WIDTH_SHIFT); i++){
		widthAvgCalc += analogInAvg[i][3];
	}
			
	// divide by the registers				
	widthAvg = widthAvgCalc / WIDTH_SHIFT;
		
	// calculate battery voltage
	if (voltageAvg > 0){
		voltageBattery = (((float)MAX_BATTERY_VOLTAGE / 4095.0) * (float)voltageAvg); // to be verified
	}
	else{
		voltageBattery = 0;
	}
			
	// calculate battery current
	if (currentAvg > 0){
		currentBattery = (((float)MAX_BATTERY_CURRENT / 4095.0) * (float)currentAvg); // to be verified
	}
	else{
		currentBattery = 0;
	}
			
	// calculate delay pulse
		// select pulse delay mode
	if(modeDelayPulse == 1){
		delayPulse = ((((float)MAX_PULSE_DELAY / 4095.0) * (float)analogInputs[2]) * (float)STEP_MULTIPLIER_DELAY);//analogInputs[2]); // to be verified 1000 = 1mS  * multiplier
	}
	else{
		delayPulse =(((float) comparePulseAvg / 1000.0) * ((900.0 / 4095.0) * (float)delayAvg)); // rpm percentage delay
	}
		
	// select pulse width mode
	if(modeWidthPulse == 1){
		// timed width
		widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)widthAvg) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
	}
	else{
		// duty cycle
		widthPulse = (((float)comparePulseAvg / 10000.0) * ((5000.0 / 4095.0) * (((float)widthAvg / 100.0) * (float)multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
	}
}

#if defined(WIFI_UART_DATA)
	void initSerialWifi(){
		HAL_Delay(500);
	
		// Check if wifi module responses
		HAL_UART_Transmit(&huart5,OK_WIFI,sizeof(OK_WIFI),10);
		TIM1->CNT = 0;
		while(dataCommand == 0){
			if(TIM1->CNT > 65000){
				goto bailout;
			}
		}
	
		// enable wifi is response is "OK"
		if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
			wifiEnable = 1;
		}
		else{
			wifiEnable = 0;
		}
		// set all databits to 0
		for(int i = 0; i < 8; i++){
				Rx_data[i] = 0x00;
		}
	
		HAL_Delay(100);
	
		// if wifi is enabled
		if(wifiEnable == 1){
		
			// set chip mode to station/ap
			HAL_UART_Transmit(&huart5,SET_MODE_WIFI,sizeof(SET_MODE_WIFI),10);
			TIM1->CNT = 0;
			while(dataCommand == 0){
				if(TIM1->CNT > 65000){
					goto bailout;
				}
			}
			
			// check message to be "OK"
			if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
				// wifi mode is set
			}
			else{
				goto bailout;
			}
			// set all databits to 0
			for(int i = 0; i < 8; i++){
					Rx_data[i] = 0x00;
			}
			HAL_Delay(100);
		
			// connect to access point
			HAL_UART_Transmit(&huart5,CONNECT_WIFI,sizeof(CONNECT_WIFI),10);
			TIM1->CNT = 0;
			while(dataCommand == 0){
				if(TIM1->CNT > 65000){
					goto bailout;
				}
			}
			// check message to be "OK"
			if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
				// wifi is connected
			}
			else{
				goto bailout;
			}	
			// set all databits to 0
			for(int i = 0; i < 8; i++){
					Rx_data[i] = 0x00;
			}
			HAL_Delay(100);
		
			// set ip adress
			HAL_UART_Transmit(&huart5, IP_ADRES_WIFI, sizeof(IP_ADRES_WIFI), 10);
			TIM1->CNT = 0;
			while(dataCommand == 0){
				if(TIM1->CNT > 65000){
					goto bailout;
				}
			}
			// check message to be "OK"
			if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
				// ip adress is set
			}
			else{
				goto bailout;
			}
			// set all databits to 0
			for(int i = 0; i < 8; i++){
					Rx_data[i] = 0x00;
			}
			HAL_Delay(100);
			
			// set multiple connections mode
			HAL_UART_Transmit(&huart5, MULTI_CONNECT_WIFI, sizeof(MULTI_CONNECT_WIFI), 10);
			TIM1->CNT = 0;
			while(dataCommand == 0){
				if(TIM1->CNT > 65000){
					goto bailout;
				}
			}
			// check message to be "OK"
			if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
				// Multi connect is set
			}
			else{
				goto bailout;
			}
			// set all databits to 0
			for(int i = 0; i < 8; i++){
					Rx_data[i] = 0x00;
			}
			HAL_Delay(100);
			
			// start a tcp server at port x
			HAL_UART_Transmit(&huart5, RUN_SERVER_WIFI, sizeof(RUN_SERVER_WIFI), 10);
			TIM1->CNT = 0;
			while(dataCommand == 0){
				if(TIM1->CNT > 65000){
					goto bailout;
				}
			}
			
			// check message to be "OK"
			if((Rx_data[0] == 0x4F) && (Rx_data[1] == 0x4B)){
				//Server is running
			}	
			else{
				goto bailout;
			}
			// set all databits to 0
			for(int i = 0; i < 8; i++){
					Rx_data[i] = 0x00;
			}
			HAL_Delay(100);
		
		
		}
	bailout:
			HAL_Delay(100);
	}
#elif defined(BLUE_UART_DATA)
#endif
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
