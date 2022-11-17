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

	TIM_HandleTypeDef htim2;
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

	int bandwidthPid = 10;
		
	//PID constants
	double kp = 10.0;
	double ki = 0.0001;
	double kd = 3.0;
	 
	float error;
	float lastError;
	float cumError, rateError;
		
	int PID_MIN = 1;
	int PID_MAX = 150;

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
	int MAX_COMPARE_PULSE = 10;
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

	const uint32_t DATA_CYCLE_COUNTER = 500;
		
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
		

	int pidEnabledSerial;
	uint32_t pidProportionalSerial;
	uint32_t pidIntegralSerial;
	uint32_t pidDerevitiveSerial;
	double pidSetProportionalSerial;
	double pidSetIntegralSerial;
	double pidSetDerevitiveSerial;
	uint32_t minTunePidSerial;
	uint32_t maxTunePidSerial;
	uint32_t rpmPulseSerial;  



	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_DMA_Init(void);
	static void MX_ADC1_Init(void);
	static void MX_UART5_Init(void);
	static void MX_ADC2_Init(void);
	static void MX_TIM3_Init(void);
	static void MX_TIM2_Init(void);
	/* USER CODE BEGIN PFP */

		void runPID(void);

	void handleMotorRunstate(void);
	void readAnalogConversion(void);
		void readBusConversion(void);
	void uartDataReceive(void);
	void uartDataTransmit(void);
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
		MX_TIM2_Init();
		/* USER CODE BEGIN 2 */
		// Init reference timer
		HAL_TIM_Base_Start(&htim2);
		// Init uart receive interrupt
		HAL_UART_Receive_IT (&huart5, Rx_data, 15);
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
		while (1)
		{
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			// check if motor is enabled and the motor is in running state
			if(((!(GPIOB->IDR &(1<<4)) && (analogValueMode == 0x00)) || (analogValueMode == 0x01) && (enablePulseSerial == 0x01)) && (motorRunState == 2)){
				
				// check if the pulse has passed
				if(pulseTrigger == 2){
					
					// take 2 pulses and calculate average
					for(int i = (MAX_COMPARE_PULSE - 1); i >= 0; i--){
						comparePulse[i] = comparePulse[(i-1)];
					}
					
					// save new pulse time
					comparePulse[0] = totalPulseTime;
					
					//reset pulse avarage
					comparePulseAvgTemp = 0;
					
					// count all values up
					for(int i = 0; i < (MAX_COMPARE_PULSE); i++){
						comparePulseAvgTemp += comparePulse[i];
					}
					
					// calculate pulse avarage and rpm if > 0
					if (comparePulseAvgTemp > 0){
						comparePulseAvg = (float)comparePulseAvgTemp / (float)MAX_COMPARE_PULSE;
						rpmPulse = (((1000000.0 / (float)comparePulseAvg) * 60.0) / (float)MAGNETS_ON_ROTOR);
					}
					else{
						rpmPulse = 0;
					}
					
					// PID
					if(((!(GPIOB->IDR &(1<<2))) && (analogValueMode  == 0x00)) || ((analogValueMode == 0x01) && (pidEnabledSerial == 0x01))){
						runPID();
					}
					else{
						multiplierPid = 150;
					}
					
					// reset pulse state
					pulseTrigger = 0;
				}

			}
				
			// read analog, do pid, handle motor state ever x cyclus time
			if( /*pulseTrigger == 0 &&*/ (TIM2->CNT >= (prevTimerAnalogCount + ANALOG_CYCLE_COUNTER))){
				
				// set new cycle time
				prevTimerAnalogCount = TIM2->CNT;
				
				// Handle Analog Values
				if(analogValueMode == 0x01){
					readBusConversion();
				}	
				else{
					readAnalogConversion();
				}
			}
			
			// check max timer value
			if((prevTimerAnalogCount > (comparePulse[0] - ANALOG_CYCLE_COUNTER)) || (prevTimerAnalogCount > (4294966 - ANALOG_CYCLE_COUNTER))){
				prevTimerAnalogCount = 0;
			}
			
			// Handle motor runstate
			handleMotorRunstate();

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
		TIM_OC_InitTypeDef sConfigOC = {0};

		/* USER CODE BEGIN TIM2_Init 1 */

		/* USER CODE END TIM2_Init 1 */
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 180-1;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 4294967;
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
		if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
		{
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		{
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
		TIM_SlaveConfigTypeDef sSlaveConfig = {0};
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
		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
		sSlaveConfig.InputTrigger = TIM_TS_ITR0;
		if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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

	void readBusConversion(){
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
			widthPulse = ((((float)MAX_PULSE_WIDTH / 4095.0) * (float)pulseWidthSerial) * (float)multiplierPid); // to be verified 1000 = 1mS  * multiplier
		}
		else{
			// duty cycle
			widthPulse = (((float)comparePulseAvg / 10000.0) * ((5000.0 / 4095.0) * (((float)pulseWidthSerial / 100.0) * (float)multiplierPid)));//(float) analogInputs[3])); // 0-50% duty cycle
		}
	}

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

		uartDataReceive(); 	
				
		__HAL_UART_CLEAR_OREFLAG(huart);
		huart->ErrorCode |= HAL_UART_ERROR_ORE;
				
		HAL_UART_Receive_IT(&huart5, Rx_data, 15);
		//HAL_UART_Receive_IT(&huart5, Rx_data, 15); 
		//dataCommand = 1;
	}

	void uartDataReceive(){
		int start = 0;
		for(int i = 0; i <= 15; i++){
			if ((Rx_data[i] == 0x80) || (Rx_data[i] == 0x70)){
				start = i;
			}
		}
		// if the uart command was a register write action
		if(Rx_data[start] == 0x80){
			switch(Rx_data[(start + 1)]){
				case 0x01: // enable bus control
					analogValueMode = Rx_data[(start + 5)]; // 0=analog 1=busvalue
					data1[start]= 0x80;
					data1[1]= 0x01;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(analogValueMode);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x02: // enable motor
					enablePulseSerial = Rx_data[(start + 5)];
					data1[0]= 0x80;
					data1[1]= 0x02;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(enablePulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			
					
				case 0x03: // enable pid
					pidEnabledSerial = Rx_data[(start + 5)]; // 0= disabled 1=standard coded pid values 2=bus controlled pid values
					data1[0]= 0x80;
					data1[1]= 0x03;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(pidEnabledSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
	
				case 0x05: // set rpm
					rpmPulseSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					data1[0]= 0x80;
					data1[1]= 0x05;
					data1[2]=(rpmPulseSerial >> 24);
					data1[3]=(rpmPulseSerial >> 16);
					data1[4]=(rpmPulseSerial >> 8);
					data1[5]=(rpmPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				
				case 0x06: // set mode width pulse
					modeWidthPulseSerial = Rx_data[(start + 5)];
					data1[0]= 0x80;
					data1[1]= 0x06;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(modeWidthPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				
				case 0x07: // set mode delay pulse
					modeDelayPulseSerial = Rx_data[(start + 5)];
					data1[0]= 0x80;
					data1[1]= 0x07;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(modeDelayPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			
				case 0x40: // set pulse width
					pulseWidthSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					data1[0]= 0x80;
					data1[1]= 0x40;
					data1[2]=(pulseWidthSerial >> 24);
					data1[3]=(pulseWidthSerial >> 16);
					data1[4]=(pulseWidthSerial >> 8);
					data1[5]=(pulseWidthSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x50: // set delay pulse
					delayPulseSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					data1[0]= 0x80;
					data1[1]= 0x50;
					data1[2]=(delayPulseSerial >> 24);
					data1[3]=(delayPulseSerial >> 16);
					data1[4]=(delayPulseSerial >> 8);
					data1[5]=(delayPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;

				case 0x60: // set pid proportional
					pidProportionalSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					pidSetProportionalSerial = (double)pidProportionalSerial / 10000.0;
					pidProportionalSerial = pidSetProportionalSerial * 10000;
					data1[0]= 0x80;
					data1[1]= 0x60;
					data1[2]=(pidProportionalSerial >> 24);
					data1[3]=(pidProportionalSerial >> 16);
					data1[4]=(pidProportionalSerial >> 8);
					data1[5]=(pidProportionalSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x61: // set pid integral
					pidIntegralSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					pidSetIntegralSerial = (double)pidIntegralSerial / 10000.0;
					pidIntegralSerial = pidSetIntegralSerial * 10000;
					data1[0]= 0x80;
					data1[1]= 0x61;
					data1[2]=(pidIntegralSerial >> 24);
					data1[3]=(pidIntegralSerial >> 16);
					data1[4]=(pidIntegralSerial >> 8);
					data1[5]=(pidIntegralSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x62: // set pid derevitive
					pidDerevitiveSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					pidSetDerevitiveSerial = (double)pidDerevitiveSerial / 10000.0;
					pidDerevitiveSerial = pidSetDerevitiveSerial * 10000;
					data1[0]= 0x80;
					data1[1]= 0x62;
					data1[2]=(pidDerevitiveSerial >> 24);
					data1[3]=(pidDerevitiveSerial >> 16);
					data1[4]=(pidDerevitiveSerial >> 8);
					data1[5]=(pidDerevitiveSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x63: // set pid min tune
					minTunePidSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					data1[0]= 0x80;
					data1[1]= 0x63;
					data1[2]=(minTunePidSerial >> 24);
					data1[3]=(minTunePidSerial >> 16);
					data1[4]=(minTunePidSerial >> 8);
					data1[5]=(minTunePidSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x64: // set pid max tune
					maxTunePidSerial = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
					data1[0]= 0x80;
					data1[1]= 0x64;
					data1[2]=(maxTunePidSerial >> 24);
					data1[3]=(maxTunePidSerial >> 16);
					data1[4]=(maxTunePidSerial >> 8);
					data1[5]=(maxTunePidSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			}
		}
		else if(Rx_data[start] == 0x70){ // read action on the bus
			switch(Rx_data[(start + 1)]){
				case 0x01: // enable bus control
					data1[0]= 0x70;
					data1[1]= 0x01;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(analogValueMode);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x02: // enable motor
					data1[0]= 0x70;
					data1[1]= 0x02;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(enablePulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			
				case 0x03: // enable pid
					//pidEnabledSerial = Rx_data[2]; // 0= disabled 1=standard coded pid values 2=bus controlled pid values
					data1[0]= 0x70;
					data1[1]= 0x03;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(pidEnabledSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
		
				case 0x05: // set rpm
					data1[0]= 0x70;
					data1[1]= 0x05;
					data1[2]=(rpmPulseSerial >> 24);
					data1[3]=(rpmPulseSerial >> 16);
					data1[4]=(rpmPulseSerial >> 8);
					data1[5]=(rpmPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				
				case 0x06: // set mode width pulse
					data1[0]= 0x70;
					data1[1]= 0x06;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(modeWidthPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
				
				case 0x07: // set mode delay pulse
					data1[0]= 0x70;
					data1[1]= 0x07;
					data1[2]=0x00;
					data1[3]=0x00;
					data1[4]=0x00;
					data1[5]=(modeDelayPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x10: // voltage
					data1[0]= 0x70;
					data1[1]= 0x10;
					data1[2]=(voltageBattery >> 24);
					data1[3]=(voltageBattery >> 16);
					data1[4]=(voltageBattery >> 8);
					data1[5]=(voltageBattery);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x20: // current
					data1[0]= 0x70;
					data1[1]= 0x20;
					data1[2]=(currentBattery >> 24);
					data1[3]=(currentBattery >> 16);
					data1[4]=(currentBattery >> 8);
					data1[5]=(currentBattery);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x30: // motor rpm
					data1[0]= 0x70;
					data1[1]= 0x30;
					data1[2]=(rpmPulse >> 24);
					data1[3]=(rpmPulse >> 16);
					data1[4]=(rpmPulse >> 8);
					data1[5]=(rpmPulse);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
			
				case 0x40: // set pulse width
					data1[0]= 0x70;
					data1[1]= 0x40;
					data1[2]=(pulseWidthSerial >> 24);
					data1[3]=(pulseWidthSerial >> 16);
					data1[4]=(pulseWidthSerial >> 8);
					data1[5]=(pulseWidthSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x50: // set delay pulse
					delayPulseSerial = (Rx_data[2] << 24) | (Rx_data[3] << 16) | (Rx_data[4] << 8) | (Rx_data[5]);
					data1[0]= 0x80;
					data1[1]= 0x50;
					data1[2]=(delayPulseSerial >> 24);
					data1[3]=(delayPulseSerial >> 16);
					data1[4]=(delayPulseSerial >> 8);
					data1[5]=(delayPulseSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x60: // set pid proportional
					data1[0]= 0x80;
					data1[1]= 0x60;
					data1[2]=((pidProportionalSerial * 10000) >> 24);
					data1[3]=((pidProportionalSerial * 10000) >> 16);
					data1[4]=((pidProportionalSerial * 10000) >> 8);
					data1[5]=((pidProportionalSerial * 10000));
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x61: // set pid integral
					data1[0]= 0x70;
					data1[1]= 0x61;
					data1[2]=((pidIntegralSerial * 10000) >> 24);
					data1[3]=((pidIntegralSerial * 10000) >> 16);
					data1[4]=((pidIntegralSerial * 10000) >> 8);
					data1[5]=((pidIntegralSerial * 10000));
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x62: // set pid derevitive
					data1[0]= 0x80;
					data1[1]= 0x62;
					data1[2]=((pidDerevitiveSerial * 10000) >> 24);
					data1[3]=((pidDerevitiveSerial * 10000) >> 16);
					data1[4]=((pidDerevitiveSerial * 10000) >> 8);
					data1[5]=((pidDerevitiveSerial * 10000));
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x63: // set pid min tune
					data1[0]= 0x80;
					data1[1]= 0x63;
					data1[2]=(minTunePidSerial >> 24);
					data1[3]=(minTunePidSerial >> 16);
					data1[4]=(minTunePidSerial >> 8);
					data1[5]=(minTunePidSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
					
				case 0x64: // set pid max tune
					data1[0]= 0x80;
					data1[1]= 0x64;
					data1[2]=(maxTunePidSerial >> 24);
					data1[3]=(maxTunePidSerial >> 16);
					data1[4]=(maxTunePidSerial >> 8);
					data1[5]=(maxTunePidSerial);
					data1[6]=0x11;
					data1[7]=0x12;
					HAL_UART_Transmit(&huart5,data1,sizeof(data1),10);
					break;
			}
		}
				
		// set all databits to 0
		for(int i = 0; i <= 15; i++){
			Rx_data[i] = 0x00;
		}
			
		// reset the data command flag
			dataCommand = 0;
	}

	void runPID(){ 
	
			//if((rpmPulse < (rpmSetPulse + bandwidthPid)) && (rpmPulse > (rpmSetPulse - bandwidthPid))){
			//	cumError = 0;
			//}
	
		if(pidEnabledSerial == 0x01){
			error = (float)rpmPulseSerial - (float)rpmPulse; // determine error
			cumError += error; // compute integral
			rateError = (error - lastError);
			multiplierPid = (pidSetProportionalSerial)*error + (pidSetIntegralSerial )*cumError + (pidSetDerevitiveSerial )*rateError;   
			lastError = error; 
			
			if(multiplierPid > maxTunePidSerial){
				multiplierPid = maxTunePidSerial;
			}
			else if(multiplierPid < minTunePidSerial){
				multiplierPid = minTunePidSerial;
			}
				
		}
		else{
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
	}
		
	void handleMotorRunstate(){
		if((((!(GPIOB->IDR &(1<<4))) && (analogValueMode == 0x00)) || ((analogValueMode == 0x01) && (enablePulseSerial == 0x01))) && (motorRunState == 0)){
			error = 0;
			cumError = 0;
			rateError = 0;
			motorRunState = 1;
		}
		else if(((analogValueMode == 0x00) && (!(GPIOB->IDR &(1<<4))) || ((analogValueMode == 0x01) && (enablePulseSerial == 0x01))) && (motorRunState == 1)){
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
		else if(((analogValueMode == 0x00) && (!(GPIOB->IDR &(1<<4))) || ((analogValueMode == 0x01) && (enablePulseSerial == 0x01))) && (motorRunState == 2)){
			// check if motor is still running
			if(TIM2->CNT > 2000000){
				motorRunState = 0;
				comparePulse[0] = 0;
			}
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
