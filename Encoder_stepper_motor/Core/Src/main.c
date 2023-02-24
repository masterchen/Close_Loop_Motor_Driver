/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/****************************** UART commands *****************/

uint8_t RxBuffer[50]; //UART command buffer
uint8_t RxIndex = 0;
uint8_t Exec_command = 0;
uint8_t rxByte;
char sending_data[50] = "Nan \n"; //
uint8_t RX_Char;
/**************************************************************/

/************ Z Stepper Motor varibles ************/


//const float steps_per_millimeters = 50000.0; // configure steps per millimeter
float targetPos = -1.0; //in millimeters

const uint32_t multiplier = 1000000;

const int32_t min_pos_mm = 0;
const float  max_pos_mm = 21* multiplier; //in millimeters
const float home_speed_mm = 1;
const float Acceleration_mm = 80 *50000;
int32_t min_pos = (int32_t ) (steps_per_millimeters * min_pos_mm);
int32_t max_pos = (int32_t ) (steps_per_millimeters * max_pos_mm);
int32_t home_speed = (int32_t) (steps_per_millimeters * home_speed_mm);

float Acceleration = Acceleration_mm*steps_per_millimeters; //stepsPerSecondSquared

int32_t Total_Steps;

//z homing variables
uint8_t Home_Pass = 0; // to make two pass of z homing
uint8_t HOMED = false;
/*******************************************************************/

/********** Encoder variables ************/

uint32_t Encoder_Count = 0;
//int16_t count;
int i = 0;
/*******************************************************************/

/************** Camera based variables *************/

// flag used to change the reaction for camera_input falling edge detection
// sending encoder values or toggle the LED colour
uint8_t flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void __delay_ms(int32_t);
//void HAL_GPIO_EXTI_Callback(uint16_t);

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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /****************************** Setting up IRQ ***********************/
	HAL_NVIC_DisableIRQ(Z_END_STOP_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(Z_reference_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(P_limit_EXTI_IRQn);
	HAL_NVIC_DisableIRQ(Q_limit_EXTI_IRQn);

	//HAL_NVIC_DisableIRQ(Camera_input_EXTI_IRQn);
	//let's enable the camera_input only

  /*****************************************************************/


/*********************** LED SPI Intialization *******************/

	//initializing LED values
	dectobin(ch1_br, &ch1_buff1, &ch1_buff2);
	dectobin(ch2_br, &ch2_buff1, &ch2_buff2);
	dectobin(ch3_br, &ch3_buff1, &ch3_buff2);
	//setting up channel select pin to HIGH
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);



/************************** UART Initialisation *********************/
	// initiating interrupt for PC data receiving
	HAL_UART_Receive_IT(&huart2, &rxByte, 1);



/************************** Encoder Intialization *******************/

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); //starting encoder



/*************************** Timer Intializations ****************/

  HAL_TIM_Base_Start(&htim6); //to calculate pulse time

  HAL_TIM_Base_Start(&htim7); // for microseconds delay



/*********************** Z motor initialization *********************/


	stepper_setup(STEP_GPIO_Port, STEP_Pin, DIR_GPIO_Port, DIR_Pin );

	//sample movement
	Total_Steps = (int32_t) ( steps_per_millimeters * targetPos );
	setMaxSpeed(10*steps_per_millimeters);
	setAcceleration(Acceleration);
	moveTo(Total_Steps);

	//runToPosition(); //moving to desired positions

/*********************************************************************/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(Exec_command){
	//			char str[] = "G91 Z30 F300 \r";
		int MAX_TOKENS = 3;
		char *token;
		char *tokens[3] = {0};
		int i = 0;


		/* Split the string by the delimiter " " */
		token = strtok((char *)RxBuffer, " ");

		while (token != NULL && i < MAX_TOKENS) {
			tokens[i] = token;
			i++;
			token = strtok(NULL, " ");
		}

		//Excute the command
		UART_Command(tokens);
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
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 40-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 40-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, DIR_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(End_stop_GPIO_Port, End_stop_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_Selection_GPIO_Port, SPI_Selection_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_reference_Pin P_limit_Pin Q_limit_Pin */
  GPIO_InitStruct.Pin = Z_reference_Pin|P_limit_Pin|Q_limit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : End_stop_Pin */
  GPIO_InitStruct.Pin = End_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(End_stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Z_END_STOP_Pin */
  GPIO_InitStruct.Pin = Z_END_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Z_END_STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Selection_Pin */
  GPIO_InitStruct.Pin = SPI_Selection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_Selection_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Camera_input_Pin */
  GPIO_InitStruct.Pin = Camera_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Camera_input_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( GPIO_Pin == Z_END_STOP_Pin ){
		/* Z_END_STOP detected ->
		 * stop the homing motor
		 *  reset the position
		 * Disable the interrupt for Z_END_STOP
		 *
		 */


		if(Home_Pass == 0){
			// first pass
			Home_Pass = 1;

			Homing_completion();

			move((int32_t ) (steps_per_millimeters * 0.25));
			runToPosition();
			//setting speed again to intiate the HOMING
			//otherwise _stepintervel became zero
			setSpeed(-1*home_speed); //MOVING IN ccw
			return;
		}
		else if(Home_Pass == 1){

			HOMED = true;
			Home_Pass = 0;

			move((int32_t ) (steps_per_millimeters * 0.25));
			runToPosition();
			Homing_completion();

		}


		Exec_command = 0; //stop the motor
		memset(RxBuffer,0,sizeof(RxBuffer));

		//homing configuration
		Homing_completion();
		//disabling the Interrupt for End stop

		//Set encoder counting to zero
		__HAL_TIM_SET_COUNTER(&htim2,0);

		HAL_NVIC_DisableIRQ(Z_END_STOP_EXTI_IRQn);

		//Sending completion
		memset(sending_data,0,sizeof(sending_data));
		sprintf(sending_data,"Homed \n");
		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

		return;

		}
	else if(GPIO_Pin == Camera_input_Pin){
		//Camera pulse detected

		//checking which operation have to do

		if(flag == 0){
		// PC in homing mode so sending encoder data to PC
		// for every one camera click encoder will be send to PC through UART
		memset(sending_data,0,sizeof(sending_data));
		sprintf(sending_data,"%ld\n",(int32_t)__HAL_TIM_GET_COUNTER(&htim2) );
		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);
		}

		else if(flag == 1){
			//PC in data capturing mode so toggle the led
			 switch(count){

			 case 0 :
				 // turning off all LEDs
				 stop_function(LED3);
				 // incrementing count to change the LED color in next time
				 count++;
				 break;
			 case 1:
				 //turning on 2nd LED only
				 stop_function(LED1);
				 send_function(LED2,&ch2_buff1,&ch2_buff2);
				 count++;// incrementing count to change the LED color in next time
				 break;
			 case 2:
				 //turning on 3rd LED only
				 stop_function(LED2); // turn off LED 2
				 send_function(LED3,&ch3_buff1,&ch3_buff2);// turn on LED 3
				 count = 0;// set count to zero to Off the LED in next time
				 break;
			 }

		}
	}


	else if( GPIO_Pin == Z_reference_Pin ){
	// Z_reference_detected.

		// pulse for end stop
		HAL_GPIO_WritePin(End_stop_GPIO_Port, End_stop_Pin, GPIO_PIN_RESET);
		__delay_ms(100); // custom delay function
		HAL_GPIO_WritePin(End_stop_GPIO_Port, End_stop_Pin, GPIO_PIN_SET);
		//homing completed

		//Set encoder counting to zero
		__HAL_TIM_SET_COUNTER(&htim2,0);
	}

	else if(GPIO_Pin == Q_limit_Pin){
	// Q_limit detected ( Lower Limit)
		//pulse for end stop
		HAL_GPIO_WritePin(End_stop_GPIO_Port, End_stop_Pin, GPIO_PIN_RESET);
		__delay_ms(100);
		HAL_GPIO_WritePin(End_stop_GPIO_Port, End_stop_Pin, GPIO_PIN_SET);
	}

	else if(GPIO_Pin == P_limit_Pin){
	// P_limit detected(Upper Limit)
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart == &huart2){
    /* Receive one byte in the receive data register */
//    uint8_t rxByte = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);

    /* Check if the received character is a /r or buffer is full */
    if (rxByte == '\r' || RxIndex == 50 - 1) // 50 is buffer length
    {
        /* Set flag to indicate command is complete */
        Exec_command = 1;

        /* Add null character to terminate string */
        RxBuffer[RxIndex] = 0;

        /* Resetting RxIndex to zero */
        RxIndex = 0;

        /* Restart the UART Interrupt */
		HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }

    else
    {
    	/****** Special Character Checking ***********/
    	if(rxByte == 'e' ){
		//sending encoder value through UART

		memset(sending_data,0,sizeof(sending_data));
		sprintf(sending_data,"%ld\n",__HAL_TIM_GET_COUNTER(&htim2) );
		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);


		 /* Restart the UART Interrupt */
		HAL_UART_Receive_IT(&huart2, &rxByte, 1);

		return;
    	}

        /* Add the character to the buffer */
        RxBuffer[RxIndex] = rxByte;
        RxIndex++;

        /* Restart the UART Interrupt */
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);

    }

	}
}



void __delay_ms(int32_t k){
	int32_t i,j;
	for(i=0;i<k;i++)
		for(j=0;j<3000;j++){}
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
  //sending the ready status
	memset(sending_data,0,sizeof(sending_data));
	sprintf(sending_data,"ERROR");
	HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

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
