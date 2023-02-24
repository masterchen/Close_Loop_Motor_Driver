/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "stepper.h"
#include "GCode_helper.h"
#include "UART_helper.h"
#include "LedDriver.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_0
#define DIR_GPIO_Port GPIOC
#define STEP_Pin GPIO_PIN_1
#define STEP_GPIO_Port GPIOC
#define A_signal_Pin GPIO_PIN_0
#define A_signal_GPIO_Port GPIOA
#define B_signal_Pin GPIO_PIN_1
#define B_signal_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SPI_Clk_Pin GPIO_PIN_5
#define SPI_Clk_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define Z_reference_Pin GPIO_PIN_0
#define Z_reference_GPIO_Port GPIOB
#define Z_reference_EXTI_IRQn EXTI0_IRQn
#define End_stop_Pin GPIO_PIN_10
#define End_stop_GPIO_Port GPIOB
#define Z_END_STOP_Pin GPIO_PIN_8
#define Z_END_STOP_GPIO_Port GPIOA
#define Z_END_STOP_EXTI_IRQn EXTI9_5_IRQn
#define SPI_Selection_Pin GPIO_PIN_9
#define SPI_Selection_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Camera_input_Pin GPIO_PIN_3
#define Camera_input_GPIO_Port GPIOB
#define Camera_input_EXTI_IRQn EXTI3_IRQn
#define P_limit_Pin GPIO_PIN_4
#define P_limit_GPIO_Port GPIOB
#define P_limit_EXTI_IRQn EXTI4_IRQn
#define Q_limit_Pin GPIO_PIN_5
#define Q_limit_GPIO_Port GPIOB
#define Q_limit_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
extern uint8_t Exec_command;
extern uint8_t RxBuffer[50];
extern char sending_data[50] ;
extern int32_t max_pos;
extern int32_t min_pos;
extern int32_t home_speed;

extern uint8_t Home_Pass; // to make two pass of z homing
extern uint8_t HOMED ;
//const float steps_per_millimeters
/* USER CODE END Private defines */

#define steps_per_millimeters 1 //50000
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
