/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
void Motor_Transmit(char[], uint8_t);
HAL_StatusTypeDef Motor_Receive(uint8_t[]);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Solenoid_Pin GPIO_PIN_0
#define Solenoid_GPIO_Port GPIOA
#define MOTOR_TX_Pin GPIO_PIN_2
#define MOTOR_TX_GPIO_Port GPIOA
#define MOTOR_RX_Pin GPIO_PIN_3
#define MOTOR_RX_GPIO_Port GPIOA
#define segmentG_Pin GPIO_PIN_4
#define segmentG_GPIO_Port GPIOA
#define digit1_Pin GPIO_PIN_5
#define digit1_GPIO_Port GPIOA
#define digit2_Pin GPIO_PIN_6
#define digit2_GPIO_Port GPIOA
#define digit3_Pin GPIO_PIN_7
#define digit3_GPIO_Port GPIOA
#define digit4_Pin GPIO_PIN_8
#define digit4_GPIO_Port GPIOA
#define HALL_SENSOR_Pin GPIO_PIN_9
#define HALL_SENSOR_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_0
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define segmentF_Pin GPIO_PIN_12
#define segmentF_GPIO_Port GPIOB
#define segmentE_Pin GPIO_PIN_13
#define segmentE_GPIO_Port GPIOB
#define segmentD_Pin GPIO_PIN_14
#define segmentD_GPIO_Port GPIOB
#define segmentC_Pin GPIO_PIN_15
#define segmentC_GPIO_Port GPIOB
#define segmentB_Pin GPIO_PIN_6
#define segmentB_GPIO_Port GPIOC
#define segmentA_Pin GPIO_PIN_10
#define segmentA_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B2_EXTI_IRQn EXTI0_IRQn
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define B3_EXTI_IRQn EXTI1_IRQn
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB
#define STLINERX_Pin GPIO_PIN_6
#define STLINERX_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
