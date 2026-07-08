/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOC
#define EIN1_Pin GPIO_PIN_2
#define EIN1_GPIO_Port GPIOF
#define EIN2_Pin GPIO_PIN_2
#define EIN2_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOA
#define KEY_USER_Pin GPIO_PIN_0
#define KEY_USER_GPIO_Port GPIOB
#define LIFT_TOP_SENSOR_Pin GPIO_PIN_13
#define LIFT_TOP_SENSOR_GPIO_Port GPIOB
#define LIFT_MIDDLE_SENSOR_Pin GPIO_PIN_8
#define LIFT_MIDDLE_SENSOR_GPIO_Port GPIOD
#define GAS_SENSOR_Pin GPIO_PIN_9
#define GAS_SENSOR_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_10
#define DIN2_GPIO_Port GPIOD
#define DIN1_Pin GPIO_PIN_11
#define DIN1_GPIO_Port GPIOD
#define LIFT_BOTTOM_SENSOR_Pin GPIO_PIN_14
#define LIFT_BOTTOM_SENSOR_GPIO_Port GPIOD
#define LED_USER_Pin GPIO_PIN_15
#define LED_USER_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
