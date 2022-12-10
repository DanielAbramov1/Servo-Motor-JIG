/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_0_Pin GPIO_PIN_3
#define IR_0_GPIO_Port GPIOA
#define IR_1_Pin GPIO_PIN_4
#define IR_1_GPIO_Port GPIOA
#define IR_2_Pin GPIO_PIN_5
#define IR_2_GPIO_Port GPIOA
#define tim16ch1_rgb_leds_Pin GPIO_PIN_6
#define tim16ch1_rgb_leds_GPIO_Port GPIOA
#define DC_I_PROP_Pin GPIO_PIN_7
#define DC_I_PROP_GPIO_Port GPIOA
#define tim3ch3_servo3_Pin GPIO_PIN_0
#define tim3ch3_servo3_GPIO_Port GPIOB
#define servo_en1_Pin GPIO_PIN_11
#define servo_en1_GPIO_Port GPIOB
#define servo_en2_Pin GPIO_PIN_12
#define servo_en2_GPIO_Port GPIOB
#define servo_en3_Pin GPIO_PIN_13
#define servo_en3_GPIO_Port GPIOB
#define dcmotor_mode_Pin GPIO_PIN_14
#define dcmotor_mode_GPIO_Port GPIOB
#define dcmotor_flt_n_Pin GPIO_PIN_15
#define dcmotor_flt_n_GPIO_Port GPIOB
#define microswitch0_Pin GPIO_PIN_8
#define microswitch0_GPIO_Port GPIOA
#define microswitch1_Pin GPIO_PIN_9
#define microswitch1_GPIO_Port GPIOA
#define user_led_Pin GPIO_PIN_3
#define user_led_GPIO_Port GPIOB
#define tim3ch1_servo1_Pin GPIO_PIN_4
#define tim3ch1_servo1_GPIO_Port GPIOB
#define tim3ch2_servo2_Pin GPIO_PIN_5
#define tim3ch2_servo2_GPIO_Port GPIOB
#define dcmotor_sleep_n_Pin GPIO_PIN_6
#define dcmotor_sleep_n_GPIO_Port GPIOB
#define dcmotor_in2_Pin GPIO_PIN_7
#define dcmotor_in2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SERVO_OUT_MAX 220.0
#define SERVO_OUT_MIN 75.0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
