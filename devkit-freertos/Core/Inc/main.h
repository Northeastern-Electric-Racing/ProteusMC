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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_ctrl.h"
#include "gatedriver.h"
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
#define PWM_PERIOD_CYCLES (ADV_TIM_CLK_MHz*1000000/PWM_FREQUENCY)&0xFFFE
#define PWM_FREQUENCY 30000
#define REGULATION_EXECUTION_RATE 1
#define TIM_CLOCK_DIVIDER 1
#define ADC_CLK_MHz 72
#define APB1TIM_FREQ 72000000
#define HALL_TIM_CLK 72000000
#define PWM_FREQ_SCALING 1
#define REP_COUNTER REGULATION_EXECUTION_RATE*2-1
#define HTMIN 1
#define ADV_TIM_CLK_MHz 144
#define SYSCLK_FREQ 72000000
#define ESTOP_Pin GPIO_PIN_13
#define ESTOP_GPIO_Port GPIOC
#define ESTOP_EXTI_IRQn EXTI15_10_IRQn
#define DRIVER_EN_Pin GPIO_PIN_6
#define DRIVER_EN_GPIO_Port GPIOA
#define ENC_Z_Pin GPIO_PIN_10
#define ENC_Z_GPIO_Port GPIOB
#define ENC_Z_EXTI_IRQn EXTI15_10_IRQn
#define EN1_Pin GPIO_PIN_10
#define EN1_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_11
#define EN2_GPIO_Port GPIOC
#define EN3_Pin GPIO_PIN_12
#define EN3_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
