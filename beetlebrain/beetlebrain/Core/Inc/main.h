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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_interrupt.h"
#include "fixed_point.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union {
	uint32_t dma_pointer;
	uint16_t adc_values[2];
} adc_values_type;

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
#define PHA_A_Pin GPIO_PIN_0
#define PHA_A_GPIO_Port GPIOA
#define PHA_B_Pin GPIO_PIN_1
#define PHA_B_GPIO_Port GPIOA
#define EN_TOF1_Pin GPIO_PIN_2
#define EN_TOF1_GPIO_Port GPIOA
#define EN_TOF2_Pin GPIO_PIN_3
#define EN_TOF2_GPIO_Port GPIOA
#define I_OTF_Pin GPIO_PIN_4
#define I_OTF_GPIO_Port GPIOA
#define RUN_Pin GPIO_PIN_7
#define RUN_GPIO_Port GPIOA
#define SWITCH_Pin GPIO_PIN_1
#define SWITCH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PWM_PERIOD 319 // 16Mhz/50kHz
#define SENSOR_TARGET
#define FORWARD_SPEED 300
#define ADC_TARGET	2000

#define KP 0.07
#define KI 0.007
#define DT 0.00002 // 1/50kHz

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
