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
#include "stm32l0xx_hal.h"

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
#define Minus_Pin GPIO_PIN_0
#define Minus_GPIO_Port GPIOC
#define Plus_Pin GPIO_PIN_1
#define Plus_GPIO_Port GPIOC
#define Current_ADC_CMOS_Pin GPIO_PIN_2
#define Current_ADC_CMOS_GPIO_Port GPIOC
#define Voltage_ADC_CMOS_Pin GPIO_PIN_3
#define Voltage_ADC_CMOS_GPIO_Port GPIOC
#define Discrete_Bit_0_Pin GPIO_PIN_0
#define Discrete_Bit_0_GPIO_Port GPIOA
#define Discrete_Bit_1_Pin GPIO_PIN_1
#define Discrete_Bit_1_GPIO_Port GPIOA
#define Discrete_Bit_2_Pin GPIO_PIN_4
#define Discrete_Bit_2_GPIO_Port GPIOA
#define Current_ADC_18650_Pin GPIO_PIN_4
#define Current_ADC_18650_GPIO_Port GPIOC
#define Voltage_ADC_18650_Pin GPIO_PIN_5
#define Voltage_ADC_18650_GPIO_Port GPIOC
#define User_Input_Status_Light_Pin GPIO_PIN_0
#define User_Input_Status_Light_GPIO_Port GPIOB
#define User_Input_Status_Light_Green_Pin GPIO_PIN_1
#define User_Input_Status_Light_Green_GPIO_Port GPIOB
#define User_Input_Status_Light_Blue_Pin GPIO_PIN_2
#define User_Input_Status_Light_Blue_GPIO_Port GPIOB
#define LS_1_Pin GPIO_PIN_12
#define LS_1_GPIO_Port GPIOB
#define LS_2_Pin GPIO_PIN_13
#define LS_2_GPIO_Port GPIOB
#define LS_3_Pin GPIO_PIN_14
#define LS_3_GPIO_Port GPIOB
#define LS_4_Pin GPIO_PIN_15
#define LS_4_GPIO_Port GPIOB
#define LS_LOW_Pin GPIO_PIN_6
#define LS_LOW_GPIO_Port GPIOC
#define LS_5_Pin GPIO_PIN_7
#define LS_5_GPIO_Port GPIOC
#define LS_6_Pin GPIO_PIN_8
#define LS_6_GPIO_Port GPIOC
#define LS_7_Pin GPIO_PIN_9
#define LS_7_GPIO_Port GPIOC
#define LS_8_Pin GPIO_PIN_8
#define LS_8_GPIO_Port GPIOA
#define LS_HIGH_Pin GPIO_PIN_9
#define LS_HIGH_GPIO_Port GPIOA
#define Threshold_Red_Pin GPIO_PIN_3
#define Threshold_Red_GPIO_Port GPIOB
#define Threshold_Green_Pin GPIO_PIN_4
#define Threshold_Green_GPIO_Port GPIOB
#define Threshold_Blue_Pin GPIO_PIN_5
#define Threshold_Blue_GPIO_Port GPIOB
#define SD_INPUT_DATA_Pin GPIO_PIN_8
#define SD_INPUT_DATA_GPIO_Port GPIOB
#define SD_INPUT_DIGIT_Pin GPIO_PIN_9
#define SD_INPUT_DIGIT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
