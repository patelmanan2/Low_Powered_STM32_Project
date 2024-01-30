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
#include "stm32f7xx_hal.h"

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
#define SD_CardDetect_Input_Pin GPIO_PIN_0
#define SD_CardDetect_Input_GPIO_Port GPIOC
#define SD_CardDetect_Output_Pin GPIO_PIN_3
#define SD_CardDetect_Output_GPIO_Port GPIOC
#define Current_ADC_18650_Pin GPIO_PIN_0
#define Current_ADC_18650_GPIO_Port GPIOA
#define Voltage_ADC_18650_Pin GPIO_PIN_3
#define Voltage_ADC_18650_GPIO_Port GPIOA
#define Current_ADC_CMOS_Pin GPIO_PIN_0
#define Current_ADC_CMOS_GPIO_Port GPIOB
#define Voltage_ADC_CMOS_Pin GPIO_PIN_1
#define Voltage_ADC_CMOS_GPIO_Port GPIOB
#define Load_Switch_CMOS_Pin GPIO_PIN_10
#define Load_Switch_CMOS_GPIO_Port GPIOB
#define Load_Switch_18650_Pin GPIO_PIN_11
#define Load_Switch_18650_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
