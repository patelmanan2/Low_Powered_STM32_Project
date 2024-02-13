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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void Measurement_of_ADC_Voltage_18650();
void Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Current_CMOS();
void Measurement_of_ADC_Current_18650();
void Continuous_Same_State_Average();
void process_SD_card(void);
void ADC_Select_Voltage18650(void);
void ADC_Select_VoltageCMOS(void);
void ADC_Select_Current18650(void);
void ADC_Select_CurrentCMOS(void);
void setNumber();
void User_Input_Light_Cycel();
void Button_Debounce_Set();

float V_18650 = 0.0f;
float V_CMOS = 0.0f;
float V_50gain = 0.0f;
float V_25gain = 0.0f;
float C_CMOS = 0.0f;
float C_18650 = 0.0f;
int i, j;
unsigned int Switch_State = 0;
float seconds_since_start = 0.0f;
uint32_t start_time_ms = 0;
static uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 50;       // milliseconds
const uint32_t flashingDuration = 150;  // 5 seconds
uint32_t flashingStartTime = 0;
float valueToAdjust = 0;  // Integer value to be adjusted
uint8_t lastPlusState = GPIO_PIN_RESET;
uint8_t lastMinusState = GPIO_PIN_RESET;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
     /* USER CODE BEGIN 1 */

     /* USER CODE END 1 */

     /* MCU
      * Configuration--------------------------------------------------------*/

     /* Reset of all peripherals, Initializes the Flash interface and the
      * Systick. */
     HAL_Init();

     /* USER CODE BEGIN Init */

     /* USER CODE END Init */

     /* Configure the system clock */
     SystemClock_Config();

     /* USER CODE BEGIN SysInit */

     /* USER CODE END SysInit */

     /* Initialize all configured peripherals */
     MX_GPIO_Init();
     MX_ADC_Init();
     /* USER CODE BEGIN 2 */
     start_time_ms = HAL_GetTick();
     HAL_Delay(15);
     setNumber();
     /* USER CODE END 2 */

     /* Infinite loop */
     /* USER CODE BEGIN WHILE */
     while (1) {
    	  Button_Debounce_Set();
          setNumber();
          HAL_Delay(25);
          Measurement_of_ADC_Voltage_18650();
          Measurement_of_ADC_Voltage_CMOS();
          Measurement_of_ADC_Current_CMOS();
          Measurement_of_ADC_Current_18650();

          /* USER CODE END 3 */
     }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
     RCC_OscInitTypeDef RCC_OscInitStruct = {0};
     RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

     /** Configure the main internal regulator output voltage
      */
     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

     /** Initializes the RCC Oscillators according to the specified parameters
      * in the RCC_OscInitTypeDef structure.
      */
     RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
     RCC_OscInitStruct.MSIState = RCC_MSI_ON;
     RCC_OscInitStruct.MSICalibrationValue = 0;
     RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
     RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
     if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
          Error_Handler();
     }

     /** Initializes the CPU, AHB and APB buses clocks
      */
     RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
     RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
     RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
     RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
     RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

     if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
          Error_Handler();
     }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {
     /* USER CODE BEGIN ADC_Init 0 */

     /* USER CODE END ADC_Init 0 */

     //ADC_ChannelConfTypeDef sConfig = {0};

     /* USER CODE BEGIN ADC_Init 1 */

     /* USER CODE END ADC_Init 1 */

     /** Configure the global features of the ADC (Clock, Resolution, Data
      * Alignment and number of conversion)
      */
     hadc.Instance = ADC1;
     hadc.Init.OversamplingMode = DISABLE;
     hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
     hadc.Init.Resolution = ADC_RESOLUTION_12B;
     hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
     hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
     hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
     hadc.Init.ContinuousConvMode = DISABLE;
     hadc.Init.DiscontinuousConvMode = DISABLE;
     hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
     hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
     hadc.Init.DMAContinuousRequests = DISABLE;
     hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
     hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
     hadc.Init.LowPowerAutoWait = DISABLE;
     hadc.Init.LowPowerFrequencyMode = ENABLE;
     hadc.Init.LowPowerAutoPowerOff = DISABLE;
     if (HAL_ADC_Init(&hadc) != HAL_OK) {
          Error_Handler();
     }

     /** Configure for the selected ADC regular channel to be converted.

     sConfig.Channel = ADC_CHANNEL_12;
     sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }

     * Configure for the selected ADC regular channel to be converted.

     sConfig.Channel = ADC_CHANNEL_13;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }

     * Configure for the selected ADC regular channel to be converted.

     sConfig.Channel = ADC_CHANNEL_14;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }

     * Configure for the selected ADC regular channel to be converted.

     sConfig.Channel = ADC_CHANNEL_15;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }
      USER CODE BEGIN ADC_Init 2

      USER CODE END ADC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     /* USER CODE BEGIN MX_GPIO_Init_1 */
     /* USER CODE END MX_GPIO_Init_1 */

     /* GPIO Ports Clock Enable */
     __HAL_RCC_GPIOC_CLK_ENABLE();
     __HAL_RCC_GPIOA_CLK_ENABLE();
     __HAL_RCC_GPIOB_CLK_ENABLE();

     /*Configure GPIO pin Output Level */
     HAL_GPIO_WritePin(
         GPIOA, Discrete_Bit_0_Pin | Discrete_Bit_1_Pin | Discrete_Bit_2_Pin,
         GPIO_PIN_RESET);

     /*Configure GPIO pin Output Level */
     HAL_GPIO_WritePin(GPIOB,
                       User_Input_Status_Light_Pin |
                           User_Input_Status_Light_Green_Pin |
                           User_Input_Status_Light_Blue_Pin,
                       GPIO_PIN_RESET);

     /*Configure GPIO pins : Minus_Pin Plus_Pin */
     GPIO_InitStruct.Pin = Minus_Pin | Plus_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

     /*Configure GPIO pins : Discrete_Bit_0_Pin Discrete_Bit_1_Pin
      * Discrete_Bit_2_Pin */
     GPIO_InitStruct.Pin =
         Discrete_Bit_0_Pin | Discrete_Bit_1_Pin | Discrete_Bit_2_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     /*Configure GPIO pins : User_Input_Status_Light_Pin
      * User_Input_Status_Light_Green_Pin User_Input_Status_Light_Blue_Pin */
     GPIO_InitStruct.Pin = User_Input_Status_Light_Pin |
                           User_Input_Status_Light_Green_Pin |
                           User_Input_Status_Light_Blue_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

     /* USER CODE BEGIN MX_GPIO_Init_2 */
     /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
     /* Place your implementation of fputc here */
     /* e.g. write a character to the UART3 and Loop until the end of
      * transmission */
     // HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
     return ch;
}

void Measurement_of_ADC_Voltage_18650() {
     float V_ref = 3.3;  // This is known for each micro controller from data
     // sheet, V_ref = power supply in
     float ADC_resolution = (4096 - 1);  // 2^12 - 1
     float V_stepSize = V_ref / ADC_resolution;
     // ADC
     /* Start ADC Conversion for ADC1 */
     ADC_Select_Voltage18650();
     HAL_ADC_Start(&hadc);
     uint16_t rawValue1;
     if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
          /* Read the ADC1 value */
          rawValue1 = HAL_ADC_GetValue(&hadc);
          V_18650 = rawValue1 * V_stepSize;
     }
     HAL_ADC_Stop(&hadc);
}
void Measurement_of_ADC_Voltage_CMOS() {
     float V_ref = 3.3;  // This is known for each micro controller from data
     // sheet, V_ref = power supply in
     float ADC_resolution = (4096 - 1);  // 2^12 - 1
     float V_stepSize = V_ref / ADC_resolution;
     // ADC
     /* Start ADC Conversion for ADC1 */
     ADC_Select_VoltageCMOS();
     HAL_ADC_Start(&hadc);
     uint16_t rawValue1;
     if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
          /* Read the ADC1 value */
          rawValue1 = HAL_ADC_GetValue(&hadc);
          V_CMOS = rawValue1 * V_stepSize;
     }
     HAL_ADC_Stop(&hadc);
}

void Measurement_of_ADC_Current_18650() {
     float V_ref = 3.3;  // This is known for each micro controller from data
     // sheet, V_ref = power supply in
     float ADC_resolution = (4096 - 1);  // 2^12 - 1
     float V_stepSize = V_ref / ADC_resolution;
     // ADC
     /* Start ADC Conversion for ADC1 */
     ADC_Select_Current18650();
     HAL_ADC_Start(&hadc);
     uint16_t rawValue1;
     if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
          /* Read the ADC1 value */
          rawValue1 = HAL_ADC_GetValue(&hadc);
          C_18650 = ((rawValue1 * V_stepSize));

        /// 50) /
        //.0299562) // I_load = ((V_ADC / 50 gain) / .03 calibrated
                                 // shunt)
     }
     HAL_ADC_Stop(&hadc);
}

void Measurement_of_ADC_Current_CMOS() {
     float V_ref = 3.3;  // This is known for each micro controller from data
     // sheet, V_ref = power supply in
     float ADC_resolution = (4096 - 1);  // 2^12 - 1
     float V_stepSize = V_ref / ADC_resolution;
     // ADC
     /* Start ADC Conversion for ADC1 */
     ADC_Select_CurrentCMOS();
     HAL_ADC_Start(&hadc);
     uint16_t rawValue1;
     if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
          /* Read the ADC1 value */
          rawValue1 = HAL_ADC_GetValue(&hadc);
          C_CMOS = ((rawValue1 * V_stepSize));
        		  /// 20) /
                   // 4.713492);  // I_load = (( V_ADC / 20 Gain ) / 4.71
                                // calibrated shunt )
     }
     HAL_ADC_Stop(&hadc);
}

void ADC_Select_Voltage18650(void) {
     ADC_ChannelConfTypeDef sConfig = {0};
     sConfig.Channel = ADC_CHANNEL_15;
     sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }
}

void ADC_Select_VoltageCMOS(void) {
     ADC_ChannelConfTypeDef sConfig = {0};
     sConfig.Channel = ADC_CHANNEL_13;
     sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }
}

void ADC_Select_Current18650(void) {
     ADC_ChannelConfTypeDef sConfig = {0};
     sConfig.Channel = ADC_CHANNEL_14;
     sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }
}

void ADC_Select_CurrentCMOS(void) {
     ADC_ChannelConfTypeDef sConfig = {0};
     sConfig.Channel = ADC_CHANNEL_12;
     sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
     if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
          Error_Handler();
     }
}

void setNumber() {
     // Check each value and set the pins accordingly
     if (valueToAdjust == 1) {
          // value 1 = 001
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

          // Set Red
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

     } else if (valueToAdjust == 2) {
          // value 2 = 010
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

          // Set Yellow
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

     } else if (valueToAdjust == 3) {
          // value 3 = 011
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

          // Set Green
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

     } else if (valueToAdjust == 4) {
          // value 4 = 100
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

          // Set Cyan
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

     } else if (valueToAdjust == 5) {
          // value 5 = 101
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

          // Set Blue
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

     } else if (valueToAdjust == 6) {
          // value 6 = 110
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

          // Set Magenta
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

     } else if (valueToAdjust == 7) {
          // value 7 = 111
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

          // Set White
          HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                            User_Input_Status_Light_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                            User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                            User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);
     }

     else if (valueToAdjust == 0) {
               // value 7 = 111
               HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
               HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
               HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

               // Set White
               HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
                                 User_Input_Status_Light_Pin, GPIO_PIN_RESET);
               HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port,
                                 User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
               HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port,
                                 User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);
          }
     /*may need to implement state for numbers entered over 7 and numbers
     under zero */
}
     // testing

void User_Input_Light_Cycel(){
	// 1. Set Red
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);
	                         HAL_Delay(14);

	                         // 2. Set Yellow (Red + Green)
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);
	                         HAL_Delay(14);

	                         // 3. Set Green
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);
	                         HAL_Delay(14);

	                         // 4. Set Cyan (Green + Blue)
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);
	                         HAL_Delay(14);

	                         // 5. Set Blue
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);
	                         HAL_Delay(14);

	                         // 6. Set Magenta (Red + Blue)
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);
	                         HAL_Delay(14);

	                         // 7. Set White (Red + Green + Blue)
	                         HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port,
	                                           User_Input_Status_Light_Pin,
	                                           GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Green_GPIO_Port,
	                             User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
	                         HAL_GPIO_WritePin(
	                             User_Input_Status_Light_Blue_GPIO_Port,
	                             User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);
	                         HAL_Delay(14);
}

void Button_Debounce_Set(){
	uint8_t currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
	          uint8_t currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);

	          if (currentPlusState == GPIO_PIN_SET ||
	              currentMinusState == GPIO_PIN_SET) {
	               // set to high state
	               if ((HAL_GetTick() - lastDebounceTime) > debounceDelay) {
	                    // Only update the value if the state has changed
	                    if ((currentPlusState == GPIO_PIN_SET &&
	                         lastPlusState != GPIO_PIN_SET) ||
	                        (currentMinusState == GPIO_PIN_SET &&
	                         lastMinusState != GPIO_PIN_SET)) {
	                         if (currentPlusState == GPIO_PIN_SET) {
	                              valueToAdjust++;
	                              if (valueToAdjust >= 7) {
	                                   valueToAdjust = 7;
	                              }
	                         }

	                         else if (currentMinusState == GPIO_PIN_SET) {
	                              valueToAdjust--;
	                              if (valueToAdjust < 0) {
	                                   valueToAdjust = 0;
	                              }
	                         }
	                    }

	                    flashingStartTime = HAL_GetTick();
	                    while ((HAL_GetTick() - flashingStartTime) <
	                           flashingDuration) {
	                         // Save the last state before reading the current state
	                         lastPlusState = currentPlusState;
	                         lastMinusState = currentMinusState;

	                         currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
	                         currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);

	                         // Check for subsequent button presses to restart the
	                         // timer
	                         if (currentPlusState == GPIO_PIN_SET ||
	                             currentMinusState == GPIO_PIN_SET) {
	                              flashingStartTime =
	                                  HAL_GetTick();  // Restart the 5-second
	                                                  // interval

	                              // Only update the value if the state has changed
	                              if ((currentPlusState == GPIO_PIN_SET &&
	                                   lastPlusState != GPIO_PIN_SET) ||
	                                  (currentMinusState == GPIO_PIN_SET &&
	                                   lastMinusState != GPIO_PIN_SET)) {
	                                   if (currentPlusState == GPIO_PIN_SET) {
	                                        valueToAdjust++;
	                                        if (valueToAdjust >= 7) {
	                                             valueToAdjust = 7;
	                                        }
	                                   }

	                                   else if (currentMinusState == GPIO_PIN_SET) {
	                                        valueToAdjust--;
	                                        if (valueToAdjust < 0) {
	                                             valueToAdjust = 0;
	                                        }
	                                   }
	                              }

	                              // HAL_GPIO_TogglePin(User_Input_Status_Light_GPIO_Port,
	                              // User_Input_Status_Light_Pin); HAL_Delay(100);
	                         }
	                         // 1. Set Red
	                         User_Input_Light_Cycel();
	                    }
	                    lastDebounceTime = HAL_GetTick();
	               }

	               // Save the last state at the end of the loop
	               lastPlusState = currentPlusState;
	               lastMinusState = currentMinusState;

	               /* USER CODE END WHILE */

	               /* USER CODE BEGIN 3 */
	          }
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
     /* USER CODE BEGIN Error_Handler_Debug */
     /* User can add his own implementation to report the HAL error return state
      */
     __disable_irq();
     while (1) {
     }
     /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
     /* USER CODE BEGIN 6 */
     /* User can add his own implementation to report the file name and line
        number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
        file, line) */
     /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
