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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Measurement_of_ADC_Voltage_18650();
void Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Voltage_DiffAmp_CMOS();
void Measurement_of_ADC_Voltage_DiffAmp_18650();
float Convert_Measurement_of_ADC_Voltage_DiffAmp_18650_to_Current_of_18650(float Voltage_DiffAmp_18650);
float Convert_Measurement_of_ADC_Voltage_DiffAmp_CMOS_to_Current_of_CMOS(float Voltage_DiffAmp_CMOS);
void Continuous_Same_State_Average();
void process_SD_card(void);
void ADC_Select_Voltage18650(void);
void ADC_Select_VoltageCMOS(void);
void ADC_Select_Current18650(void);
void ADC_Select_CurrentCMOS(void);
void setNumber();
void User_Input_Light_Cycle();
void Button_Debounce_Set();
void Reset_The_Whole_B();
void Set_LS_1();
void Set_LS_2();
void Set_LS_3();
void Set_LS_4();
void Set_LS_5();
void Set_LS_6();
void Set_LS_7();
void Set_LS_8();
void Set_High();
void Set_Low();
void AdjustStateTo0();
void AdjustStateTo1();
void AdjustStateTo2();
void AdjustStateTo3();
void AdjustStateTo4();
void AdjustStateTo5();
void AdjustStateTo6();
void AdjustStateTo7();
void AdjustStateTo8();
void FlickersetNumber();
void AdjustValueInTo0();
void AdjustValueInTo1();
void AdjustValueInTo2();
void AdjustValueInTo3();
void AdjustValueInTo4();
void AdjustValueInTo5();
void AdjustValueInTo6();
void AdjustValueInTo7();
float Threshold(int value_from_button);

float V_18650 = 0.0f;
float V_CMOS = 0.0f;
float V_50gain = 0.0f;
float V_25gain = 0.0f;
float Voltage_DiffAmp_CMOS = 0.0f;
float C_18650 = 0.0f;
float Voltage_DiffAmp_18650 = 0.0f;
float C_CMOS = 0.0f;
int i, j;
unsigned int Switch_State = 0;
float seconds_since_start = 0.0f;
uint32_t start_time_ms = 0;
static uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 50;      // milliseconds
const uint32_t flashingDuration = 150;  // 5 seconds
uint32_t flashingStartTime = 0;
int valueToAdjust = 0;  // Integer value to be adjusted
uint8_t lastPlusState = GPIO_PIN_RESET;
uint8_t lastMinusState = GPIO_PIN_RESET;
int measurement_num = 0;
float conversionFactor = 0;
// State transition flags
int entered_LS1_from_LS2 = 0; // Indicates if LS1 was entered from LS2
int entered_LS2_from_LS1 = 0; // Indicates if LS2 was entered from LS1
int entered_LS2_from_LS3 = 0; // Indicates if LS2 was entered from LS3
int entered_LS3_from_LS2 = 0; // Indicates if LS3 was entered from LS2
int entered_LS3_from_LS4 = 0; // Indicates if LS3 was entered from LS4
int entered_LS4_from_LS3 = 0; // Indicates if LS4 was entered from LS3
// State transition flags
int entered_LS5_from_LS6 = 0; // Indicates if LS5 was entered from LS6
int entered_LS6_from_LS5 = 0; // Indicates if LS6 was entered from LS5
int entered_LS6_from_LS7 = 0; // Indicates if LS6 was entered from LS7
int entered_LS7_from_LS6 = 0; // Indicates if LS7 was entered from LS6
int entered_LS7_from_LS8 = 0; // Indicates if LS7 was entered from LS8
int entered_LS8_from_LS7 = 0; // Indicates if LS8 was entered from LS7
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {CASE_INIT, LS_1, LS_2, LS_3, LS_4, LS_5, LS_6, LS_7, LS_8} CASE;
CASE state = CASE_INIT;
//typedef enum { LS_1, LS_2, LS_3, LS_4 } LOW;
//typedef enum { LS_5, LS_6, LS_7, LS_8 } HIGH;

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
   MX_USART2_UART_Init();

   /* USER CODE BEGIN 2 */
   char msg[128];
   start_time_ms = HAL_GetTick();
   HAL_Delay(15);
   setNumber();
   HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

   Reset_The_Whole_B();



   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   while (1) {
	   /* USER CODE BEGIN 3 */
	   Measurement_of_ADC_Voltage_18650();
	         Measurement_of_ADC_Voltage_CMOS();
	         Measurement_of_ADC_Voltage_DiffAmp_CMOS();
	         Measurement_of_ADC_Voltage_DiffAmp_18650();
	         C_18650 = Convert_Measurement_of_ADC_Voltage_DiffAmp_18650_to_Current_of_18650(Voltage_DiffAmp_18650);
	         C_CMOS = Convert_Measurement_of_ADC_Voltage_DiffAmp_CMOS_to_Current_of_CMOS(Voltage_DiffAmp_CMOS);
      Button_Debounce_Set();

      uint32_t current_time_ms = HAL_GetTick();
      seconds_since_start = (current_time_ms - start_time_ms) / 1000.0f;
      if(state == LS_8 || state == LS_7 || state == LS_6 || state == LS_5){
            		  if(C_18650 <= (Threshold(valueToAdjust))*.9){
            			 Set_Low();
            			 state = LS_4;
            			 Set_LS_4();
            		  }
            		  else if(C_18650 >= 2.05){
            			  MX_GPIO_Init();
            			  Error_Handler();

            		    }
                  	  }

            	  else if(state == LS_4 || state == LS_3 || state == LS_2 || state == LS_1){
            		  if(C_CMOS >= (Threshold(valueToAdjust))*1.1){
            			 Set_High();
            			 state = LS_8;
            			 Set_LS_8();
            	  }
            		  else if(C_CMOS >= 2.05){
            			  MX_GPIO_Init();
            			  Error_Handler();
            	  }
            	  }
      Measurement_of_ADC_Voltage_18650();
      Measurement_of_ADC_Voltage_CMOS();
      Measurement_of_ADC_Voltage_DiffAmp_CMOS();
      Measurement_of_ADC_Voltage_DiffAmp_18650();
      C_18650 = Convert_Measurement_of_ADC_Voltage_DiffAmp_18650_to_Current_of_18650(Voltage_DiffAmp_18650);
      C_CMOS = Convert_Measurement_of_ADC_Voltage_DiffAmp_CMOS_to_Current_of_CMOS(Voltage_DiffAmp_CMOS);
      // UART Debuggin
      sprintf(msg, "%.3f,%.3f,%.5f,%.3f,%.5f,%d,%d,%d\r\n", seconds_since_start, V_18650,
              C_18650,        // 18650 Current
              V_CMOS,         // CMOS Voltage
			  C_CMOS,         // CMOS Current
              valueToAdjust,  // Threshold
              Switch_State, measurement_num);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);






//               Handle behavior based on state
          switch (state) {
          case CASE_INIT: {
                  state = LS_8;
                  break;
          }

          case LS_8:
        	  Switch_State = 8;
              static int Hys_ls8 = 1;
              Set_LS_8();
              AdjustStateTo7();

              // Transitioning back to LS_7
              if ((Hys_ls8 == 0 && Voltage_DiffAmp_18650 < .33) || (Hys_ls8 == 1 && Voltage_DiffAmp_18650 < .231)) {
                  Set_LS_7();
                  state = LS_7;
                  Hys_ls8 = 1; // Reset Hys_ls8 when leaving LS_8
              }

              // Update hysteresis flag based on remaining in LS_8
              if (Voltage_DiffAmp_18650 > .429) {
                  Hys_ls8 = 0;
              }

              break;

          case LS_7:
        	  Switch_State = 7;
              static int Hys_ls7 = 1;
              Set_LS_7();
              AdjustStateTo6();

              // Transitioning to LS_6 or LS_8 based on conditions
              if ((Hys_ls7 == 0 && Voltage_DiffAmp_18650 < .33) || (Hys_ls7 == 1 && Voltage_DiffAmp_18650 < .231)) {
                  Set_LS_6();
                  state = LS_6;
                  // Indicate entering LS_6 from LS_7
                  entered_LS6_from_LS7 = 1;
              } else if ((Hys_ls7 == 0 && Voltage_DiffAmp_18650 > 2.97) || (Hys_ls7 == 1 && Voltage_DiffAmp_18650 > 3.069)) {
                  Set_LS_8();
                  state = LS_8;
                  // Reset flags related to LS_7 transitions
                  entered_LS7_from_LS6 = 0;
              }

              // Update hysteresis flag based on voltage thresholds
              if (Voltage_DiffAmp_18650 > .429 || Voltage_DiffAmp_18650 > 3.069) {
                  Hys_ls7 = 0;
              } else {
                  Hys_ls7 = 1;
              }

              break;

          case LS_6:
        	  Switch_State = 6;
              static int Hys_ls6 = 1; // For persistence across state transitions
              Set_LS_6();
              AdjustStateTo5();

              // Transitioning from LS_6 to LS_5
              if ((Hys_ls6 == 0 && Voltage_DiffAmp_18650 < .33) || (Hys_ls6 == 1 && Voltage_DiffAmp_18650 < .231)) {
                  Set_LS_5();
                  state = LS_5;
                  // Reset flags related to LS_6 transitions
                  entered_LS6_from_LS7 = 0;
              } else if ((Hys_ls6 == 0 && Voltage_DiffAmp_18650 > 2.97) || (Hys_ls6 == 1 && Voltage_DiffAmp_18650 > 3.069)) {
                  Set_LS_7();
                  state = LS_7;
                  // Indicate entering LS_7 from LS_6
                  entered_LS7_from_LS6 = 1;
              }

              // Update hysteresis flag based on voltage thresholds
              if (Voltage_DiffAmp_18650 > .429 || Voltage_DiffAmp_18650 > 3.069) {
                  Hys_ls6 = 0;
              } else {
                  Hys_ls6 = 1;
              }

              break;

          case LS_5:
        	  Switch_State = 5;
              int Hys_ls5 = 1;
              Set_LS_5();
              AdjustStateTo4();
              if (Voltage_DiffAmp_18650 >= 2.97) { // Using the special threshold for moving up from a very low state
                  Set_LS_6();
                  state = LS_6;
              }
              break;
               //end of LS_5

          case LS_4:
        	  Switch_State = 4;
              static int Hys_ls4 = 1;
              Set_LS_4();
              AdjustStateTo3();

              // Transitioning back to LS_3
              if ((Hys_ls4 == 0 && Voltage_DiffAmp_CMOS < .33) || (Hys_ls4 == 1 && Voltage_DiffAmp_CMOS < .231)) {
                  Set_LS_3();
                  state = LS_3;
                  Hys_ls4 = 1; // Reset Hys_ls4 when leaviQ21aZaqw21	AQ2Ang LS_4
              }

              // Update hysteresis flag based on remaining in LS_4
              if (Voltage_DiffAmp_CMOS > .429) {
                  Hys_ls4 = 0;
              }

              break;

          case LS_3:
        	  Switch_State = 3;
              static int Hys_ls3 = 1;
              Set_LS_3();
              AdjustStateTo2();

              // Transitioning to LS_2 or LS_4 based on conditions
              if ((Hys_ls3 == 0 && Voltage_DiffAmp_CMOS < .33) || (Hys_ls3 == 1 && Voltage_DiffAmp_CMOS < .231)) {
                  Set_LS_2();
                  state = LS_2;
                  // Indicate entering LS_2 from LS_3
                  entered_LS2_from_LS3 = 1;
              } else if ((Hys_ls3 == 0 && Voltage_DiffAmp_CMOS > 2.97) || (Hys_ls3 == 1 && Voltage_DiffAmp_CMOS > 3.069)) {
                  Set_LS_4();
                  state = LS_4;
                  // Reset flags related to LS_3 transitions
                  entered_LS3_from_LS2 = 0;
              }

              // Update hysteresis flag based on voltage thresholds
              if (Voltage_DiffAmp_CMOS > .429 || Voltage_DiffAmp_CMOS > 3.069) {
                  Hys_ls3 = 0;
              } else {
                  Hys_ls3 = 1;
              }

              break;

          case LS_2:
        	  Switch_State = 2;
              static int Hys_ls2 = 1; // For persistence across state transitions
              Set_LS_2();
              AdjustStateTo1();

              // Transitioning from LS_2 to LS_1
              if ((Hys_ls2 == 0 && Voltage_DiffAmp_CMOS < .33) || (Hys_ls2 == 1 && Voltage_DiffAmp_CMOS < .231)) {
                  Set_LS_1();
                  state = LS_1;
                  // Reset flags related to LS_2 transitions
                  entered_LS2_from_LS3 = 0;
              } else if ((Hys_ls2 == 0 && Voltage_DiffAmp_CMOS > 2.97) || (Hys_ls2 == 1 && Voltage_DiffAmp_CMOS > 3.069)) {
                  Set_LS_3();
                  state = LS_3;
                  // Indicate entering LS_3 from LS_2
                  entered_LS3_from_LS2 = 1;
              }

              // Update hysteresis flag based on voltage thresholds
              if (Voltage_DiffAmp_CMOS > .429 || Voltage_DiffAmp_CMOS > 3.069) {
                  Hys_ls2 = 0;
              } else {
                  Hys_ls2 = 1;
              }

              break;

          case LS_1:
        	  Switch_State = 1;
              int Hys_ls1 = 1;
              Set_LS_1();
              AdjustStateTo0();
              if (Voltage_DiffAmp_CMOS >= 2.97) { // Using the special threshold for moving up from a very low state
                  Set_LS_2();
                  state = LS_2;
              }
              break;

          default: {
                  Reset_The_Whole_B();
                  state = CASE_INIT;
          }
          }

      measurement_num++;


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
   RCC_ClkInitStruct.ClockType =
       RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
   // ADC_Select_Voltage18650();
   // ADC_Select_VoltageCMOS();
   // ADC_Select_Current18650();
   // ADC_Select_CurrentCMOS();
   /* USER CODE END ADC_Init 0 */

   /* USER CODE BEGIN ADC_Init 1 */

   /* USER CODE END ADC_Init 1 */

   /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of
    * conversion)
    */
   hadc.Instance = ADC1;
   hadc.Init.OversamplingMode = DISABLE;
   hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
   hadc.Init.Resolution = ADC_RESOLUTION_12B;
   hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
   hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
   hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
   hadc.Init.ContinuousConvMode = ENABLE;
   hadc.Init.DiscontinuousConvMode = DISABLE;
   hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
   hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
   hadc.Init.DMAContinuousRequests = DISABLE;
   hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
   hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
   hadc.Init.LowPowerAutoWait = DISABLE;
   hadc.Init.LowPowerFrequencyMode = DISABLE;
   hadc.Init.LowPowerAutoPowerOff = DISABLE;
   if (HAL_ADC_Init(&hadc) != HAL_OK) {
      Error_Handler();
   }

   /** Configure for the selected ADC regular channel to be converted.
    */

   /* USER CODE BEGIN ADC_Init 2 */

   /* USER CODE END ADC_Init 2 */
}
static void MX_USART2_UART_Init(void) {
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
   huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin|Discrete_Bit_1_Pin|Discrete_Bit_2_Pin|LS_8_Pin
                          |LS_HIGH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, User_Input_Status_Light_Pin|User_Input_Status_Light_Green_Pin|User_Input_Status_Light_Blue_Pin|LS_1_Pin
                          |LS_2_Pin|LS_3_Pin|LS_4_Pin|Threshold_Red_Pin
                          |Threshold_Green_Pin|Threshold_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LS_LOW_Pin|LS_5_Pin|LS_6_Pin|LS_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Minus_Pin Plus_Pin */
  GPIO_InitStruct.Pin = Minus_Pin|Plus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Discrete_Bit_0_Pin Discrete_Bit_1_Pin Discrete_Bit_2_Pin LS_8_Pin
                           LS_HIGH_Pin */
  GPIO_InitStruct.Pin = Discrete_Bit_0_Pin|Discrete_Bit_1_Pin|Discrete_Bit_2_Pin|LS_8_Pin
                          |LS_HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : User_Input_Status_Light_Pin User_Input_Status_Light_Green_Pin User_Input_Status_Light_Blue_Pin LS_1_Pin
                           LS_2_Pin LS_3_Pin LS_4_Pin Threshold_Red_Pin
                           Threshold_Green_Pin Threshold_Blue_Pin */
  GPIO_InitStruct.Pin = User_Input_Status_Light_Pin|User_Input_Status_Light_Green_Pin|User_Input_Status_Light_Blue_Pin|LS_1_Pin
                          |LS_2_Pin|LS_3_Pin|LS_4_Pin|Threshold_Red_Pin
                          |Threshold_Green_Pin|Threshold_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LS_LOW_Pin LS_5_Pin LS_6_Pin LS_7_Pin */
  GPIO_InitStruct.Pin = LS_LOW_Pin|LS_5_Pin|LS_6_Pin|LS_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

//******************************** START OF ADC-RELATED FUNCTIONS **************************************//

/*
 * Measurement_of_ADC functions perform ADC measurements and conversions for each respective channel. This
 * is done with the following steps:
 * 1) Select the respective ADC channel register using CHSELR (uses standard STM bit-masking)
 * 2) Select the respective ADC channel (using the respective ADC_Select function)
 * 3) Start the ADC
 * 4) Read the raw value
 * 5) Convert to respective measurement units and store to respective global variable
 * 6) Stop the ADC
 */

/*
 * Measurement_of_ADC_Voltage_18650() performs an ADC measurement and conversion for the VOLTAGE of the
 * 18650 battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x8000h (channel 15)
 * Calls ADC_Select_Voltage18650() to set channel
 * Converted values store to V_18650
 */
void Measurement_of_ADC_Voltage_18650() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x8000;
   ADC_Select_Voltage18650();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue1;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue1 = HAL_ADC_GetValue(&hadc);
      V_18650 = ((rawValue1 * V_stepSize) * (1/.65)); //change this math here
   }
   HAL_ADC_Stop(&hadc);
}

/*
 * Measurement_of_ADC_Voltage_CMOS() performs an ADC measurement and conversion for the VOLTAGE of the
 * CMOS battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x2000h (channel 13)
 * Calls ADC_Select_VoltageCMOS() to set channel
 * Converted values store to V_CMOS
 */
void Measurement_of_ADC_Voltage_CMOS() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x2000;
   ADC_Select_VoltageCMOS();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue1;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue1 = HAL_ADC_GetValue(&hadc);
      V_CMOS = ((rawValue1 * V_stepSize) * (1/.65));



   }
   HAL_ADC_Stop(&hadc);
}

/*
 * Measurement_of_ADC_Voltage_DiffAmp_18650() performs an ADC measurement and conversion for the CURRENT of the
 * 18650 battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x4000h (channel 14)
 * Calls ADC_Select_Current18650() to set channel
 * Converted values store to Voltage_DiffAmp_18650
 */

float Convert_Measurement_of_ADC_Voltage_DiffAmp_18650_to_Current_of_18650(float Voltage_DiffAmp_18650){
	float conversion_18650 = 0;
	if(state == LS_5)
	{
		conversion_18650 = (Voltage_DiffAmp_18650/888); // do math
	}
	if(state == LS_6)
	{
		conversion_18650 = (Voltage_DiffAmp_18650/101); // do math
	}
	if(state == LS_7)
	{
		conversion_18650 = (Voltage_DiffAmp_18650/11.5); // do math
	}
	if(state == LS_8)
	{
		conversion_18650 = (Voltage_DiffAmp_18650/1.18); // do math
	}
	if(state == CASE_INIT)
	{
		conversion_18650 = (Voltage_DiffAmp_18650/1.18); // do math
	}

	return conversion_18650;
}



void Measurement_of_ADC_Voltage_DiffAmp_18650() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 bits - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x4000;
   ADC_Select_Current18650();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue1;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue1 = HAL_ADC_GetValue(&hadc);
      Voltage_DiffAmp_18650 = ((rawValue1 * V_stepSize));
   }
   HAL_ADC_Stop(&hadc);
}

float Convert_Measurement_of_ADC_Voltage_DiffAmp_CMOS_to_Current_of_CMOS(float Voltage_DiffAmp_CMOS){
	float conversion_CMOS = 0;
	if(state == LS_1)
	{
		conversion_CMOS = (Voltage_DiffAmp_CMOS/889); // do math
	}
	if(state == LS_2)
	{
		conversion_CMOS = (Voltage_DiffAmp_CMOS/99.7); // do math
	}
	if(state == LS_3)
	{
		conversion_CMOS = (Voltage_DiffAmp_CMOS/11.1);
	}
	if(state == LS_4)
	{
		conversion_CMOS = (Voltage_DiffAmp_CMOS/1.149); // do math
	}
	if(state == CASE_INIT)
		{
			conversion_CMOS = 0;
		}

	return conversion_CMOS;
}

float Threshold(int value_from_button){
	if (value_from_button == 0){
		return .001;
	}
	if (value_from_button == 1){
			return .005;
	}
	if (value_from_button == 2){
			return .010;
	}
	if (value_from_button == 3){
			return .050;
	}
	if (value_from_button == 4){
			return .100;
	}
	if (value_from_button == 5){
			return .500;
	}
	if (value_from_button == 6){
			return 1.00;
	}
	if (value_from_button == 7){
			return 1.5;
	}
}

/*
 * Measurement_of_ADC_Voltage_DiffAmp_CMOS() performs an ADC measurement and conversion for the CURRENT of the
 * CMOS battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x1000h (channel 12)
 * Calls ADC_Select_CurrentCMOS() to set channel
 * Converted values store to Voltage_DiffAmp_CMOS
 */
void Measurement_of_ADC_Voltage_DiffAmp_CMOS() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x1000;
   ADC_Select_CurrentCMOS();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue1;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue1 = HAL_ADC_GetValue(&hadc);
      Voltage_DiffAmp_CMOS = ((rawValue1 * V_stepSize));
   }
   HAL_ADC_Stop(&hadc);
}

/*
 * ADC_Select_Voltage18650() selects the channel that relates to the VOLTAGE of the 18650 battery.
 * It sets sConfig to its respective channel (15) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_Voltage18650(void) {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_15;
   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_VoltageCMOS() selects the channel that relates to the VOLTAGE of the 18650 battery.
 * It sets sConfig to its respective channel (13) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_VoltageCMOS(void) {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_13;
   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_Current18650() selects the channel that relates to the VOLTAGE of the 18650 battery.
 * It sets sConfig to its respective channel (14) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_Current18650(void) {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_14;
   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_CurrentCMOS() selects the channel that relates to the VOLTAGE of the 18650 battery.
 * It sets sConfig to its respective channel (12) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_CurrentCMOS(void) {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_12;
   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

//************************ START OF THRESHOLD INPUT AND BOARD COMMUNICATION FUNCTIONS **********************//

/*
 * setNumber() checks the value of valueToAdjust variable. If the value is set as a number from 0 to 7,
 * it calls the respective AdjustValueInTo function and sets the threshold LED to its respective color.
 */
void setNumber() {
   // Check each value and set the pins accordingly
   if (valueToAdjust == 1) {
      // value 1 = 001
	   AdjustValueInTo1();

	  HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	  	                        GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	  	                        GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	  	                        GPIO_PIN_RESET);
   } else if (valueToAdjust == 2) {
      // value 2 = 010
	   AdjustValueInTo2();

	   HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_RESET);


   } else if (valueToAdjust == 3) {
	   AdjustValueInTo3();

      HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
      	  	                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
      	  	                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
      	  	                        GPIO_PIN_RESET);

   } else if (valueToAdjust == 4) {
	   AdjustValueInTo4();

	   HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_SET);

   } else if (valueToAdjust == 5) {
	   AdjustValueInTo5();

	   HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_SET);


   } else if (valueToAdjust == 6) {
	   AdjustValueInTo6();

	   HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_SET);


   } else if (valueToAdjust == 7) {
	   AdjustValueInTo7();

	   HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_SET);

   }

   else if (valueToAdjust == 0) {
      // value 7 = 111
	   AdjustValueInTo0();

	  HAL_GPIO_WritePin(GPIOB, Threshold_Red_Pin,
	   	  	                        GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, Threshold_Green_Pin,
	   	  	                        GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, Threshold_Blue_Pin,
	   	  	                        GPIO_PIN_RESET);
   }
   /*may need to implement state for numbers entered over 7 and numbers
   under zero */
}

/*
 * AdjustStateTo functions are related to board communication features. Each of the functions will set
 * the OUTPUT bits to their respective State values to be received by the writing board. Then, it sets
 * the state LED to it's corresponding state color by setting each of the RGB pins ON or OFF.
 *
 * NOTE: Does NOT change the actual state the board is in, ONLY changes outputs between LEDs and Writing board
 *
 * Used in the State Machine, Button_Debounce_Set(), and setNumber() functions.
 */

/*
 * AdjustStateTo0() Sets output communication bits to 0 (000) and the state LED to OFF
 */
void AdjustStateTo0(){
	// value 0 = 000
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set White
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_RESET);
}

/*
 * AdjustStateTo1() Sets output communication bits to 1 (001) and the state LED to RED
 */
void AdjustStateTo1(){
	HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set Red
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_RESET);
}

/*
 * AdjustStateTo2() Sets output communication bits to 2 (010) and the state LED to YELLOW
 */
void AdjustStateTo2(){
	// value 2 = 010
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set Yellow
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_RESET);
}

/*
 * AdjustStateTo3() Sets output communication bits to 3 (011) and the state LED to GREEN
 */
void AdjustStateTo3(){
	// value 3 = 011
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set Green
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_RESET);
}

/*
 * AdjustStateTo4() Sets output communication bits to 4 (100) and the state LED to CYAN
 */
void AdjustStateTo4(){
	// value 4 = 100
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Cyan
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_SET);
}

/*
 * AdjustStateTo5() Sets output communication bits to 5 (101) and the state LED to BLUE
 */
void AdjustStateTo5(){
	// value 5 = 101
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Blue
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_SET);
}

/*
 * AdjustStateTo6() Sets output communication bits to 6 (110) and the state LED to MAGENTA
 */
void AdjustStateTo6(){
	// value 6 = 110
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Magenta
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_SET);
}

/*
 * AdjustStateTo7() Sets output communication bits to 7 (111) and the state LED to WHITE
 */
void AdjustStateTo7(){
	// value 7 = 111
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set White
	      HAL_GPIO_WritePin(User_Input_Status_Light_GPIO_Port, User_Input_Status_Light_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
	                        GPIO_PIN_SET);
	      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
	                        GPIO_PIN_SET);
}

/*
 * AdjustValueInTo functions are related to board communication and threshold input features. Each of
 * the functions will set the OUTPUT bits to their respective threshold input values to be received by the
 * writing board. Then, it sets the threshold LED to it's corresponding state color by setting each of the
 * RGB pins ON or OFF.
 *
 * NOTE: Does NOT change the actual threshold value, ONLY changes outputs between LEDs and Writing board
 *
 * Used in setNumber() function.
 */

/*
 * AdjustValueInTo0() Sets output communication bits to 0 (000) and the threshold LED to OFF
 */
void AdjustValueInTo0(){
	// value 0 = 000
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set OFF
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo1() Sets output communication bits to 1 (001) and the threshold LED to RED
 */
void AdjustValueInTo1(){
	HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	// Set Red
	   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
	   HAL_Delay(14);
}

/*
 * AdjustValueInTo2() Sets output communication bits to 2 (010) and the threshold LED to YELLOW
 */
void AdjustValueInTo2(){
	// value 2 = 010
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set Yellow
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo3() Sets output communication bits to 3 (110) and the threshold LED to GREEN
 */
void AdjustValueInTo3(){
	// value 3 = 011
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_RESET);

	      // Set Green
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo4() Sets output communication bits to 4 (100) and the threshold LED to CYAN
 */
void AdjustValueInTo4(){
	// value 4 = 100
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Cyan
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo5() Sets output communication bits to 5 (101) and the threshold LED to BLUE
 */
void AdjustValueInTo5(){
	// value 5 = 101
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Blue
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo6() Sets output communication bits to 6 (110) and the threshold LED to MAGENTA
 */
void AdjustValueInTo6(){
	// value 6 = 110
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set Magenta
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
	      HAL_Delay(14);
}

/*
 * AdjustValueInTo7() Sets output communication bits to 7 (111) and the threshold LED to WHITE
 */
void AdjustValueInTo7(){
	// value 7 = 111
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_0_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOA, Discrete_Bit_2_Pin, GPIO_PIN_SET);

	      // Set White
	      HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
	      HAL_Delay(14);
}

/*
 * User_Input_Light_Cycle() goes through each of the seven colors (red, yellow, green, cyan, blue
 * magenta, white) of the threshold LED. Each has a 14ms delay to remain visible to user.
 *
 * NOTE: Only cycles through colors once! To cycle multiple times, call the function multiple times.
 *
 * Used in Button_Debounce_Set()
 */
void User_Input_Light_Cycle() {
   // 1. Set Red
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
   HAL_Delay(14);

   // 2. Set Yellow (Red + Green)
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
   HAL_Delay(14);

   // 3. Set Green
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_RESET);
   HAL_Delay(14);

   // 4. Set Cyan (Green + Blue)
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
   HAL_Delay(14);

   // 5. Set Blue
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
   HAL_Delay(14);

   // 6. Set Magenta (Red + Blue)
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
   HAL_Delay(14);

   // 7. Set White (Red + Green + Blue)
   HAL_GPIO_WritePin(Threshold_Red_GPIO_Port, Threshold_Red_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Green_GPIO_Port, Threshold_Green_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(Threshold_Blue_GPIO_Port, Threshold_Blue_Pin, GPIO_PIN_SET);
   HAL_Delay(14);
}

/*
 * Button_Debounce_Set() changes the valueToAdjust variable based on threshold push-button input. It
 * only updates when a button is pressed for at least 50ms to reduce bounce.
 * While the button is being pressed, the load switch output is set to the highest by calling
 * Reset_The_Whole_B() and the threshold LED flashes using User_Input_Light_Cycle().
 * Outputs by calling setNumber() after every change to valueToAdjust and at the end of the function.
 */
void Button_Debounce_Set() {
   uint8_t currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
   uint8_t currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);
   //setNumber();

   if (currentPlusState == GPIO_PIN_SET || currentMinusState == GPIO_PIN_SET) {
	   Reset_The_Whole_B();
      // set to high state
      if ((HAL_GetTick() - lastDebounceTime) > debounceDelay) {
         // Only update the value if the state has changed
         if ((currentPlusState == GPIO_PIN_SET && lastPlusState != GPIO_PIN_SET) ||
             (currentMinusState == GPIO_PIN_SET && lastMinusState != GPIO_PIN_SET)) {
        	 Reset_The_Whole_B();
            if (currentPlusState == GPIO_PIN_SET) {
               valueToAdjust++;
               if (valueToAdjust >= 7) {
                  valueToAdjust = 7;
                  setNumber();
               }
            }

            else if (currentMinusState == GPIO_PIN_SET) {
               valueToAdjust--;
               if (valueToAdjust < 0) {
                  valueToAdjust = 0;
                  setNumber();
               }
            }
         }

         flashingStartTime = HAL_GetTick();
         while ((HAL_GetTick() - flashingStartTime) < flashingDuration) {
            // Save the last state before reading the current state
            lastPlusState = currentPlusState;
            lastMinusState = currentMinusState;

            currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
            currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);

            // Check for subsequent button presses to restart the
            // timer
            if (currentPlusState == GPIO_PIN_SET || currentMinusState == GPIO_PIN_SET) {
            	Reset_The_Whole_B();
            	User_Input_Light_Cycle();
            	flashingStartTime = HAL_GetTick();  // Restart the 5-second
                                                   // interval

               // Only update the value if the state has changed
               if ((currentPlusState == GPIO_PIN_SET && lastPlusState != GPIO_PIN_SET) ||
                   (currentMinusState == GPIO_PIN_SET && lastMinusState != GPIO_PIN_SET)) {
            	   Reset_The_Whole_B();
                  if (currentPlusState == GPIO_PIN_SET) {
                     valueToAdjust++;
                     if (valueToAdjust >= 7) {
                        valueToAdjust = 7;
                        setNumber();
                     }
                  }

                  else if (currentMinusState == GPIO_PIN_SET) {
                     valueToAdjust--;
                     if (valueToAdjust < 0) {
                        valueToAdjust = 0;
                        setNumber();
                     }
                  }
               }

               // HAL_GPIO_TogglePin(User_Input_Status_Light_GPIO_Port,
               // User_Input_Status_Light_Pin); HAL_Delay(100);
            }
            // 1. Set Red

         }
         lastDebounceTime = HAL_GetTick();
      }

      // Save the last state at the end of the loop
      lastPlusState = currentPlusState;
      lastMinusState = currentMinusState;

      setNumber();

   }

}


//************************************* START OF "SET" STATE FUNCTIONS *************************************//

/*
 * Reset_The_Whole_B() SETS the highest load switch output (8) and SETS the 18650 battery as output.
 * RESETS all other load switch outputs.
 *
 * NOTE: This is the ONLY state function that also sets the state variable. All other states are handled
 * in the state machine.
 */
void Reset_The_Whole_B(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_LOW_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOA, LS_HIGH_Pin, GPIO_PIN_SET);
	   state = CASE_INIT;
}

/*
 * Set_LS_1() SETS the output for load switch 1 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_1(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_2() SETS the output for load switch 2 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_2(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_3() SETS the output for load switch 3 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_3(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_4() SETS the output for load switch 4 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_4(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_5() SETS the output for load switch 5 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_5(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_6() SETS the output for load switch 6 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_6(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_7() SETS the output for load switch 7 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_7(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_RESET);
}

/*
 * Set_LS_8() SETS the output for load switch 8 and RESETS all other load switch outputs.
 *
 * NOTE: Does NOT change output to battery load switches.
 */
void Set_LS_8(){
	   HAL_GPIO_WritePin(GPIOB, LS_1_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_2_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_3_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOB, LS_4_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_5_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_6_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOC, LS_7_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_8_Pin, GPIO_PIN_SET);
}

/*
 * Set_Low() SETS the output for the CMOS battery load switch and RESETS the 18650 load switch output.
 *
 * NOTE: Does NOT change output to other non-battery load switches.
 */
void Set_Low(){
	   HAL_GPIO_WritePin(GPIOC, LS_LOW_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(GPIOA, LS_HIGH_Pin, GPIO_PIN_RESET);
}

/*
 * Set_High() SETS the output for the 18650 battery load switch and RESETS the CMOS load switch output.
 *
 * NOTE: Does NOT change output to other non-battery load switches.
 */
void Set_High(){
	   HAL_GPIO_WritePin(GPIOC, LS_LOW_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, LS_HIGH_Pin, GPIO_PIN_SET);
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
