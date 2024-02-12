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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
float Calculate_Continuous_Average(float sum_voltages, int number_of_voltages);
float Measurement_of_ADC_Voltage_18650();
float Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Current_CMOS();
void Measurement_of_ADC_Current_18650();
//void Continuous_Same_State_Average();
void process_SD_card(void);
void ADC_Select_Voltage18650(void);
void ADC_Select_VoltageCMOS(void);
void ADC_Select_Current18650(void);
void ADC_Select_CurrentCMOS(void);
void readNumber();

float V_18650 = 0.0f;
float V_CMOS = 0.0f;
float V_50gain= 0.0f;
float V_25gain = 0.0f;
float C_CMOS = 0.0f;
float C_18650 = 0.0f;
float Continuous_Average = 0.0f;
int i,j;
unsigned int Switch_State = 0;
float seconds_since_start = 0.0f;
uint32_t start_time_ms = 0;
int valueToAdjust = 0; // Integer value to be adjusted
int current_State = 0;
int previous_State = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//typedef enum { State_CMOS, State_18650 } StateMachine; //State 0 = State_CMOS State 1 = State_18650
//StateMachine state = State_CMOS;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  start_time_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_SET)
	 		  {
	 			  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_SET);
	 			  uint32_t current_time_ms = HAL_GetTick();
	 			  seconds_since_start = (current_time_ms - start_time_ms) / 1000.0f;
	 			  readNumber();
	 			  process_SD_card();
	 		  }
	 		  else
	 		  {
	 			  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);

	 	  }
//	  if (state == State_CMOS) {
//	 	              // if(Voltage_Current_Read)
//	 		  	 	 if(C_CMOS >= 0.03) //threshold
//	 		  	 		 state = State_18650;  // 18650 Mode >= 20mA //1
//	 	          } else if(state == State_18650){
//	 	        	  if(C_18650 <= .01) //threshold
//	 	              state = State_CMOS;  // Cmos Mode > //2
//	 	          }
//
//	 	  else {
////	 		  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
////	 		  HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_RESET);
////	 		  HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_RESET);
////	 		  Error_Handler();
//
//	 	  }
//	 	  switch (state) {
//	 	              case State_CMOS: {  //0
//	 	            	  Switch_State = 0;
//	 	                  break;
//	 	              }
//	 	              case State_18650: { //1
//	 	            	  Switch_State = 1;
//	 	                  break;
//	 	              }
//	 	          }

    /* USER CODE END WHILE */
  }

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_CardDetect_Output_Pin|GPIO_PIN_4|User_Input_Status_Light_Red_Pin|User_Input_Status_Light_Green_Pin
                          |User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Discrete_Bit_0_Pin Discrete_Bit_1_Pin */
  GPIO_InitStruct.Pin = Discrete_Bit_0_Pin|Discrete_Bit_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CardDetect_Input_Pin */
  GPIO_InitStruct.Pin = SD_CardDetect_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CardDetect_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CardDetect_Output_Pin PA4 User_Input_Status_Light_Red_Pin User_Input_Status_Light_Green_Pin
                           User_Input_Status_Light_Blue_Pin */
  GPIO_InitStruct.Pin = SD_CardDetect_Output_Pin|GPIO_PIN_4|User_Input_Status_Light_Red_Pin|User_Input_Status_Light_Green_Pin
                          |User_Input_Status_Light_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Discrete_Bit_2_Pin */
  GPIO_InitStruct.Pin = Discrete_Bit_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Discrete_Bit_2_GPIO_Port, &GPIO_InitStruct);

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
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void process_SD_card(void)
{
    FATFS FatFs;   // Fatfs handle
    FIL fil;       // File handle
    FRESULT fres;  // Result after operations

    // Buffer for storing the complete string to write
    char writeBuffer[50]; // Adjust the size based on your needs

    // Attempt to mount the SD Card
    fres = f_mount(&FatFs, "", 1); // 1=mount now
    if (fres != FR_OK) return; // Exit if fail to mount

    // Open or create the file and append data
    fres = f_open(&fil, "TestingReadings.csv", FA_WRITE | FA_READ | FA_OPEN_APPEND);
    if (fres != FR_OK) {
        f_mount(NULL, "", 0); // Dismount the SD card if fail to open
        return; // Exit if fail to open/create the file
    }

    // Prepare the data string
    snprintf(writeBuffer, sizeof(writeBuffer),
             "%.3f,%.3f,%.3f,%.3f,%.3f,%d,\n",
             seconds_since_start, // Time
             V_18650, // 18650 Voltage
             C_18650, // 18650 Current
             V_CMOS, // CMOS Voltage
             C_CMOS, // CMOS Current
			 valueToAdjust); // Switch State

    // Write the prepared string to the file
    f_puts(writeBuffer, &fil);

    // Close the file to ensure data is written to the SD card
    f_close(&fil);

    // Dismount the SD card
    f_mount(NULL, "", 0);
}


float Measurement_of_ADC_Voltage_18650(){
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
	       }
	    HAL_ADC_Stop(&hadc);
	    return V_18650 = rawValue1 * V_stepSize;
}
float Measurement_of_ADC_Voltage_CMOS(){
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
	       }
	    HAL_ADC_Stop(&hadc);
	    return V_CMOS = rawValue1 * V_stepSize;
}

void Measurement_of_ADC_Current_18650(){
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
	           C_18650 = (((rawValue1 * V_stepSize)/50)/.0299562); //I_load = ((V_ADC / 50 gain) / .03 calibrated shunt)
	       }
	    HAL_ADC_Stop(&hadc);
}

void Measurement_of_ADC_Current_CMOS(){
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
	           C_CMOS = (((rawValue1 * V_stepSize)/20)/4.713492); // I_load = (( V_ADC / 20 Gain ) / 4.71 calibrated shunt )
	       }
	    HAL_ADC_Stop(&hadc);
}


void ADC_Select_Voltage18650(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_15;
sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void ADC_Select_VoltageCMOS(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_13;
sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}

}

void ADC_Select_Current18650(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_14;
sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
	    Error_Handler();
}
}

void ADC_Select_CurrentCMOS(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_12;
sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
{
Error_Handler();
}

}

void readNumber() {

	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 &&
		HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
		HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0 ) {
		HAL_Delay(5);
		valueToAdjust = 1;
		// Set Red
		HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

	}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 &&
		HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
		HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0 ) {
		HAL_Delay(5);
		valueToAdjust = 2;
		// Set Yellow
		HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

	}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0 ) {
			HAL_Delay(5);
			valueToAdjust = 3;
			// Set Green
			HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_RESET);

		}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 &&
			HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
			HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1 ) {
			HAL_Delay(5);
			valueToAdjust = 4;
			// Set Cyan
			HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

		}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
			HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1 ) {
			HAL_Delay(5);
			valueToAdjust = 5;
			// Set Blue
			HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

		}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 &&
			HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1 ) {
			HAL_Delay(5);
			valueToAdjust = 6;
			// Set Magenta
			HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

		}
	if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
			HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1 ) {
			HAL_Delay(5);
			valueToAdjust = 7;
			// Set White
			HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin, GPIO_PIN_SET);

		}


}

//void Continuous_Same_State_Average() {
//float test_voltages[] = {1.4, 1.41, 1.42, 1.43, 1.44, 1.45, 1.46, 1.47, 1.48, 1.49, 1.5};
//float voltage_sum = 0;
//int x = 0;
//
//int array_length = sizeof(test_voltages) / sizeof(test_voltages[0]);
//for (i = 0; i < array_length; i++) {
//    voltage_sum += test_voltages[i];
//    x++;
//}
//
//Continuous_Average = Calculate_Continuous_Average(voltage_sum, x);
//}
//
//float Calculate_Continuous_Average(float sum_voltages, int number_of_voltages){
//	return sum_voltages/number_of_voltages;
//}




//		if((current_State != previous_State) || ((current_State && previous_State) == 0)){
//			// First, check if any of the pins are set
//				    if(HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) ||
//				       HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) ||
//				       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    	// Check each combination of pins and set valueToAdjust accordingly
//				    		     if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		        !HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		        !HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 1;
//				    		        current_State = 1;
//				    		    }
//				    		     if (!HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               !HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 2;
//				    		        current_State = 2;
//
//				    		    }  if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               !HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 3;
//				    		        current_State = 3;
//				    		    }  if (!HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               !HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 4;
//				    		        current_State = 4;
//				    		    }  if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               !HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 5;
//				    		        current_State = 5;
//				    		    }  if (!HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 6;
//				    		        current_State = 6;
//				    		    }  if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) &&
//				    		               HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin)) {
//				    		        HAL_Delay(5);
//				    		        valueToAdjust = 7;
//				    		        current_State = 7;
//				    		    }
//				    }
//				    previous_State = current_State;
//
//		}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
