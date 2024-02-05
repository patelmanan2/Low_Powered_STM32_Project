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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
float Calculate_Continuous_Average(float sum_voltages, int number_of_voltages);
float Measurement_of_ADC_Voltage_18650();
float Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Current_CMOS();
void Measurement_of_ADC_Current_18650();
void Continuous_Same_State_Average();
void process_SD_card(void);
void ADC_Select_Voltage18650(void);
void ADC_Select_VoltageCMOS(void);
void ADC_Select_Current18650(void);
void ADC_Select_CurrentCMOS(void);

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
static uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 50;  // milliseconds
const uint32_t flashingDuration = 5000; // 5 seconds
uint32_t flashingStartTime = 0;
float valueToAdjust = 0; // Integer value to be adjusted
uint8_t lastPlusState = GPIO_PIN_RESET;
uint8_t lastMinusState = GPIO_PIN_RESET;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum { State_CMOS, State_18650 } StateMachine; //State 0 = State_CMOS State 1 = State_18650
StateMachine state = State_CMOS;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  start_time_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
	      uint8_t currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);

	      if (currentPlusState == GPIO_PIN_SET || currentMinusState == GPIO_PIN_SET) {
	    	  //set to high state
	          if ((HAL_GetTick() - lastDebounceTime) > debounceDelay) {
	              // Only update the value if the state has changed
	              if ((currentPlusState == GPIO_PIN_SET && lastPlusState != GPIO_PIN_SET) ||
	                  (currentMinusState == GPIO_PIN_SET && lastMinusState != GPIO_PIN_SET)) {
	                  if (currentPlusState == GPIO_PIN_SET) {
	                      valueToAdjust++;
	                  } else if (currentMinusState == GPIO_PIN_SET) {
	                      valueToAdjust--;
	                  }
	              }

	              flashingStartTime = HAL_GetTick();
	              while ((HAL_GetTick() - flashingStartTime) < flashingDuration) {
	                  // Save the last state before reading the current state
	                  lastPlusState = currentPlusState;
	                  lastMinusState = currentMinusState;

	                  currentPlusState = HAL_GPIO_ReadPin(GPIOC, Plus_Pin);
	                  currentMinusState = HAL_GPIO_ReadPin(GPIOC, Minus_Pin);

	                  // Check for subsequent button presses to restart the timer
	                  if (currentPlusState == GPIO_PIN_SET || currentMinusState == GPIO_PIN_SET) {
	                      flashingStartTime = HAL_GetTick(); // Restart the 5-second interval

	                      // Only update the value if the state has changed
	                      if ((currentPlusState == GPIO_PIN_SET && lastPlusState != GPIO_PIN_SET) ||
	                          (currentMinusState == GPIO_PIN_SET && lastMinusState != GPIO_PIN_SET)) {
	                          if (currentPlusState == GPIO_PIN_SET) {
	                              valueToAdjust++;
	                          } else if (currentMinusState == GPIO_PIN_SET) {
	                              valueToAdjust--;
	                          }
	                      }
	                  }

	                  HAL_GPIO_TogglePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin);
	                  HAL_Delay(100);
	              }

	              HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
	              lastDebounceTime = HAL_GetTick();
	          }
	      }

	      // Save the last state at the end of the loop
	      lastPlusState = currentPlusState;
	      lastMinusState = currentMinusState;




	      //BootUp(); < 1 second


	  if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_SET)
	 		  {
	 			  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_SET);
	 			  uint32_t current_time_ms = HAL_GetTick();
	 			  seconds_since_start = (current_time_ms - start_time_ms) / 1000.0f;
	 			  Continuous_Same_State_Average();
	 			  process_SD_card();
	 		  }
	 		  else
	 		  {
	 			  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);

	 	  }
	  if (state == State_CMOS) {
	 	              // if(Voltage_Current_Read)
	 		  	 	 if(C_CMOS >= 0.03) //threshold
	 		  	 		 state = State_18650;  // 18650 Mode >= 20mA //1
	 	          } else if(state == State_18650){
	 	        	  if(C_18650 <= .01) //threshold
	 	              state = State_CMOS;  // Cmos Mode > //2
	 	          }

	 	  else {
//	 		  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
//	 		  HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_RESET);
//	 		  HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_RESET);
//	 		  Error_Handler();

	 	  }
	 	  switch (state) {
	 	              case State_CMOS: {  //0
	 	            	  Switch_State = 0;
	 	                  break;
	 	              }
	 	              case State_18650: { //1
	 	            	  Switch_State = 1;
	 	                  break;
	 	              }
	 	          }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SD_CardDetect_Output_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |Load_Switch_CMOS_Pin|Load_Switch_18650_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Minus_Pin Plus_Pin */
  GPIO_InitStruct.Pin = Minus_Pin|Plus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CardDetect_Input_Pin */
  GPIO_InitStruct.Pin = SD_CardDetect_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CardDetect_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CardDetect_Output_Pin PA4 */
  GPIO_InitStruct.Pin = SD_CardDetect_Output_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 PB13
                           PB14 PB15 PB3 PB4
                           Load_Switch_CMOS_Pin Load_Switch_18650_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |Load_Switch_CMOS_Pin|Load_Switch_18650_Pin;
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
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void process_SD_card( void )
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  FRESULT     fres;                 //Result after operations

//Testing Buff Sizes are maybe larger than they need to be currently *CHANGE LATER*
  char buf[1];
  char res_18650[7];
  char res_C18650[7];
  char res_CCMOS[9];
  char res_CMOS[7];
  char res_time[32];
  char res_SwitchState[7];

  do
  {
    //Mount the SD Card
    fres = f_mount(&FatFs, "", 1);    //1=mount now
    if (fres != FR_OK)
    {
      //printf("No SD Card found : (%i)\r\n", fres);
      break;
    }
    //Open the file
    fres = f_open(&fil, "TestingReadings.csv", FA_WRITE | FA_READ | FA_OPEN_APPEND);
    if(fres != FR_OK)
    {
      //printf("File creation/open Error : (%i)\r\n", fres);
      break;
    }
    //printf("Writing data!!!\r\n");
    //write the data

    //Write the Time for each write
    sprintf(res_time,"%.3f,", valueToAdjust); //Position A //seconds since start, changed for testing
    f_puts(res_time, &fil);

    //Write the 18650 Voltage Readings
    sprintf(res_18650, "%.3f,", Continuous_Average); //Position B
    f_puts(res_18650, &fil);

    //Write the 18650 Current Readings
    sprintf(res_C18650, "%.3f,", C_18650); //Position C
    f_puts(res_C18650, &fil);

    //Write the CMOS Voltage Readings
    sprintf(res_CMOS, "%.3f,", V_CMOS); //Position D
    f_puts(res_CMOS,&fil);

    //Write the CMOS Current Readings
    sprintf(res_CCMOS, "%.3f,", C_CMOS); //Position E
    f_puts(res_CCMOS, &fil);

    //Writes the Switch State, 0 = State_CMOS / 1 = State_18650
    sprintf(res_SwitchState,"%u, \n", Switch_State); //Position F
    f_puts(res_SwitchState, &fil);

    //close your file
    f_close(&fil);
    //Open the file
    //read the data
    f_gets(buf, sizeof(buf), &fil);
    //printf("Read Data : %s\n", buf);
    //close your file
    f_close(&fil);
    //printf("Closing File!!!\r\n");
#if 0
    //Delete the file.
    fres = f_unlink(Readings.txt);
    if (fres != FR_OK)
    {
      //printf("Cannot able to delete the file\n");
    }
#endif
  } while(0);
  //We're done, so de-mount the drive
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

void Continuous_Same_State_Average() {
float test_voltages[] = {1.4, 1.41, 1.42, 1.43, 1.44, 1.45, 1.46, 1.47, 1.48, 1.49, 1.5};
float voltage_sum = 0;
int x = 0;

int array_length = sizeof(test_voltages) / sizeof(test_voltages[0]);
for (i = 0; i < array_length; i++) {
    voltage_sum += test_voltages[i];
    x++;
}

Continuous_Average = Calculate_Continuous_Average(voltage_sum, x);
}

float Calculate_Continuous_Average(float sum_voltages, int number_of_voltages){
	return sum_voltages/number_of_voltages;
}


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
