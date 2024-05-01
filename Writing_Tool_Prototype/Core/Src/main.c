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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Measurement_of_ADC_Voltage_18650();
void Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Current_CMOS();
void Measurement_of_ADC_Current_18650();
void Measurement_of_Load_Voltage();
void process_SD_card(void);
void ADC_Select_Voltage18650();
void ADC_Select_VoltageCMOS();
void ADC_Select_Current18650();
void ADC_Select_CurrentCMOS();
void ADC_Select_Load_Voltage();
void readNumber();
int write_num = 0;
float Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_18650(float V_DiffAmp, int state);
float Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_CMOS(float V_DiffAmp, int state);
int read_SD(void);
void communicate_value(int number);

float V_18650 = 0.0f;
float V_CMOS = 0.0f;
float V_DiffAmp_18650 = 0.0f;
float V_DiffAmp_CMOS = 0.0f;
float C_CMOS = 0.0f;
float C_18650 = 0.0f;
float Load_Voltage = 0.0f;
int i, j;
unsigned int Switch_State = 0;
float seconds_since_start = 0.0f;
uint32_t start_time_ms = 0;
int valueToAdjust = 0;  // Integer value to be adjusted

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  int number = read_SD();
  communicate_value(number);
  HAL_Delay(2000);

   char msg[128];
   start_time_ms = HAL_GetTick();
   HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1) {
      if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_SET) {
         HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_SET);
         uint32_t current_time_ms = HAL_GetTick();
         seconds_since_start = (current_time_ms - start_time_ms) / 1000.0f;
         readNumber();
         Measurement_of_ADC_Voltage_18650();
         Measurement_of_ADC_Voltage_CMOS();
         Measurement_of_ADC_Current_CMOS();
         Measurement_of_ADC_Current_18650();
         Measurement_of_Load_Voltage();
         C_CMOS = Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_CMOS(V_DiffAmp_CMOS, valueToAdjust);
         C_18650 = Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_18650(V_DiffAmp_18650, valueToAdjust);

         process_SD_card();
         sprintf(msg, "%.3f,%.3f,%.5f,%.3f,%.5f,%d,%d\r\n",
             		  seconds_since_start, //time (in seconds)
         			  V_18650,		  // 18650 Voltage
                       C_18650,        // 18650 Current
                       V_CMOS,         // CMOS Voltage
         			  C_CMOS,         // CMOS Current
                       valueToAdjust,  // Threshold

         			  write_num);//Total number of measurements
               HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      } else {
         HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      write_num++;
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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



  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
   /* e.g. write a character to the UART3 and Loop until the end of transmission
    */
   // HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
   return ch;
}

/*
 * process_SD_card() allows stored values to be written to the SD card using the FAT32 file system under
 * the FATFS library. It does so using the following basic steps:
 * 1) Mount the SD card
 * 2) Create the file or append to it if it already exists
 * 3) Create a string buffer that stores all measurements as comma-separated values
 * 4) Write to the SD card and then close the file
 * 5) Dismount the SD card
 *
 * Note: The ACT light on the SD card breakout board should be flickering if functioning properly. It is
 *       not working properly when the light either stays ON or stays OFF. In that case, check that the
 *       breakout board is correctly wired.
 */
void process_SD_card(void) {
   FATFS FatFs;   // Fatfs handle
   FIL fil;       // File handle
   FRESULT fres;  // Result after operations

   // Buffer for storing the complete string to write
   char writeBuffer[500];  // Adjust the size based on your needs

   // Attempt to mount the SD Card
   fres = f_mount(&FatFs, "", 1);  // 1=mount now
   if (fres != FR_OK) return;      // Exit if fail to mount

   // Open or create the file and append data
   fres = f_open(&fil, "Readings.csv", FA_WRITE | FA_READ | FA_OPEN_APPEND);
   if (fres != FR_OK) {
      f_mount(NULL, "", 0);  // Dismount the SD card if fail to open
      return;                // Exit if fail to open/create the file
   }

   // Prepare the data string
   snprintf(writeBuffer, sizeof(writeBuffer), "%.3f,%.3f,%.5f,%.3f,%.5f,%.3f,%d,%d,\n",
            seconds_since_start,  // Time
            V_18650,              // 18650 Voltage
            C_18650,              // 18650 Current
            V_CMOS,               // CMOS Voltage
            C_CMOS,
			Load_Voltage,// CMOS Current
            valueToAdjust,
			write_num);       // Switch State

   // Write the prepared string to the file
   f_puts(writeBuffer, &fil);

   // Close the file to ensure data is written to the SD card
   f_close(&fil);

   // Dismount the SD card
   f_mount(NULL, "", 0);
}

/*
 * read_SD() checks if there is a custom threshold switching value written on the SD card in "config.txt" and returns
 * the value converted from a String to a integer. If there is no file, this function will return a 0.
 *
 * NOTE: If the file is present, this function will attempt to read regardless of actual content. Assumes the value is
 * given as whole numbers, in mA, and up to four digits (between 0 - 2000 mA) for communicate_vlaue() to function
 * properly.
 *
 * @return an integer that represents the the custom switching threshold value in mA. Will return 0 if file does not
 * exist
 */
int read_SD(void)
{
	FATFS FatFs;   // Fatfs handle
	   FIL fil;       // File handle
	   FRESULT fres;  // Result after operations

	   // Buffer for storing the complete string to read
	   char readBuffer[6];  // Adjust the size based on your needs

	   // Attempt to mount the SD Card
	   fres = f_mount(&FatFs, "", 1);  // 1=mount now
	   if (fres != FR_OK) return 0;      // Exit if fail to mount

	   // Open or create the file and append data
	   fres = f_open(&fil, "config.txt", FA_READ);
	   if (fres != FR_OK)
	   {
	      f_mount(NULL, "", 0);  // Dismount the SD card if fail to open
	      return 0;                // Exit if fail to open the file
	   }

	   f_gets(readBuffer, sizeof(readBuffer), &fil); //store read values to string buffer

	   // Close the file
	   f_close(&fil);

	   // Dismount the SD card
	   f_mount(NULL, "", 0);

	   return atoi(readBuffer);

}//end of read_SD()

/*
 * communicate_value(number) sends the value given by number to be read by the Switching board (typically, read_SD()).
 *  If the number to be sent is greater than zero, it will cycle through each of the decimal places until the number
 *  is complete and/or when it reaches 4 digits. Each cycle will %10 to obtain the last digit of the number. If that
 *  value is greater than 0, then toggle the DATA pin [digit] amount of times. When the decimal "place" is complete,
 *  the DIGIT pin is toggled and the number is now set to number/10 to reduce the number by a factor of 10 and repeats
 *  through the function until either all numbers have been read or the number of decimal "places" read = 4.
 *
 * For example, if the number was 1234:
 *
 * digit = 1234%10 = 4
 * Toggle DATA pin 4 times (DATA pin output: 10101010)
 * Toggle DIGIT pin        (Digit pin output:        10)
 * number = 1234/10 = 123
 * NEXT PLACE! (place = 1)
 *
 * digit = 123%10 = 3
 * Toggle DATA pin 3 times (DATA pin output: 101010)
 * Toggle DIGIT pin        (Digit pin output:      10)
 * number = 123/10 = 12
 * NEXT PLACE! (place = 2)
 *
 * digit = 12%10 = 2
 * Toggle DATA pin 2 times (DATA pin output: 1010)
 * Toggle DIGIT pin        (Digit pin output:    10)
 * number = 12/10 = 1
 * NEXT PLACE! (place = 3)
 *
 * digit = 1%10 = 1
 * Toggle DATA pin 1 times (DATA pin output: 10)
 * Toggle DIGIT pin        (Digit pin output:  10)
 * number = 1/10 = 0
 * Exit (place = 4)
 *
 * Uninterrupted output will look like the following:
 *                   4       3     2   1
 * DATA pin: 1010101000101010001010001000
 * DIGIT pin:0000000010000000100000100010
 *
 * @param: number is an integer that represents the threshold value between 0 - 9999 to be sent from the Writing
 * board to the Switching board (typically obtained from read_SD())
 *
 */
void communicate_value(int number)
{
	int place = 0;

	if(number > 0) //if the given number is greater than zero
	{
		while(place < 4)//while the number of "places" (1's, 10's, 100's, 1000's place) sent out is less than 4 (starting at 0 for a total of 4 digits)
		{
			int digit = number%10; //get the last digit of the given number

			while(digit > 0)
			{
				// Set Red
				HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET); //Send "1" on data pin
				HAL_Delay(7);
				HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_RESET);//Send "0" on data pin
				HAL_Delay(5);
				digit--;
			}//digit finished sending

			// Set Green
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);//Send "1" on digit pin
			HAL_Delay(7);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);//Send "0" on digit pin
			HAL_Delay(5);

			number = number/10; //set the number to be a factor of 10 less (if number was 1000, number is now 100)
			place++; //indicates the "place" of the number has increased to the next "place"
		}

	}
	else //otherwise send that the number is 0
	{
		while(place < 4)
		{
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_SET);//Send "1" on digit pin
			HAL_Delay(7);
			HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin, GPIO_PIN_RESET);//Send "0" on digit pin
			HAL_Delay(5);

			place++;
		}
		// Set Blue
				      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin,
				                        GPIO_PIN_RESET);
				      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
				                        GPIO_PIN_RESET);
				      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
				                        GPIO_PIN_SET);
		return;
	}

}

//**************************************** START OF ADC-RELATED FUNCTIONS ****************************************//

/*
 * Measurement_of_ADC functions perform ADC measurements and conversions for each respective channel. This
 * is done with the following steps:
 * 1) Select the respective ADC channel register using CHSELR (uses standard STM bit-masking)
 * 2) Select the respective ADC channel (using the respective ADC_Select function)
 * 3) Start the ADC
 * 4) Read the raw value
 * 5) Convert to respective measurement units and store to respective global variable
 * 6) Stop the ADC
 *
 * NOTE: Some of the ADC channels on the writing board differ from the channels used on the Switching board.
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
   float V_ref = 3.3;  // This is known for each micro controller from data sheet, V_ref = power supply in
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
      V_18650 = ((rawValue1 * V_stepSize) * (1/.65));

      // V_18650 = rawValue1;
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
   uint16_t rawValue2;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue2 = HAL_ADC_GetValue(&hadc);
      V_CMOS = ((rawValue2 * V_stepSize) * (1/.65));
      // V_CMOS = rawValue2;
   }
   HAL_ADC_Stop(&hadc);
}

/*
 * Measurement_of_ADC_CURRENT_18650() performs an ADC measurement and conversion for the CURRENT of the
 * 18650 battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x0200h (channel 9)
 * Calls ADC_Select_Current18650() to set channel
 * Converted values store to C_18650
 */
void Measurement_of_ADC_Current_18650() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x0200;
   ADC_Select_Current18650();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue3;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue3 = HAL_ADC_GetValue(&hadc);
      V_DiffAmp_18650 = ((rawValue3 * V_stepSize));
      // C_18650 = rawValue3;
      // 50)/.0299562); //I_load = ((V_ADC / 50 gain) / .03 calibrated shunt)
   }
   HAL_ADC_Stop(&hadc);
}

/*
 * Measurement_of_ADC_Current_CMOS() performs an ADC measurement and conversion for the CURRENT of the
 * CMOS battery. *Refer to Measurement_of_ADC steps above for more detail on steps*
 *
 * CHSELR is set to 0x1000h (channel 8)
 * Calls ADC_Select_CurrentCMOS() to set channel
 * Converted values store to C_CMOS
 */
void Measurement_of_ADC_Current_CMOS() {
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
   uint16_t rawValue4;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue4 = HAL_ADC_GetValue(&hadc);
      V_DiffAmp_CMOS = ((rawValue4 * V_stepSize));
      // C_CMOS = rawValue4;
      /// 20)/4.713492); // I_load = (( V_ADC / 20 Gain ) / 4.71 calibrated shunt
      /// )
   }
   HAL_ADC_Stop(&hadc);
}

void Measurement_of_Load_Voltage() {
   HAL_ADC_Stop(&hadc);
   HAL_ADC_Init(&hadc);
   float V_ref = 3.3;  // This is known for each micro controller from data
   // sheet, V_ref = power supply in
   float ADC_resolution = (4096 - 1);  // 2^12 - 1
   float V_stepSize = V_ref / ADC_resolution;
   // ADC
   /* Start ADC Conversion for ADC1 */
   ADC1->CHSELR = 0x0004;
   ADC_Select_CurrentCMOS();
   HAL_ADC_Start(&hadc);
   uint16_t rawValue5;
   if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {
      /* Read the ADC1 value */
      rawValue5 = HAL_ADC_GetValue(&hadc);
      Load_Voltage = ((rawValue5 * V_stepSize) * (1/.65));
      // C_CMOS = rawValue4;
      /// 20)/4.713492); // I_load = (( V_ADC / 20 Gain ) / 4.71 calibrated shunt
      /// )
   }
   HAL_ADC_Stop(&hadc);
}

/*
 * ADC_Select_Voltage18650() selects the channel that relates to the VOLTAGE of the 18650 battery.
 * It sets sConfig to its respective channel (15) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_Load_Voltage() {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_2;
   sConfig.Rank = 0;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

void ADC_Select_Voltage18650() {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_15;
   sConfig.Rank = 0;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_VoltageCMOS() selects the channel that relates to the VOLTAGE of the CMOS battery.
 * It sets sConfig to its respective channel (13) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_VoltageCMOS() {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_13;
   sConfig.Rank = 0;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_Current18650() selects the channel that relates to the CURRENT of the 18650 battery.
 * It sets sConfig to its respective channel (9) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_Current18650() {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_9;
   sConfig.Rank = 0;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

/*
 * ADC_Select_CurrentCMOS() selects the channel that relates to the CURRENT of the CMOS battery.
 * It sets sConfig to its respective channel (12) and channel rank. It then checks if the channel
 * has been configured correctly.
 */
void ADC_Select_CurrentCMOS() {
   ADC_ChannelConfTypeDef sConfig = {0};
   sConfig.Channel = ADC_CHANNEL_12;
   sConfig.Rank = 0;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
      Error_Handler();
   }
}

//******************************** START OF BOARD COMMUNICATION FUNCTIONS ********************************//

/*
 * readNumber() reads the inputs from the discrete bits (output from Switching board) and sets the
 * valueToAdjust variable and writing LED color to its respective value and color.
 *
 * It is used to read both threshold input and which state the switching board is in.
 */
void readNumber() {
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0) {
      HAL_Delay(5);
      valueToAdjust = 1;
      // Set LED to Off
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_RESET);
   }

   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0) {
      HAL_Delay(5);
      valueToAdjust = 2;
      // Set Red
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_RESET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0) {
      HAL_Delay(5);
      valueToAdjust = 3;
      // Set Yellow
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_RESET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 0) {
      HAL_Delay(5);
      valueToAdjust = 4;
      // Set Green
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_RESET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1) {
      HAL_Delay(5);
      valueToAdjust = 5;
      // Set Cyan
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_SET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 0 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1) {
      HAL_Delay(5);
      valueToAdjust = 6;
      // Set Blue
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_SET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 0 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1) {
      HAL_Delay(5);
      valueToAdjust = 7;
      // Set Magenta
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_SET);
   }
   if (HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_0_Pin) == 1 && HAL_GPIO_ReadPin(GPIOC, Discrete_Bit_1_Pin) == 1 &&
       HAL_GPIO_ReadPin(GPIOB, Discrete_Bit_2_Pin) == 1) {
      HAL_Delay(5);
      valueToAdjust = 8;
      // Set White
      HAL_GPIO_WritePin(User_Input_Status_Light_Red_GPIO_Port, User_Input_Status_Light_Red_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Green_GPIO_Port, User_Input_Status_Light_Green_Pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(User_Input_Status_Light_Blue_GPIO_Port, User_Input_Status_Light_Blue_Pin,
                        GPIO_PIN_SET);
   }
}

/*
 * Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current functions allow the conversion equations for each load
 * switch to be used on the Writing board. This function determines which load switch it has just measured given
 * the voltage at the differential amplifier (V_DiffAmp) and the state (normally given by valueToAdjust) to return
 * a converted float that represents the measured current (in amps).
 *
 * Based on which state the load switch is in will determine the conversion value for the measured current.
 */

/*
 * Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_CMOS is used to convert the differential voltage of the
 * "LOW" battery to a current value (in Amps).
 * @param V_DiffAmp is the voltage of the LOW battery at the differential amplifier (typically V_DiffAmp_CMOS)
 * @param state is the state the switching board is in (typically valueToAdjust)
 *
 * @return Converted value representing the current of the LOW battery in Amps
 *
 * NOTE: If the LOW battery is not currently active (the state is not 0-3), then it will return a 0
 */
float Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_CMOS(float V_DiffAmp, int state)
{
	switch(state)
	{
	case 1: return (V_DiffAmp/905); break;   //conversion for LS_1
	case 2: return (V_DiffAmp/99.); break;  //conversion for LS_2
	case 3: return (V_DiffAmp/10.4); break;  //conversion for LS_3
	case 4: return (V_DiffAmp/1.22); break; //conversion for LS_4
	default: return 0; break;
	}
}

/*
 * Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_18650 is used to convert the differential voltage of the
 * "HIGH" battery to a current value (in Amps).
 * @param V_DiffAmp is the voltage of the HIGH battery at the differential amplifier (typically V_DiffAmp_18650)
 * @param state is the state the switching board is in (typically valueToAdjust)
 *
 * @return Converted value representing the current of the HIGH battery in Amps
 *
 * NOTE: If the HIGH battery is not currently active (the state is not 4-7), then it will return a 0
 */
float Convert_Measurement_of_ADC_Voltage_DiffAmp_to_Current_18650(float V_DiffAmp, int state)
{
	switch(state)
	{
	case 5: return (V_DiffAmp/885); break;   //conversion for LS_5
	case 6: return (V_DiffAmp/98.7); break;   //conversion for LS_6
	case 7: return (V_DiffAmp/11.0); break;  //conversion for LS_7
	case 8: return (V_DiffAmp/1.17); break;  //conversion for LS_8
	default: return 0; break;
	}
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
   while (1) {
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
   /* User can add his own implementation to report the file name and line
      number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
      line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
