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
  * Latest 11/09
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void process_SD_card( void );
void process_SD_card1( void );
void ADC_Select_Voltage18650(void);
void ADC_Select_VoltageCMOS(void);
void ADC_Select_Current18650(void);
void ADC_Select_CurrentCMOS(void);
void Measurement_of_ADC_Voltage_18650();
void Measurement_of_ADC_Voltage_CMOS();
void Measurement_of_ADC_Current_CMOS();
void Measurement_of_ADC_Current_18650();

float V_18650 = 0.0f;
float V_CMOS = 0.0f;
float C_CMOS = 0.0f;
float C_18650 = 0.0f;
unsigned int Switch_State = 0;
unsigned int seconds_since_start;

//array of current values


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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //process_SD_card();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  process_SD_card();
	  seconds_since_start++;
	  if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_SET)
	  {
		  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_SET);//error light is OFF and ready to run
		  // HAL_GPIO_WritePin(SD_CardDetect_Output_ReadyToRun_GPIO_Port, SD_CardDetect_Output_ReadyToRun__Pin, GPIO_PIN_SET); //ready light is ON ready to run
	  if (state == State_CMOS) {
	              // if(Voltage_Current_Read)
		  	 	 if(C_CMOS >= 2.0) //threshold
		  	 		 state = State_18650;  // 18650 Mode >= 20mA //1
	          } else if(state == State_18650){
	        	  if(C_18650 <= 1.0) //threshold
	              state = State_CMOS;  // Cmos Mode > //2
	          }
	  }

	  else {
		  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
		  Error_Handler();

	  }


	  switch (state) {
	              case State_CMOS: {  //0
	            	  HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_RESET);
	            	  HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_SET);
	            	  Measurement_of_ADC_Voltage_CMOS();
	            	  Measurement_of_ADC_Current_CMOS();
	            	  Switch_State = 0;
	                  break;
	              }

	              case State_18650: { //1
	            	  HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_RESET);
	            	  HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_SET);
	            	  Measurement_of_ADC_Current_18650();
	            	  Measurement_of_ADC_Voltage_18650();

	            	  Switch_State = 1;
	            	   // temporaRY HERE FOR TEST
	                  break;
	              }
	          }

//	  Measurement_of_ADC_Voltage_18650();
//	  Measurement_of_ADC_Voltage_CMOS();
//	  Measurement_of_ADC_Current_CMOS();
//	  Measurement_of_ADC_Current_18650();
//
//	  seconds_since_start++;
//	  process_SD_card();
//
//	  if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_SET)
//	  	  	  {
//		  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_SET);
//	  	  	  }
//
//	  	  else if (HAL_GPIO_ReadPin(SD_CardDetect_Input_GPIO_Port, SD_CardDetect_Input_Pin) == GPIO_PIN_RESET) {
//	  		HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);
//	  		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
//	  	  }
//
//	  //Testing Load Switch at 2 seconds
//	  	HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_SET);
//	  	HAL_Delay(2000);
//	  	HAL_GPIO_WritePin(Load_Switch_CMOS_GPIO_Port, Load_Switch_CMOS_Pin, GPIO_PIN_RESET);
//	  	HAL_Delay(500);
//	  	HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_SET);
//	  	HAL_Delay(2000);
//	  	HAL_GPIO_WritePin(Load_Switch_18650_GPIO_Port, Load_Switch_18650_Pin, GPIO_PIN_RESET);
//	  	HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  HAL_GPIO_WritePin(SD_CardDetect_Output_GPIO_Port, SD_CardDetect_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Load_Switch_CMOS_Pin|Load_Switch_18650_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CardDetect_Input_Pin */
  GPIO_InitStruct.Pin = SD_CardDetect_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CardDetect_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CardDetect_Output_Pin */
  GPIO_InitStruct.Pin = SD_CardDetect_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CardDetect_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Load_Switch_CMOS_Pin Load_Switch_18650_Pin */
  GPIO_InitStruct.Pin = Load_Switch_CMOS_Pin|Load_Switch_18650_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void process_SD_card( void )
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  FRESULT     fres;                 //Result after operations

  char buf[100];
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
    //printf("SD Card Mounted Successfully!!!\r\n");
    //Read the SD Card Total size and Free Size
//    FATFS *pfs;
//    DWORD fre_clust;
//    uint32_t totalSpace, freeSpace;
//    f_getfree("", &fre_clust, &pfs);
//    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
    //printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
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
    sprintf(res_time,"%u,", seconds_since_start); //Position A
    f_puts(res_time, &fil);

    //Write the 18650 Voltage Readings
    sprintf(res_18650, "%.3f,", V_18650); //Position B
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
  //printf("SD Card Unmounted Successfully!!!\r\n");
}


void Measurement_of_ADC_Voltage_18650(){
	float V_ref = 3.3;  // This is known for each micro controller from data
		// sheet, V_ref = power supply in
		float ADC_resolution = (4096 - 1);  // 2^12 - 1
		float V_stepSize = V_ref / ADC_resolution;
		// ADC
	    /* Start ADC Conversion for ADC1 */
	    ADC_Select_Voltage18650();
	    HAL_ADC_Start(&hadc1);
	    uint16_t rawValue1;
	       if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
	           /* Read the ADC1 value */
	           rawValue1 = HAL_ADC_GetValue(&hadc1);
	           V_18650 = rawValue1 * V_stepSize;  //
	       }
	    HAL_ADC_Stop(&hadc1);
}

void Measurement_of_ADC_Voltage_CMOS(){
	float V_ref = 3.3;  // This is known for each micro controller from data
		// sheet, V_ref = power supply in
		float ADC_resolution = (4096 - 1);  // 2^12 - 1
		float V_stepSize = V_ref / ADC_resolution; // 3.3/4095 = 0.0008V
		// ADC
	    /* Start ADC Conversion for ADC1 */
	    ADC_Select_VoltageCMOS();
	    HAL_ADC_Start(&hadc1);
	    uint16_t rawValue1;
	       if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
	           /* Read the ADC1 value */
	           rawValue1 = HAL_ADC_GetValue(&hadc1);
	           V_CMOS = rawValue1 * V_stepSize;
	       }
	    HAL_ADC_Stop(&hadc1);
}

void Measurement_of_ADC_Current_CMOS(){
	float V_ref = 3.3;  // This is known for each micro controller from data
		// sheet, V_ref = power supply in
		float ADC_resolution = (4096 - 1);  // 2^12 - 1
		float V_stepSize = V_ref / ADC_resolution;
		// ADC
	    /* Start ADC Conversion for ADC1 */
	    ADC_Select_CurrentCMOS();
	    HAL_ADC_Start(&hadc1);
	    uint16_t rawValue1;
	       if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
	           /* Read the ADC1 value */
	           rawValue1 = HAL_ADC_GetValue(&hadc1);
	           C_CMOS = rawValue1 * V_stepSize;
	       }
	    HAL_ADC_Stop(&hadc1);
}

void Measurement_of_ADC_Current_18650(){
	float V_ref = 3.3;  // This is known for each micro controller from data
	//float temp_voltage;

		// sheet, V_ref = power supply in
		float ADC_resolution = (4096 - 1);  // 2^12 - 1
		float V_stepSize = V_ref / ADC_resolution;
		// ADC
	    /* Start ADC Conversion for ADC1 */
	    ADC_Select_Current18650();
	    HAL_ADC_Start(&hadc1);
	    uint16_t rawValue1;
	       if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
	           /* Read the ADC1 value */
	           rawValue1 = HAL_ADC_GetValue(&hadc1);
	           C_18650 = rawValue1 * V_stepSize;
	           //C_CMOS = (94.3 * temp_voltage) + .008;

	       }
	    HAL_ADC_Stop(&hadc1);
}

void ADC_Select_Voltage18650(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void ADC_Select_VoltageCMOS(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_9;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}

void ADC_Select_Current18650(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void ADC_Select_CurrentCMOS(void){
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_8;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  Error_Handler();
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
