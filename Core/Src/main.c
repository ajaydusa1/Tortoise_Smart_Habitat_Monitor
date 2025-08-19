/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ds3231_for_stm32_hal.h"
#include "lcd16x2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



// Below variables are for RTC DS3231
uint8_t hr,min,sec;
char clock[16];

// Below variables are for AHT20
#define addr (0x38<<1)    //This address writes to Slave Devices, we get 0x70
#define status1 0x71

char buffer[45];
uint8_t buff_stat[1];
uint8_t aht_init[3]={0xBE,0x08,0x00};
uint8_t trig_measurement[3]={0xAC,0x33,0x00};
uint8_t data[6];
char temp_str[16];
char humi_str[16];

uint32_t t;
uint32_t h;

float Temp;
uint8_t Humidity;

//For Relay Timings

#define UV_ON_HOUR     8     // 08:00 AM
#define UV_OFF_HOUR    21    // 05:00 PM



// Ambient Light (LED Strip) ON/OFF and Fade Times
#define AMBIENT_LIGHT_FADE_IN_START_HOUR   9
#define AMBIENT_LIGHT_FADE_IN_START_MIN    30

#define AMBIENT_LIGHT_FADE_IN_END_HOUR     9
#define AMBIENT_LIGHT_FADE_IN_END_MIN      45

#define AMBIENT_LIGHT_FADE_OUT_START_HOUR  20
#define AMBIENT_LIGHT_FADE_OUT_START_MIN   50

#define AMBIENT_LIGHT_FADE_OUT_END_HOUR    20
#define AMBIENT_LIGHT_FADE_OUT_END_MIN     55

#define PWM_MAX_DUTY_CYCLE                 100						//Alter this no. to control the Duty Cycle/Brightness of the LED. 199 corresponds to 100% Duty Cycle. Currently set to 50%




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Temperature_();
void Humidity_();



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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  lcd16x2_init_4bits(RS_GPIO_Port,RS_Pin,EN_Pin,
    		D4_GPIO_Port,D4_Pin,D5_Pin,D6_Pin,D7_Pin);
  lcd16x2_cursorShow(0);






  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);			//This value:199 means 100% duty cycle. 199 value is set in the ARR.You must match the compare value to the ARR value to get 100percent duty cycle


  /* DS3231 RTC SENSOR  */
  // DS3231_SetFullDate(day, month, dow, year)
  // 'dow' = Day of the week (1=Sunday, ..., 7=Saturday)
    DS3231_Init(&hi2c2);
  //DS3231_SetFullTime(19, 49, 00);
  //DS3231_SetFullDate(01, 8, 1, 2025);    //


  /* AHT20 TEMP & HUM  */
  HAL_Delay(40);
  buff_stat[0] = status1;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buff_stat,1, HAL_MAX_DELAY);
  HAL_Delay(5);
  HAL_I2C_Master_Receive(&hi2c2, addr, &buff_stat[0],1, HAL_MAX_DELAY);
  HAL_Delay(5);

  if((buff_stat[0] & (1<<3)) == 0x00){ 		//bit 3 contains 0 instead of 1 then we need to initialize it/Calibrate it

	  	  HAL_I2C_Master_Transmit(&hi2c2, addr, aht_init,3, HAL_MAX_DELAY);
	  	  HAL_Delay(12);
	  	  HAL_I2C_Master_Transmit(&hi2c2, addr, trig_measurement,3, HAL_MAX_DELAY);
	  	  HAL_Delay(80);

	  	 // sprintf(buffer,"%x\r\n",temp_crc_read[0]);
	      sprintf(buffer,"It is Not Calibrated, need to init N trig");
	 	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sizeof(buffer), 100);
	 	  HAL_Delay(1000);


  }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // ---- Getting RTC Values from the RTC Module ----
	  hr = DS3231_GetHour();
	  min = DS3231_GetMinute();
	  sec = DS3231_GetSecond();

	  // ---- Ambient Light PWM Control ----
	  uint16_t current_minutes = hr * 60 + min;
	  uint16_t fade_in_start  = AMBIENT_LIGHT_FADE_IN_START_HOUR * 60 + AMBIENT_LIGHT_FADE_IN_START_MIN;
	  uint16_t fade_in_end    = AMBIENT_LIGHT_FADE_IN_END_HOUR * 60 + AMBIENT_LIGHT_FADE_IN_END_MIN;
	  uint16_t fade_out_start = AMBIENT_LIGHT_FADE_OUT_START_HOUR * 60 + AMBIENT_LIGHT_FADE_OUT_START_MIN;
	  uint16_t fade_out_end   = AMBIENT_LIGHT_FADE_OUT_END_HOUR * 60 + AMBIENT_LIGHT_FADE_OUT_END_MIN;

	  uint16_t brightness = 0;

	  if (current_minutes >= fade_in_start && current_minutes < fade_in_end) {
	      float ratio = (float)(current_minutes - fade_in_start) / (fade_in_end - fade_in_start);
	      brightness = (uint16_t)(ratio * PWM_MAX_DUTY_CYCLE);
	  }
	  else if (current_minutes >= fade_in_end && current_minutes < fade_out_start) {
	      brightness = PWM_MAX_DUTY_CYCLE;
	  }
	  else if (current_minutes >= fade_out_start && current_minutes < fade_out_end) {
	      float ratio = (float)(fade_out_end - current_minutes) / (fade_out_end - fade_out_start);
	      brightness = (uint16_t)(ratio * PWM_MAX_DUTY_CYCLE);
	  }
	  else {
	      brightness = 0;
	  }

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, brightness);



	  // ---- Getting Temp and Humidity Data from the Sensor Module
	  HAL_I2C_Master_Transmit(&hi2c2, addr, trig_measurement,3, HAL_MAX_DELAY);
	  HAL_Delay(80);
	  HAL_I2C_Master_Receive(&hi2c2, addr,data,sizeof(data), HAL_MAX_DELAY);
      Humidity_();
      Temperature_();

      sprintf(temp_str, "%.1f", Temp);
      sprintf(humi_str, "%u", Humidity);
      sprintf(clock,"%02u:%02u:%02u", hr, min,sec);


	  if (hr >= UV_ON_HOUR && hr < UV_OFF_HOUR) {
	      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);  // Turn ON UV Lamp
	  } else {
	      HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET); // Turn OFF UV Lamp
	  }



	  // Screen 1: Temp & Humidity (10s static)
	      // -------------------
	      lcd16x2_clear();
	      lcd16x2_printf("T:%s%cC", temp_str, 223);
	      lcd16x2_2ndLine();
	      lcd16x2_printf("H:%s%c", humi_str, 37);
	      HAL_Delay(10000);  // wait for 10 seconds

	      // -------------------
	      // Screen 2: Time & UV status (3s static)
	      // -------------------
	      lcd16x2_clear();
	      lcd16x2_printf("%s", clock);
	      lcd16x2_2ndLine();
	      lcd16x2_printf("UV:%s", (hr >= UV_ON_HOUR && hr < UV_OFF_HOUR) ? "ON " : "OFF");
	      HAL_Delay(3000);   // wait for 3 seconds

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin EN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Humidity_(void){

	  h = data[1];
	  h<<=8;
	  h|=data[2];
	  h<<=4;
	  h|=data[3]>>4;
	  Humidity = ((float)h*100) / 0x100000;
}

void Temperature_(void){

	 t = data[3] & 0x0F;
     t <<=8;
     t|=data[4];
	 t<<=8;
	 t|=data[5];
	 Temp = ((float)t*200/0x100000)-50;

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
#ifdef USE_FULL_ASSERT
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
