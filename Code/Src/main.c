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
#include <stdio.h>
#include <string.h>
#include "fonts.h"
#include "bme280.h"
#include "ssd1306.h"

/*
float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];




ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);




int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}
*/

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  //MX_ADC1_Init();


  // BME280 init
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  // BME280 settings
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

  // Initialize OLED
  SSD1306_Init();

  //SSD1306_GotoXY (0, 0);
  //SSD1306_Puts (hum_string, &Font_11x18, 1);
  //SSD1306_Puts ("STM32", &Font_11x18, 1);
  //SSD1306_UpdateScreen();
  //HAL_Delay (2000);
  SSD1306_Clear();



  while (1)
  {

	if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0))
	{
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}
	 else
	 {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 }

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);


    // Forced mode setting, switched to SLEEP mode after measurement
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    dev.delay_ms(40);
    //Get Data
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    if(rslt == BME280_OK)
    {
      temperature = comp_data.temperature / 100.0;
      humidity = comp_data.humidity / 1024.0;
      pressure = comp_data.pressure / 10000.0;

      //Display Data
      //memset(hum_string, 0, sizeof(hum_string));
      //memset(temp_string, 0, sizeof(temp_string));
      //memset(press_string, 0, sizeof(press_string));

      //sprintf(hum_string, "Humidity %03.1f %% ", humidity);
      //sprintf(temp_string, "Temp: %03.1f C ", temperature);
      //sprintf(press_string, "Pressure %03.1f hPa ", pressure);

      //SSD1306_GotoXY (0, 0);
      //SSD1306_Puts (hum_string, &Font_7x10, 1);
      SSD1306_GotoXY (0, 0);
      SSD1306_Puts ("STM32F411CEU6 Dev", &Font_7x10, 1);

      SSD1306_GotoXY (0, 30);
      SSD1306_Puts (" Temp     Humidity", &Font_7x10, 1);
      //SSD1306_UpdateScreen();


      //Temp int to float
      char str1[100];
      int d1 = temperature;          //Get the integer part
      float f2 = temperature - d1;   // Get fractional part (678.0123 - 678 = 0.0123)
      //int d2 = trunc(f2 * 10);       // Turn into integer (123)
      int d2 = (f2 * 10);       // Turn into integer (123)
      sprintf (str1, "%d.%d", d1, d2);
      SSD1306_GotoXY (0, 45);
      SSD1306_Puts (str1, &Font_11x18, 1);

      //Hum int to float
      char str2[100];
      int d3 = humidity;          //Get the integer part
      float f3 = humidity - d3;   // Get fractional part (678.0123 - 678 = 0.0123)
      int d4 = (f3 * 10);       // Turn into integer (123)
      sprintf (str2, "%d.%d", d3, d4);
      SSD1306_GotoXY (75, 45);
      SSD1306_Puts (str2, &Font_11x18, 1);

      //SSD1306_GotoXY (20, 25);
      //const char DegreeSymbol = 247;
      //SSD1306_Puts (0xB0, &Font_11x18, 1);


      //SSD1306_GotoXY (70, 0);
      //SSD1306_Puts (temp_string, &Font_11x18, 1);




      /*Working
      itoa(temperature, temp_string, 10);
      SSD1306_GotoXY (0, 0);
      SSD1306_Puts ("Temp:", &Font_11x18, 1);
      SSD1306_GotoXY (70, 0);
      SSD1306_Puts (temp_string, &Font_11x18, 1);
      */

      /*
      itoa(humidity, hum_string, 10);
      SSD1306_GotoXY (0, 20);
      SSD1306_Puts ("Hum:", &Font_11x18, 1);
      SSD1306_GotoXY (70, 20);
      SSD1306_Puts (hum_string, &Font_11x18, 1);

      itoa(pressure, press_string, 10);
      SSD1306_GotoXY (0, 40);
      SSD1306_Puts ("Pres:", &Font_11x18, 1);
      SSD1306_GotoXY (70, 40);
      SSD1306_Puts (press_string, &Font_11x18, 1);
      */

      SSD1306_UpdateScreen();
    }

    HAL_Delay(10);
  }

}

/*
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  //MX_ADC1_Init();

  // BME280 init
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  // BME280 settings
   dev.settings.osr_h = BME280_OVERSAMPLING_1X;
   dev.settings.osr_p = BME280_OVERSAMPLING_16X;
   dev.settings.osr_t = BME280_OVERSAMPLING_2X;
   dev.settings.filter = BME280_FILTER_COEFF_16;
   rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);



  SSD1306_Init();


  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("STM32", &Font_11x18, 1);
  SSD1306_GotoXY (0, 30);
  SSD1306_Puts ("Dev Board", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay (2000);
  SSD1306_Clear();



  // USER CODE END 2

  // Infinite loop
  // USER CODE BEGIN WHILE
  while (1)
  {
	  //SSD1306_GotoXY (0, 0);
	  //SSD1306_Puts ("...", &Font_11x18, 1);
	  //SSD1306_UpdateScreen();
	  //HAL_Delay (5000);


	    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	  	dev.delay_ms(40);

	  	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
	  	if(rslt == BME280_OK)
	  	{
	  	 temperature = comp_data.temperature / 100.0;
	  	 humidity = comp_data.humidity / 1024.0;
	  	 pressure = comp_data.pressure / 10000.0;

	  	 memset(hum_string, 0, sizeof(hum_string));
	  	 memset(temp_string, 0, sizeof(temp_string));
	  	 memset(press_string, 0, sizeof(press_string));

	     sprintf(hum_string, "H: %03.1f %% ", humidity);
	  	 sprintf(temp_string, "T: %03.1f C ", temperature);
	  	 sprintf(press_string, "P: %03.1f hPa ", pressure);

	  	 SSD1306_GotoXY (0, 0);
	  	 SSD1306_Puts (temp_string, &Font_11x18, 1);
	  	 //SSD1306_Puts ("Good", &Font_11x18, 1);
	  	 SSD1306_UpdateScreen();
	  	 HAL_Delay (1000);

	  	 //sprintf(hum_string,  "%03.1f %% ", humidity);
	  	 //sprintf(temp_string, "%03.1f C ", temperature);

	  	 //ssd1306_set_cursor(30,10);
	  	 //ssd1306_write_string(font11x18, temp_string);

	  	 //ssd1306_set_cursor(30,35);
	     //ssd1306_write_string(font11x18, hum_string);

	  	 //ssd1306_update_screen();
	  	 //dev.delay_ms(1000);
	  	}
	  	HAL_Delay (1000);
	}

}

*/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
