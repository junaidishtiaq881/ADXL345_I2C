/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "stdio.h"
#include "stm32l4xx_hal.h"

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t x,y,z;
uint8_t data_rec[6];

uint8_t read_register(uint8_t reg) {
	uint8_t value;
	    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
	    return value;
}

void adxl_write (uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);

}

void adxl_read_values (uint8_t reg)
{
	HAL_I2C_Mem_Read (&hi2c1, ADXL345_ADDR, reg, 1, (uint8_t *)data_rec, 6, 100);
}

void adxl_init (void)
{
	read_register (ADXL345_REG_ADDRESS); // read the DEVID

	adxl_write (ADXL345_REG_DATA_FORMAT, 0x01);  // data_format range= +- 4g --> for that setting  0 1
	adxl_write (ADXL345_POWER_CTL, 0x00);   // resetting all bits of PWR_CTRL register
	adxl_write (ADXL345_POWER_CTL, 0x08);   // PWR_CTRL measure and wake up at 8hz frequency(8 samples per second), setting 3rd bit of power_ctrl reg
	adxl_write (ADXL345_THRESH_TAP, 0x30);  // setting a tap threshold to ~3g
	adxl_write (ADXL345_TAP_DURATION, 0x10); // adjusting a tap duration to ~10 ms
	adxl_write (ADXL345_TAP_LATENCY, 0x20);  // setting tap latency to ~20 ms
	adxl_write (ADXL345_TAP_WINDOW, 0x50);   // setting tap window to 0x80 = ~200ms
	adxl_write (ADXL345_TAP_AXES, 0x07);    // enabling tap detection on all axes
	adxl_write (ADXL345_INT_ENABLE, 0x20);   // enabling single tap interrupt
	adxl_write (ADXL345_INT_MAP, 0x00);

}
/*uint8_t  adxl_readtapstatus(void){
	// Read the INT_SOURCE register
	    uint8_t int_source = read_register(ADXL345_REG_INT_SOURCE);
	    // Check if single-tap bit is set
	        if (int_source & 0x40) {
	            // Single-tap detected
	        	 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2); // Toggle LED
	        }
}*/
void DetectDoubleTap(void) {
    uint8_t intSource = read_register(ADXL345_REG_INT_SOURCE); // Read the interrupt source register

        // Check if double-tap interrupt is set
        if (intSource & 0x20) {
        	 HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1); // Toggle red LED
        }
}


int16_t adxl_readx(void)
{
	int16_t x;
	adxl_read_values (ADXL345_REG_DATAX0);
	x = ((data_rec[1]<<8)|data_rec[0]);
	return x;
}

int16_t adxl_ready(void)
{
	int16_t y;
	adxl_read_values (ADXL345_REG_DATAY0);
	y = ((data_rec[3]<<8)|data_rec[2]);
	return y;
}

int16_t adxl_readz(void)
{
	int16_t z;
	adxl_read_values (ADXL345_REG_DATAZ0);
	z = ((data_rec[5]<<8)|data_rec[4]);
	return z;
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  adxl_init();
 // uint8_t device_id = read_register_dev_id(ADXL345_REG_ADDRESS);
 // printf("ADXL345 Device ID: 0x%X\n", device_id);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 x=adxl_readx();
	 y=adxl_ready();
	 z=adxl_readz();

	 // Continuously check for single-tap

	 //adxl_readtapstatus();

	 // Continuously check for double-tap
	 DetectDoubleTap();

	 // Add a small delay to reduce polling frequency
	 HAL_Delay(10);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0010061A;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDRed_GPIO_Port, LEDRed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDGreen_GPIO_Port, LEDGreen_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDRed_Pin */
  GPIO_InitStruct.Pin = LEDRed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDRed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDGreen_Pin */
  GPIO_InitStruct.Pin = LEDGreen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDGreen_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
