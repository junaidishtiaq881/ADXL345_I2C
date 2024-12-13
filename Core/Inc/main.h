/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define ADXL345_ADDR (0x53 << 1)
// ADXL345 Register Addresses

#define ADXL345_REG_ADDRESS          0x00 // Device ID
#define ADXL345_THRESH_TAP           0x1D // Tap Threshold
#define ADXL345_TAP_DURATION         0x21 // Tap Duration
#define ADXL345_TAP_LATENCY          0x22 // Tap Latency
#define ADXL345_TAP_WINDOW           0x23 // Tap Window
#define ADXL345_TAP_AXES             0x2A // Axis Control for Single/Double Tap
#define ADXL345_ACT_TAP_STATUS       0x2B // Source of Tap/Double Tap
#define ADXL345_DATARATEPWRMODECTRL  0x2C // Data Rate and power mode control
#define ADXL345_POWER_CTL            0x2D // Power Save features control
#define ADXL345_INT_ENABLE           0x2E // Interrupt enable control
#define ADXL345_INT_MAP              0x2F // Interrupt mapping control
#define ADXL345_REG_INT_SOURCE       0x30 // Source of interrupts
#define ADXL345_REG_DATA_FORMAT      0x31 // Data Format Control
#define ADXL345_REG_DATAX0           0x32  //X axis data
#define ADXL345_REG_DATAX1           0x33
#define ADXL345_REG_DATAY0           0x34  //  Y axis data
#define ADXL345_REG_DATAY1           0x35
#define ADXL345_REG_DATAZ0           0x36  //  Z axis data
#define ADXL345_REG_DATAZ1           0x37


//Set i2c Handeler here
extern I2C_HandleTypeDef hi2c1;


// ADXL345 Function Prototypes

void adxl_write (uint8_t reg, uint8_t value);
void adxl_read_values (uint8_t reg);
void adxl_read_address (uint8_t reg);
void adxl_init (void);
int16_t adxl_readx(void);
int16_t adxl_ready(void);
int16_t adxl_readz(void);
uint8_t  adxl_readtapstatus(void);
uint8_t read_register(uint8_t reg);
void DetectDoubleTap(void);

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
#define LEDRed_Pin GPIO_PIN_1
#define LEDRed_GPIO_Port GPIOC
#define LEDGreen_Pin GPIO_PIN_2
#define LEDGreen_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
