/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "configuration.h"
#include <math.h>
#include "algorithms.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum working_mode_e
{
	MODE_NORMAL = 0,
	MODE_SIT_TEST,
	MODE_SUT_TEST,
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void serial_println(char* str, UART_HandleTypeDef *huart_disp);
void main_deploy();
void apoge_deploy();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TTL_HNDLR huart1
#define RS232_HNDLR huart3
#define SD_SPI_HNDLR hspi1
#define FLASH_SPI_HNDLR hspi3
#define TELEM_UART_HNDLR huart4
#define GPS_UART_HNDLR huart6
#define IMU_I2C_HNDLR hi2c3
#define BAR_I2C_HNDLR hi2c1
#define GYRO_IRQ EXTI9_5_IRQn
#define ACC_IRQ EXTI9_5_IRQn
#define APOGE_STAT_Pin GPIO_PIN_13
#define APOGE_STAT_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOC
#define ADC_V_Pin GPIO_PIN_1
#define ADC_V_GPIO_Port GPIOC
#define ADC_C_Pin GPIO_PIN_2
#define ADC_C_GPIO_Port GPIOC
#define APOGE_STAT_DIS_Pin GPIO_PIN_3
#define APOGE_STAT_DIS_GPIO_Port GPIOC
#define APOGEE_LED_Pin GPIO_PIN_0
#define APOGEE_LED_GPIO_Port GPIOA
#define SENS_RES_Pin GPIO_PIN_1
#define SENS_RES_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_2
#define USER_LED_GPIO_Port GPIOA
#define MAIN_LED_Pin GPIO_PIN_3
#define MAIN_LED_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define SD_CD_Pin GPIO_PIN_4
#define SD_CD_GPIO_Port GPIOC
#define MAIN_MOS_Pin GPIO_PIN_0
#define MAIN_MOS_GPIO_Port GPIOB
#define MAIN_STAT_Pin GPIO_PIN_1
#define MAIN_STAT_GPIO_Port GPIOB
#define MAIN_STAT_DIS_Pin GPIO_PIN_2
#define MAIN_STAT_DIS_GPIO_Port GPIOB
#define GPS_LED_Pin GPIO_PIN_12
#define GPS_LED_GPIO_Port GPIOB
#define IO_1_Pin GPIO_PIN_13
#define IO_1_GPIO_Port GPIOB
#define IO_0_Pin GPIO_PIN_14
#define IO_0_GPIO_Port GPIOB
#define FLASH_WP_Pin GPIO_PIN_15
#define FLASH_WP_GPIO_Port GPIOB
#define INT_GYRO_Pin GPIO_PIN_8
#define INT_GYRO_GPIO_Port GPIOC
#define INT_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define INT_ACC_Pin GPIO_PIN_9
#define INT_ACC_GPIO_Port GPIOA
#define INT_ACC_EXTI_IRQn EXTI9_5_IRQn
#define RF_AUX_Pin GPIO_PIN_10
#define RF_AUX_GPIO_Port GPIOA
#define RF_M0_Pin GPIO_PIN_11
#define RF_M0_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_12
#define RF_M1_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA
#define APOGE_MOS_Pin GPIO_PIN_12
#define APOGE_MOS_GPIO_Port GPIOC
#define FLASH_RES_Pin GPIO_PIN_2
#define FLASH_RES_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_LEN 36
#define GPS_BUFFER_LEN	1500
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
