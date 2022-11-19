/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define LED_System_Pin GPIO_PIN_3
#define LED_System_GPIO_Port GPIOE
#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
#define SER_Pin GPIO_PIN_0
#define SER_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_1
#define CLK_GPIO_Port GPIOC
#define LD_Pin GPIO_PIN_2
#define LD_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LED_Green_Pin GPIO_PIN_11
#define LED_Green_GPIO_Port GPIOD
#define LED_Blue_Pin GPIO_PIN_13
#define LED_Blue_GPIO_Port GPIOD
#define LED_White_Pin GPIO_PIN_15
#define LED_White_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOA
#define KEY0_Pin GPIO_PIN_11
#define KEY0_GPIO_Port GPIOC
#define ST7735_RES_Pin GPIO_PIN_0
#define ST7735_RES_GPIO_Port GPIOD
#define ST7735_DC_Pin GPIO_PIN_1
#define ST7735_DC_GPIO_Port GPIOD
#define ST7735_CS_Pin GPIO_PIN_2
#define ST7735_CS_GPIO_Port GPIOD
#define ST7735_BL_Pin GPIO_PIN_3
#define ST7735_BL_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define Key_PollingPeriod       (10)
#define UC_PollingPeriod        (200)
#define UC_TimerResetPeriod     (10000)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
