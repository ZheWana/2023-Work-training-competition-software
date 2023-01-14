/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "move.h"
#include "stdio.h"
#include "usart.h"
#include "utils.h"
#include "stdlib.h"
#include "printf.h"
#include "PID/pid.h"
#include "74HC165/74HC165.h"
#include "Compass/QMC5883L.h"
#include "CommonKey/comKey.h"
#include "ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LEDcontrol */
osThreadId_t LEDcontrolHandle;
const osThreadAttr_t LEDcontrol_attributes = {
  .name = "LEDcontrol",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialOutput */
osThreadId_t SerialOutputHandle;
const osThreadAttr_t SerialOutput_attributes = {
  .name = "SerialOutput",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
  .name = "StateMachine",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for bQueuePut */
osSemaphoreId_t bQueuePutHandle;
const osSemaphoreAttr_t bQueuePut_attributes = {
  .name = "bQueuePut"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LEDcontrolEntry(void *argument);
void SerialOutputEntry(void *argument);
void StateMachineEntry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of bQueuePut */
  bQueuePutHandle = osSemaphoreNew(1, 1, &bQueuePut_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDcontrol */
  LEDcontrolHandle = osThreadNew(LEDcontrolEntry, NULL, &LEDcontrol_attributes);

  /* creation of SerialOutput */
  SerialOutputHandle = osThreadNew(SerialOutputEntry, NULL, &SerialOutput_attributes);

  /* creation of StateMachine */
  StateMachineHandle = osThreadNew(StateMachineEntry, NULL, &StateMachine_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LEDcontrolEntry */
/**
  * @brief  Function implementing the LEDcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDcontrolEntry */
void LEDcontrolEntry(void *argument)
{
  /* USER CODE BEGIN LEDcontrolEntry */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, GPIO_PIN_SET);
        osDelay(50);
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, GPIO_PIN_RESET);
        osDelay(950);
    }
  /* USER CODE END LEDcontrolEntry */
}

/* USER CODE BEGIN Header_SerialOutputEntry */
/**
* @brief Function implementing the SerialOutput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialOutputEntry */
void SerialOutputEntry(void *argument)
{
  /* USER CODE BEGIN SerialOutputEntry */
    /* Infinite loop */
    for (;;) {
//        printf("data:");
//        printf("%d,", CarInfo.spd[0]);
//        printf("%f,", CarInfo.mPid[0].ctr.aim);
//        printf("%f,", CarInfo.psi[0]);
//        printf("%f,", CarInfo.pPid[0].ctr.aim);
////        printf("%f", log2(CarInfo.isCarMoving == 0 ? 1 : CarInfo.isCarMoving));
//        printf("\n");
    }
  /* USER CODE END SerialOutputEntry */
}

/* USER CODE BEGIN Header_StateMachineEntry */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateMachineEntry */
void StateMachineEntry(void *argument)
{
  /* USER CODE BEGIN StateMachineEntry */
    /* Infinite loop */
    for (;;) {
        ST7735_FillScreen(ST7735_WHITE);
        ST7735_FillScreen(ST7735_RED);
        ST7735_FillScreen(ST7735_GREEN);
        ST7735_FillScreen(ST7735_BLUE);
    }
  /* USER CODE END StateMachineEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

