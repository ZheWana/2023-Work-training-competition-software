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
#include "float.h"
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
/* Definitions for ScreenRefresh */
osThreadId_t ScreenRefreshHandle;
const osThreadAttr_t ScreenRefresh_attributes = {
        .name = "ScreenRefresh",
        .stack_size = 512 * 4,
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

void ScreenRefreshEntry(void *argument);

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

    /* creation of ScreenRefresh */
    ScreenRefreshHandle = osThreadNew(ScreenRefreshEntry, NULL, &ScreenRefresh_attributes);

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
void LEDcontrolEntry(void *argument) {
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
void SerialOutputEntry(void *argument) {
    /* USER CODE BEGIN SerialOutputEntry */
    /* Infinite loop */
    for (;;) {
//        printf("data:");
//        printf("%d,", CarInfo.spd[0]);
//        printf("%f,", CarInfo.msPid[0].ctr.aim);
//        printf("%f,", CarInfo.psi[0]);
//        printf("%f,", CarInfo.mpPid[0].ctr.aim);
//        printf("%f,", CarInfo.hmc.Mx);
//        printf("%f,", CarInfo.hmc.My);
//        printf("%f,", ToDig(CarInfo.yaw));
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
void StateMachineEntry(void *argument) {
    /* USER CODE BEGIN StateMachineEntry */
    /* Infinite loop */
    for (;;) {
//        MecanumRotate(10000000,10,1);
        osDelay(1);
    }
    /* USER CODE END StateMachineEntry */
}

/* USER CODE BEGIN Header_ScreenRefreshEntry */
/**
* @brief Function implementing the ScreenRefresh thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScreenRefreshEntry */
void ScreenRefreshEntry(void *argument) {
    /* USER CODE BEGIN ScreenRefreshEntry */
    /* Infinite loop */
    for (;;) {
        char buff[64];
        static uint32_t pretick = 0;
        float fps = 1000.0f / (HAL_GetTick() - pretick);

        pretick = HAL_GetTick();
        sprintf(buff, "FPS:%.3f\n", fps);
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);

//        for (int i = 0; i < 16; i++) {
//            buff[i] = CarInfo.infrared & (1 << i) ? '1' : '0';
//        }
//        buff[16] = '\0';
//        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);
//        for (int i = 0; i < 16; i++) {
//            buff[i] = CarInfo.infrared & (1 << (i + 16)) ? '1' : '0';
//        }
//        buff[16] = '\0';
//        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, CarInfo.yaw > 0 ? "Yaw: %.3f\n" : "Yaw:%.3f\n", ToDig(CarInfo.yaw));
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, "dx:%.3f\n", CarInfo.dx);
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, "dy:%.3f\n", CarInfo.dy);
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, "X:%.3f\n", CarInfo.curX);
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, "Y:%.3f\n", CarInfo.curY);
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);

        // End of page
        LCD_StringLayout(LCD_EOP);
    }
    /* USER CODE END ScreenRefreshEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

