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
#include "printf.h"
#include "usart.h"
#include "st7735.h"
#include "utils.h"
#include "pid.h"
#include "Compass/HMC5883L.h"
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
extern CCB_Typedef CarInfo;
/* USER CODE END Variables */
/* Definitions for LEDcontrol */
osThreadId_t LEDcontrolHandle;
const osThreadAttr_t LEDcontrol_attributes = {
        .name = "LEDcontrol",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ScreenRefresh */
osThreadId_t ScreenRefreshHandle;
const osThreadAttr_t ScreenRefresh_attributes = {
        .name = "ScreenRefresh",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
        .name = "StateMachine",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AttitudeControl */
osThreadId_t AttitudeControlHandle;
const osThreadAttr_t AttitudeControl_attributes = {
        .name = "AttitudeControl",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StepControl */
osThreadId_t StepControlHandle;
const osThreadAttr_t StepControl_attributes = {
        .name = "StepControl",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SerialOutput */
osThreadId_t SerialOutputHandle;
const osThreadAttr_t SerialOutput_attributes = {
        .name = "SerialOutput",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MessageQueue */
osMessageQueueId_t MessageQueueHandle;
const osMessageQueueAttr_t MessageQueue_attributes = {
        .name = "MessageQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void _putchar(char character) {
    HAL_UART_Transmit(&huart4, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}

/* USER CODE END FunctionPrototypes */

void LEDControlEntry(void *argument);

void ScreenRefreshEntry(void *argument);

void StateMachineEntry(void *argument);

void AttitudeControlEntry(void *argument);

void StepControlEntry(void *argument);

void SerialOutputEntry(void *argument);

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

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of MessageQueue */
    MessageQueueHandle = osMessageQueueNew(16, sizeof(uint32_t), &MessageQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of LEDcontrol */
    LEDcontrolHandle = osThreadNew(LEDControlEntry, NULL, &LEDcontrol_attributes);

    /* creation of ScreenRefresh */
    ScreenRefreshHandle = osThreadNew(ScreenRefreshEntry, NULL, &ScreenRefresh_attributes);

    /* creation of StateMachine */
    StateMachineHandle = osThreadNew(StateMachineEntry, NULL, &StateMachine_attributes);

    /* creation of AttitudeControl */
    AttitudeControlHandle = osThreadNew(AttitudeControlEntry, NULL, &AttitudeControl_attributes);

    /* creation of StepControl */
    StepControlHandle = osThreadNew(StepControlEntry, NULL, &StepControl_attributes);

    /* creation of SerialOutput */
    SerialOutputHandle = osThreadNew(SerialOutputEntry, NULL, &SerialOutput_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LEDControlEntry */
/**
  * @brief  Function implementing the LEDcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDControlEntry */
void LEDControlEntry(void *argument) {
    /* USER CODE BEGIN LEDControlEntry */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END LEDControlEntry */
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
        osDelay(1);
    }
    /* USER CODE END ScreenRefreshEntry */
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
        CarInfo.RunMainState();
    }
    /* USER CODE END StateMachineEntry */
}

/* USER CODE BEGIN Header_AttitudeControlEntry */
/**
* @brief Function implementing the AttitudeControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AttitudeControlEntry */
void AttitudeControlEntry(void *argument) {
    /* USER CODE BEGIN AttitudeControlEntry */
    /* Infinite loop */
    for (;;) {
        if (CarInfo.aPidLock == aPidLocked) {
            PID_Init(&CarInfo.aPid, 0, 0, 0, 0);
        } else {
            // 采集传感器数据
            static hmcData_t hmc;
            HMC5883_GetData(&hmc);
            hmc.Mx += 0;
            hmc.My += 0;

            // PID闭环控制
            CarInfo.aPid.ctr.aim = 0;
            CarInfo.aPid.ctr.cur = atan2f(hmc.Mx, hmc.My);
            CarInfo.aPidOut = PID_RealizeForAngle(&CarInfo.aPid);// 注意弧度制
            CarInfo.aPid.ctr.pre = CarInfo.aPid.ctr.cur;
        }

        osDelayUntil(CarInfo.aPidPeriod);
    }
    /* USER CODE END AttitudeControlEntry */
}

/* USER CODE BEGIN Header_StepControlEntry */
/**
* @brief Function implementing the StepControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StepControlEntry */
void StepControlEntry(void *argument) {
    /* USER CODE BEGIN StepControlEntry */
    /* Infinite loop */
    for (;;) {
        char *string = "message";
        size_t buff = (size_t) string;
        osMessageQueuePut(MessageQueueHandle, &buff, 0, 0xffff);
        osDelay(200);
    }
    /* USER CODE END StepControlEntry */
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
        size_t msg;
        osMessageQueueGet(MessageQueueHandle, &msg, 0, 0xffff);
        printf("%s\n", (char *) msg);
        osDelay(100);
    }
    /* USER CODE END SerialOutputEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

