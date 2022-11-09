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
/* Definitions for GetInfraredData */
osThreadId_t GetInfraredDataHandle;
const osThreadAttr_t GetInfraredData_attributes = {
        .name = "GetInfraredData",
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

void ledControlEnter(void *argument);

void ScreenRefreshEntry(void *argument);

void StateMachineEntry(void *argument);

void GetInfraredDataEntry(void *argument);

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
    LEDcontrolHandle = osThreadNew(ledControlEnter, NULL, &LEDcontrol_attributes);

    /* creation of ScreenRefresh */
    ScreenRefreshHandle = osThreadNew(ScreenRefreshEntry, NULL, &ScreenRefresh_attributes);

    /* creation of StateMachine */
    StateMachineHandle = osThreadNew(StateMachineEntry, NULL, &StateMachine_attributes);

    /* creation of GetInfraredData */
    GetInfraredDataHandle = osThreadNew(GetInfraredDataEntry, NULL, &GetInfraredData_attributes);

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

/* USER CODE BEGIN Header_ledControlEnter */
/**
  * @brief  Function implementing the LEDcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ledControlEnter */
void ledControlEnter(void *argument) {
    /* USER CODE BEGIN ledControlEnter */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        osDelay(50);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        osDelay(950);
    }
    /* USER CODE END ledControlEnter */
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
        RunMainState();
    }
    /* USER CODE END StateMachineEntry */
}

/* USER CODE BEGIN Header_GetInfraredDataEntry */
/**
* @brief Function implementing the GetInfraredData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetInfraredDataEntry */
void GetInfraredDataEntry(void *argument) {
    /* USER CODE BEGIN GetInfraredDataEntry */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END GetInfraredDataEntry */
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

