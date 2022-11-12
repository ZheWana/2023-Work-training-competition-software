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
#include "ST7735/Inc/st7735.h"
#include "utils.h"
#include "PID/pid.h"
#include "Compass/QMC5883L.h"
#include "DebugLogger/Debug.h"
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ScreenRefresh */
osThreadId_t ScreenRefreshHandle;
const osThreadAttr_t ScreenRefresh_attributes = {
  .name = "ScreenRefresh",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
  .name = "StateMachine",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialOutput */
osThreadId_t SerialOutputHandle;
const osThreadAttr_t SerialOutput_attributes = {
  .name = "SerialOutput",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MessageQueue */
osMessageQueueId_t MessageQueueHandle;
const osMessageQueueAttr_t MessageQueue_attributes = {
  .name = "MessageQueue"
};
/* Definitions for bGetaPidOutSem */
osSemaphoreId_t bGetaPidOutSemHandle;
const osSemaphoreAttr_t bGetaPidOutSem_attributes = {
  .name = "bGetaPidOutSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void _putchar(char character) {
    HAL_UART_Transmit(&huart5, (uint8_t *) &character, 1, HAL_MAX_DELAY);
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

  /* Create the semaphores(s) */
  /* creation of bGetaPidOutSem */
  bGetaPidOutSemHandle = osSemaphoreNew(1, 1, &bGetaPidOutSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MessageQueue */
  MessageQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &MessageQueue_attributes);

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
void LEDControlEntry(void *argument)
{
  /* USER CODE BEGIN LEDControlEntry */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        osDelay(50);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        osDelay(950);
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
void ScreenRefreshEntry(void *argument)
{
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
void StateMachineEntry(void *argument)
{
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
void AttitudeControlEntry(void *argument)
{
  /* USER CODE BEGIN AttitudeControlEntry */
    /* Infinite loop */
    for (;;) {
        static hmcData_t hmc;
        static int temp = 0;

        if (!temp) {
            temp++;

            int res = QMC5883_Init();
//            DebugConLog(res == 0, "QMC5883_Init:Init OK");
//            DebugConLog(res == -1, "QMC5883_Init:IIC ERROR");
//            DebugConLog(res == -2, "QMC5883_Init:ID Check ERROR");
        }
        int res = QMC5883_GetData(&hmc);
//            DebugConLog(res == 0, "QMC5883_GetData:Init OK");
//            DebugConLog(res == -1, "QMC5883_GetData:IIC ERROR");
        if (res == 0) {
            DataLog("Mx = %f,My = %f", hmc.Mx, hmc.My);
            DataLog("yaw = %f", ToDig(atan2f(hmc.Mx, hmc.My)));
        }


        hmc.Mx += 0;
        hmc.My += 0;


//        if (CarInfo.aPidLock == aPidLocked) {
//            PID_Init(&CarInfo.aPid, 0, 0, 0, 0);
//        } else {
//            // 采集传感器数�??
//            static hmcData_t hmc;
//            QMC5883_GetData(&hmc);
//            hmc.Mx += 0;
//            hmc.My += 0;
//
//            // PID闭环控制
//            CarInfo.aPid.ctr.aim = 0;
//            CarInfo.aPid.ctr.cur = CarInfo.yaw = atan2f(hmc.Mx, hmc.My);
//            osSemaphoreAcquire(bGetaPidOutSemHandle, UINT32_MAX);
//            CarInfo.aPidOut = PID_RealizeForAngle(&CarInfo.aPid);// 注意弧度�??
//            osSemaphoreRelease(bGetaPidOutSemHandle);
//            CarInfo.aPid.ctr.pre = CarInfo.aPid.ctr.cur;
//        }

//        osDelayUntil(CarInfo.aPidPeriod);
        osDelay(100);
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
void StepControlEntry(void *argument)
{
  /* USER CODE BEGIN StepControlEntry */
    /* Infinite loop */
    for (;;) {
        static uint8_t buff[16] = {[0] = 0x55, [15] = 0xAA};

        if (CarInfo.aimX != CarInfo.curX || CarInfo.aimY != CarInfo.curY) {
            float vx = CarInfo.aimX - CarInfo.curX, vy = CarInfo.aimY - CarInfo.curY,
                    freq1, freq2, freq3, freq4;


            Speed2MotorConverter(vx, vy, &freq1, &freq2, &freq3, &freq4);
            // TODO:
            //  Add aPid data to freq
            //  Handle buffer data
            //  Send buffer to driver
        }
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
void SerialOutputEntry(void *argument)
{
  /* USER CODE BEGIN SerialOutputEntry */
    /* Infinite loop */
    for (;;) {
//        size_t msg;
//        osMessageQueueGet(MessageQueueHandle, &msg, 0, 0xffff);
//        printf("%s\n", (char *) msg);
    }
  /* USER CODE END SerialOutputEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

