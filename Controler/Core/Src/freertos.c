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
#include "utils.h"
#include "printf.h"
#include "74HC165/74HC165.h"
#include "Compass/QMC5883L.h"
#include "ST7735-HAL/st7735.h"
#include "LobotSerialServo/LobotSerialServo.h"
#include "Filter/filter.h"
#include "freertos_os2.h"
#include "move.h"
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
/* Definitions for IOcontrol */
osThreadId_t IOcontrolHandle;
const osThreadAttr_t IOcontrol_attributes = {
        .name = "IOcontrol",
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
/* Definitions for SensorHandle */
osThreadId_t SensorHandleHandle;
const osThreadAttr_t SensorHandle_attributes = {
        .name = "SensorHandle",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InfCalOpticalTa */
osThreadId_t InfCalOpticalTaHandle;
const osThreadAttr_t InfCalOpticalTa_attributes = {
        .name = "InfCalOpticalTa",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorMessageQueue */
osMessageQueueId_t SensorMessageQueueHandle;
const osMessageQueueAttr_t SensorMessageQueue_attributes = {
        .name = "SensorMessageQueue"
};
/* Definitions for KeyTimer */
osTimerId_t KeyTimerHandle;
const osTimerAttr_t KeyTimer_attributes = {
        .name = "KeyTimer"
};
/* Definitions for bQueuePut */
osSemaphoreId_t bQueuePutHandle;
const osSemaphoreAttr_t bQueuePut_attributes = {
        .name = "bQueuePut"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void u8ZeroFiller(uint8_t *data);

/* USER CODE END FunctionPrototypes */

void IOcontrolEntry(void *argument);

void SerialOutputEntry(void *argument);

void StateMachineEntry(void *argument);

void ScreenRefreshEntry(void *argument);

void SensorHandleEntry(void *argument);

void InfCalOpticalEntry(void *argument);

void KeyTimerCallback(void *argument);

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

    /* Create the timer(s) */
    /* creation of KeyTimer */
    KeyTimerHandle = osTimerNew(KeyTimerCallback, osTimerPeriodic, NULL, &KeyTimer_attributes);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of SensorMessageQueue */
    SensorMessageQueueHandle = osMessageQueueNew(16, sizeof(uint16_t), &SensorMessageQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of IOcontrol */
    IOcontrolHandle = osThreadNew(IOcontrolEntry, NULL, &IOcontrol_attributes);

    /* creation of SerialOutput */
    SerialOutputHandle = osThreadNew(SerialOutputEntry, NULL, &SerialOutput_attributes);

    /* creation of StateMachine */
    StateMachineHandle = osThreadNew(StateMachineEntry, NULL, &StateMachine_attributes);

    /* creation of ScreenRefresh */
    ScreenRefreshHandle = osThreadNew(ScreenRefreshEntry, NULL, &ScreenRefresh_attributes);

    /* creation of SensorHandle */
    SensorHandleHandle = osThreadNew(SensorHandleEntry, NULL, &SensorHandle_attributes);

    /* creation of InfCalOpticalTa */
    InfCalOpticalTaHandle = osThreadNew(InfCalOpticalEntry, NULL, &InfCalOpticalTa_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_IOcontrolEntry */
/**
  * @brief  Function implementing the IOcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IOcontrolEntry */
void IOcontrolEntry(void *argument) {
    /* USER CODE BEGIN IOcontrolEntry */
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, 1);
        osDelay(50);
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, 0);
        osDelay(950);
        if (CarInfo.SerialOutputEnable)osThreadResume(SerialOutputHandle);
    }
    /* USER CODE END IOcontrolEntry */
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
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        if (!CarInfo.SerialOutputEnable)osThreadSuspend(SerialOutputHandle);

        printf("data:");
//
//        printf("%d,", CarInfo.spd[0]);
//        printf("%d,", CarInfo.spd[1]);
//        printf("%d,", CarInfo.spd[2]);
//        printf("%d,", CarInfo.spd[3]);
//        printf("%f,", CarInfo.msPid[0].ctr.aim);
//        printf("%f,", CarInfo.psi[0]);
//        printf("%f,", CarInfo.mpPid[0].ctr.aim);
//
        printf("%f,", CarInfo.yaw);
        printf("%f,", atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My));
//        printf("%f,", CarInfo.avPid.ctr.aim);

//        printf("%f,", CarInfo.curX);
//        printf("%f,", CarInfo.cpPidX.ctr.aim);
//        printf("%f,", CarInfo.curY);
//        printf("%f,", CarInfo.cpPidY.ctr.aim);

        printf("%.3f,", CarInfo.hmc.Mx);
        printf("%.3f,", CarInfo.hmc.My);

//        printf("%.3f,", CarInfo.icm.gx);
//        printf("%.3f,", CarInfo.icm.gy);
        printf("%.3f,", CarInfo.icm.gz);

        printf("%.3f,", CarInfo.avPid.error.sum);

//        printf("\r\n");
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
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        CarInfo.RunMainState();
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
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        char buff[64];
        static uint32_t pretick = 0;
        float fps = 1000.0f / (HAL_GetTick() - pretick);
        pretick = HAL_GetTick();
        sprintf(buff, "FPS:%.3f\n", fps);
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);

        sprintf(buff, CarInfo.yaw > 0 ? "Yaw: %.3f\n" : "Yaw:%.3f\n", ToDig(CarInfo.yaw));
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);

        sprintf(buff, "X:%.3f\n", CarInfo.curX);
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);
        sprintf(buff, "Y:%.3f\n", CarInfo.curY);
        LCD_StringLayout(128, buff, Font_11x18, ST7735_BLACK, ST7735_WHITE);

        // End of page
        LCD_StringLayout(LCD_EOP);
    }
    /* USER CODE END ScreenRefreshEntry */
}

/* USER CODE BEGIN Header_SensorHandleEntry */
/**
* @brief Function implementing the SensorHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorHandleEntry */
void SensorHandleEntry(void *argument) {
    /* USER CODE BEGIN SensorHandleEntry */
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        static enum SensorType {
            sCompass,
            sGyro,
            sOptical,
        } SensorType;
        static MovingFilter_t gyroFilter = {0};
        static float yawPre = 0;

        osMessageQueueGet(SensorMessageQueueHandle, &SensorType, 0, osWaitForever);

        switch (SensorType) {
            case sCompass:
                taskENTER_CRITICAL();
                QMC5883_GetData(&CarInfo.hmc);
                VecRotate(CarInfo.hmc.Mx, CarInfo.hmc.My, CarInfo.initYawOffset);
                CarInfo.yaw = Filter_Smoothing(
                        CarInfo.gyroConfi * (CarInfo.yaw + ToRad(CarInfo.icm.gz) * 0.001f)
                        + (1 - CarInfo.gyroConfi) * atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My), &yawPre, 0.5f);
                taskEXIT_CRITICAL();
                break;
            case sGyro:
                taskENTER_CRITICAL();
                ICM42605_GetData(&CarInfo.icm, ICM_MODE_ACC | ICM_MODE_GYRO);
                CarInfo.icm.gz = Filter_MovingAvgf(&gyroFilter, CarInfo.icm.gz) + 0.08f;
                taskEXIT_CRITICAL();
                break;
            case sOptical:
                taskENTER_CRITICAL();
                PMW3901_Read_Data(&CarInfo.pmw);
                CarInfo.dx = (float) -CarInfo.pmw.deltaX;
                CarInfo.dy = (float) -CarInfo.pmw.deltaY;
                VecRotate(CarInfo.dx, CarInfo.dy, CarInfo.yaw);
                CarInfo.curX += CarInfo.dx;
                CarInfo.curY += CarInfo.dy;
                taskEXIT_CRITICAL();
                break;
            default:
                break;
        }
    }
    /* USER CODE END SensorHandleEntry */
}

/* USER CODE BEGIN Header_InfCalOpticalEntry */
/**
* @brief Function implementing the InfCalOpticalTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InfCalOpticalEntry */
void InfCalOpticalEntry(void *argument) {
    /* USER CODE BEGIN InfCalOpticalEntry */
    UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        if (CarInfo.Pi_Reset) {
            CarInfo.Pi_Reset = 0;
            Pi_ResetFromOS();
        }
        osDelay(10);
    }
    /* USER CODE END InfCalOpticalEntry */
}

/* KeyTimerCallback function */
void KeyTimerCallback(void *argument) {
    /* USER CODE BEGIN KeyTimerCallback */
    /* USER CODE END KeyTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

