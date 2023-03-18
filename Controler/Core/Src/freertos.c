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
#include "Filter/filter.h"
#include "freertos_os2.h"
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
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InfCalOpticalTa */
osThreadId_t InfCalOpticalTaHandle;
const osThreadAttr_t InfCalOpticalTa_attributes = {
        .name = "InfCalOpticalTa",
        .stack_size = 256 * 4,
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
    /* Infinite loop */
    for (;;) {
        if (!CarInfo.SerialOutputEnable)osThreadSuspend(SerialOutputHandle);

        for (int i = 0; i < 8; i++) {
            printf("%c", CarInfo.inf.rawData & (1 << i) ? '1' : '0');
        }
        printf("\r\n");

//        printf("data:");
//
//        printf("%d,", CarInfo.spd[0]);
//        printf("%d,", CarInfo.spd[1]);
//        printf("%d,", CarInfo.spd[2]);
//        printf("%d,", CarInfo.spd[3]);
//        printf("%f,", CarInfo.msPid[0].ctr.aim);
//        printf("%f,", CarInfo.psi[0]);
//        printf("%f,", CarInfo.mpPid[0].ctr.aim);
//
//        printf("%f,", CarInfo.yaw);
//        printf("%f,", CarInfo.avPid.ctr.aim);

//        printf("%f,", CarInfo.curX);
//        printf("%f,", CarInfo.cpPidX.ctr.aim);
//        printf("%f,", CarInfo.curY);
//        printf("%f,", CarInfo.cpPidY.ctr.aim);

//        printf("\r\n");
//        for (int i = 0; i < 32; i++) {
//            printf("%d", CarInfo.inf & (1 << i) ? 1 : 0);
//        }

//        printf("%.3f,", CarInfo.hmc.Mx);
//        printf("%.3f,", CarInfo.hmc.My);
//        printf("%.3f,", CarInfo.icm.gx);
//        printf("%.3f,", CarInfo.icm.gy);
//        printf("%.3f,", CarInfo.icm.gz);

//        printf("%f,", CarInfo.icm.ax);
//        printf("%f,", CarInfo.icm.ay);
//        printf("%f,", CarInfo.icm.az);
//        printf("%f,", CarInfo.icm.gx);
//        printf("%f,", CarInfo.icm.gy);
//        printf("%f,", CarInfo.icm.gz);

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
    /* Infinite loop */
    for (;;) {
//        CarInfo.RunMainState();
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

        for (int i = 0; i < 16; i++) {
            buff[i] = CarInfo.inf.rawData & (1 << i) ? '1' : '0';
        }
        buff[16] = '\0';
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);

        for (int i = 0; i < 16; i++) {
            buff[i] = CarInfo.inf.rawData & (1 << (i + 16)) ? '1' : '0';
        }
        buff[16] = '\0';
        LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);

        for (int i = 0; i < 4; i++) {
            sprintf(&buff[0], "%d :", i);
            for (int j = 0; j < 8; j++) {
                sprintf(&buff[3 + j], "%d", CarInfo.infrRecord[i] & (1 << j) ? 1 : 0);
            }
            sprintf(&buff[11], "\n\0");
            LCD_StringLayout(128, buff, Font_7x10, ST7735_BLACK, ST7735_WHITE);
        }

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
    /* Infinite loop */
    for (;;) {
        static enum SensorType {
            sInfrared,
            sCompass,
            sGyro,
            sOptical,
        } SensorType;
        static FilterTypedef_t gyroFilter = {0};
        static FilterTypedef_t yawFilter = {0};

        osMessageQueueGet(SensorMessageQueueHandle, &SensorType, 0, osWaitForever);

        switch (SensorType) {
            case sInfrared: {
                HC165_Get_Data(&CarInfo.inf.rawData);

                // Byte inversion
                uint8_t a = 0, b = 0;
                for (int i = 0; i < 8; i++) {
                    a |= CarInfo.inf.data[CarInfo.infrareDir.inFront] & (1 << i) ? 1 << (7 - i) : 0;
                    b |= CarInfo.inf.data[CarInfo.infrareDir.inLeft] & (1 << i) ? 1 << (7 - i) : 0;
                }
                CarInfo.inf.data[CarInfo.infrareDir.inFront] = a;
                CarInfo.inf.data[CarInfo.infrareDir.inLeft] = b;

                // 反转黑白
//                CarInfo.inf.rawData = ~CarInfo.inf.rawData;
            }
                break;
            case sCompass:
                QMC5883_GetData(&CarInfo.hmc);
                VecRotate(CarInfo.hmc.Mx, CarInfo.hmc.My, CarInfo.initYawOffset);
                CarInfo.yaw = Filter_MovingAvgf(&yawFilter, atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My));
                break;
            case sGyro:
                ICM42605_GetData(&CarInfo.icm, ICM_MODE_ACC | ICM_MODE_GYRO);
                CarInfo.icm.gz = Filter_MovingAvgf(&gyroFilter, CarInfo.icm.gz) + 0.08f;
                break;
            case sOptical:
                PMW3901_Read_Data(&CarInfo.pmw);
                CarInfo.dx = (float) -CarInfo.pmw.deltaX;
                CarInfo.dy = (float) -CarInfo.pmw.deltaY;
                VecRotate(CarInfo.dx, CarInfo.dy, CarInfo.yaw);
                CarInfo.curX += CarInfo.dx;
                CarInfo.curY += CarInfo.dy;
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
    /* Infinite loop */
    for (;;) {
        /// TODO:监测红外传感器数据，对光流数据进行校�?
        static int cntX = 0, cntY = 0;// X、Y轴格子计数；
        const uint8_t right = 0x0f, left = 0xf0, front = 0x0f, back = 0xf0;

        // 旋转红外传感器离散坐标系
        CarInfo.infrareDir.lastFront = CarInfo.infrareDir.inFront;
        CarInfo.infrareDir.inFront = GetFrontFromYaw(CarInfo.yaw);
        if (CarInfo.infrareDir.lastFront != CarInfo.infrareDir.inFront) {
            memset(CarInfo.infrRecord, 0, 4);
            CarInfo.infrareDir.inLeft = CarInfo.infrareDir.inFront + 1;
            CarInfo.infrareDir.inBack = CarInfo.infrareDir.inLeft + 1;
            CarInfo.infrareDir.inRight = CarInfo.infrareDir.inBack + 1;
        } else {
            for (int i = 0; i < 4; i++) {
                // Record infrare history
                CarInfo.infrRecord[i] |= CarInfo.inf.data[i];
                if (CarInfo.infrRecord[i] != 0)// fill zero if data is lost
                    u8ZeroFiller(&CarInfo.infrRecord[i]);

                // Clear parallel direction record
                if (CarInfo.infrRecord[i] == 0xff)CarInfo.infrRecord[i] = 0;
                // Zero Clear
                if (CarInfo.inf.data[i] == 0 && IsCarStatic)CarInfo.infrRecord[i] = 0;
            }

            // Judge record type
            switch (CarInfo.infrRecord[CarInfo.infrareDir.inFront] & CarInfo.infrRecord[CarInfo.infrareDir.inBack]) {
                case right:
                    CarInfo.curX = (++cntX) * MOVE_X_Grid;
                    CarInfo.infrRecord[CarInfo.infrareDir.inFront] = CarInfo.infrRecord[CarInfo.infrareDir.inBack] = 0;
                    printf("right datect\r\n");
                    break;
                case left:
                    CarInfo.curX = (--cntX) * MOVE_X_Grid;
                    CarInfo.infrRecord[CarInfo.infrareDir.inFront] = CarInfo.infrRecord[CarInfo.infrareDir.inBack] = 0;
                    printf("left datect\r\n");
                    break;
                default:// do nothing
                    break;
            }
            switch (CarInfo.infrRecord[CarInfo.infrareDir.inRight] & CarInfo.infrRecord[CarInfo.infrareDir.inLeft]) {
                case front:
                    CarInfo.curY = (++cntY) * MOVE_Y_Grid;
                    CarInfo.infrRecord[CarInfo.infrareDir.inRight] = CarInfo.infrRecord[CarInfo.infrareDir.inLeft] = 0;
                    printf("front datect\r\n");
                    break;
                case back:
                    CarInfo.curY = ((--cntY < 0) ? 0 : cntY) * MOVE_Y_Grid;
                    CarInfo.infrRecord[CarInfo.infrareDir.inRight] = CarInfo.infrRecord[CarInfo.infrareDir.inLeft] = 0;
                    printf("back datect\r\n");
                    break;
                default:// do nothing
                    break;
            }
        }

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

void u8ZeroFiller(uint8_t *data) {
    enum {
        fetch1,
        fetch0,
        fill,
    } state;

    uint8_t filler = 0;
    for (int i = 0; i < 8; i++) {
        switch (state) {
            case fetch1:
            Fetch1:
                if (!(*data & (1 << i))) {
                    state = fetch0;
                } else {
                    break;
                }
            case fetch0:
                if (*data & (1 << i)) {
                    state = fill;
                } else {
                    filler |= (1 << i);
                    break;
                }
            case fill:
                state = fetch1;
                *data |= filler;
                goto Fetch1;
                break;
            default:
                // Do nothing
                break;
        }
    }
}
/* USER CODE END Application */
