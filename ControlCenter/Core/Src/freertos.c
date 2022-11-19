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
#include "stdio.h"
#include "usart.h"
#include "utils.h"
#include "stdlib.h"
#include "PID/pid.h"
#include "LogConfig.h"
#include "LobotSerialServo/LobotSerialServo.h"
#include "74HC165/74HC165.h"
#include "Compass/QMC5883L.h"
#include "CommonKey/comKey.h"
#include "ST7735/st7735.h"
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
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
        .name = "StateMachine",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AttitudeControl */
osThreadId_t AttitudeControlHandle;
const osThreadAttr_t AttitudeControl_attributes = {
        .name = "AttitudeControl",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StepControl */
osThreadId_t StepControlHandle;
const osThreadAttr_t StepControl_attributes = {
        .name = "StepControl",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialOutput */
osThreadId_t SerialOutputHandle;
const osThreadAttr_t SerialOutput_attributes = {
        .name = "SerialOutput",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeyInput */
osThreadId_t KeyInputHandle;
const osThreadAttr_t KeyInput_attributes = {
        .name = "KeyInput",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UpdateCoordinat */
osThreadId_t UpdateCoordinatHandle;
const osThreadAttr_t UpdateCoordinat_attributes = {
        .name = "UpdateCoordinat",
        .stack_size = 256 * 4,
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
/* Definitions for bSerialOutputSem */
osSemaphoreId_t bSerialOutputSemHandle;
const osSemaphoreAttr_t bSerialOutputSem_attributes = {
        .name = "bSerialOutputSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void _putchar(char character) {
    osSemaphoreAcquire(bSerialOutputSemHandle, 0xFFFF);
    HAL_UART_Transmit(&huart5, (uint8_t *) &character, 1, HAL_MAX_DELAY);
    osSemaphoreRelease(bSerialOutputSemHandle);
}

/* USER CODE END FunctionPrototypes */

void LEDControlEntry(void *argument);

void ScreenRefreshEntry(void *argument);

void StateMachineEntry(void *argument);

void AttitudeControlEntry(void *argument);

void StepControlEntry(void *argument);

void SerialOutputEntry(void *argument);

void KeyInputEntry(void *argument);

void UpdateCoordinatesEntry(void *argument);

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

    /* creation of bSerialOutputSem */
    bSerialOutputSemHandle = osSemaphoreNew(1, 1, &bSerialOutputSem_attributes);

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

    /* creation of KeyInput */
    KeyInputHandle = osThreadNew(KeyInputEntry, NULL, &KeyInput_attributes);

    /* creation of UpdateCoordinat */
    UpdateCoordinatHandle = osThreadNew(UpdateCoordinatesEntry, NULL, &UpdateCoordinat_attributes);

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
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
        osDelay(50);
        HAL_GPIO_WritePin(LED_System_GPIO_Port, LED_System_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
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
void ScreenRefreshEntry(void *argument) {
    /* USER CODE BEGIN ScreenRefreshEntry */
    /* Infinite loop */
    for (;;) {
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
        osDelay(1);
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
        if (CarInfo.aPidLock != aPidLocked) {
            // Get compass data
            static hmcData_t hmc;
            int res = QMC5883_GetData(&hmc);
            LogStart(LogAboutQMC5883_GetData);
            ErrorConLog(res == -1, "QMC5883_GetData:IIC ERROR");
            LogEnd(LogAboutQMC5883_GetData);
            hmc.Mx += 0;
            hmc.My += 0;

            // Rotating coordinate system
            hmc.Mx = hmc.Mx * cosf(CarInfo.initYawOffset) - hmc.My * sinf(CarInfo.initYawOffset);
            hmc.My = hmc.Mx * sinf(CarInfo.initYawOffset) + hmc.My * cosf(CarInfo.initYawOffset);

            if (!CarInfo.isYawInited) {//Init coordinate system
                CarInfo.isYawInited = 1;

                float sumX = 0, sumY = 0;
                for (int i = 0; i < 100; i++) {
                    res = QMC5883_GetData(&hmc);
                    if (res != 0)break;
                    sumX += hmc.Mx;
                    sumY += hmc.My;
                }
                CarInfo.initYawOffset = atan2f(sumX * 0.01f, sumY * 0.01f);
                LogStart(LogAboutQMC5883_CordanateInit);
                ErrorConLog(res == -1, "Init coordinate system FAILED with:[QMC5883_GetData:IIC ERROR]");
                InfoConLog(res == 0, "Init coordinate system success,Init yaw offset = %f", CarInfo.initYawOffset);
                LogEnd(LogAboutQMC5883_CordanateInit);
            } else {// PID close loop
                CarInfo.aPid.ctr.aim = 0;
                CarInfo.aPid.ctr.cur = CarInfo.yaw = atan2f(hmc.Mx, hmc.My);
                WarnConLog(res == 0 && hmc.Mx == 0 && hmc.My == 0, "QMC5883_GetData:All Zero Data");
                osSemaphoreAcquire(bGetaPidOutSemHandle, UINT32_MAX);
                CarInfo.aPidOut = PID_RealizeForAngle(&CarInfo.aPid);// Use radian system
                osSemaphoreRelease(bGetaPidOutSemHandle);
                CarInfo.aPid.ctr.pre = CarInfo.aPid.ctr.cur;
            }
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
        uint8_t buff[16] = {[0] = 0x55, [15] = 0xAA};
        enum cmdList {
            Reset = 0x0, Init, DriveAsStep, Abort, DriveAsMotor,
        } cmd;

        if (1) {//CarInfo.aimX != CarInfo.curX || CarInfo.aimY != CarInfo.curY) {
            float vx = CarInfo.aimX - CarInfo.curX, vy = CarInfo.aimY - CarInfo.curY;
            float freq[4];


            Speed2MotorConverter(vx, vy, &freq[0], &freq[1], &freq[2], &freq[3]);
            // TODO:
            //  Add aPid data to freq
            buff[1] = DriveAsMotor;
            for (int i = 0; i < 4; i++) {
                buff[2] = i;
                if (freq[i] < 0) {
                    freq[i] = -freq[i];
                    buff[7] = 0;
                } else {
                    buff[7] = 1;
                }
                *(uint32_t *) (buff + 3) = *(uint32_t *) &freq[i];
                HAL_SPI_Transmit(&hspi1, (uint8_t *) buff, 16, HAL_MAX_DELAY);
            }

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
void SerialOutputEntry(void *argument) {
    /* USER CODE BEGIN SerialOutputEntry */
    /* Infinite loop */
    for (;;) {
        vTaskSuspend(SerialOutputHandle);
        osDelay(1);
    }
    /* USER CODE END SerialOutputEntry */
}

/* USER CODE BEGIN Header_KeyInputEntry */
/**
* @brief Function implementing the KeyInput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyInputEntry */
void KeyInputEntry(void *argument) {
    /* USER CODE BEGIN KeyInputEntry */
    /* Infinite loop */
    for (;;) {
        ComKey_Handler();
        osDelayUntil(Key_PollingPeriod);
    }
    /* USER CODE END KeyInputEntry */
}

/* USER CODE BEGIN Header_UpdateCoordinatesEntry */
/**
* @brief Function implementing the UpdateCoordinat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UpdateCoordinatesEntry */
void UpdateCoordinatesEntry(void *argument) {
    /* USER CODE BEGIN UpdateCoordinatesEntry */
    /* Infinite loop */
    for (;;) {
        // Update Coordinates Helper struct instance
        static struct ucHelper {
            uint16_t sData;
            uint8_t sGroup[4];
            enum groupName {
                gUp = 0, gRight, gDown, gLeft
            } groupName: 2;

            // State machines and Timers for each group
            enum state {
                Start,
                FirstOn, SecondOn, ThirdOn, ForthOn, EndOn,
                FirstBack, SecondBack, ThirdBack, ForthBack, EndBack
            } stateList[4];
            uint32_t timerList[4];

            // Sensor position for each group
            const enum {
                Step1 = 0b0001000, Step2 = 0b0010100, Step3 = 0b0100010, Step4 = 0b1000001
            } Steps;
        } ucHelper;
        static float *handledData;
        float addItem = 1;

        HC165_Get_Data(&ucHelper.sData);
        ucHelper.sGroup[gUp] = ((0x7F << 4) & ucHelper.sData) >> 4;                                      // Head at 9
        ucHelper.sGroup[gRight] = ((0x7F << 8) & ucHelper.sData) >> 8;                                   // Head at 5
        ucHelper.sGroup[gDown] = (0xF << 12 & ucHelper.sData) >> 12 | ((0x7) & ucHelper.sData) << 4;     // Head at 1
        ucHelper.sGroup[gLeft] = ((0x7F) & ucHelper.sData);                                              // Head at 13

        if (ucHelper.sData != 0 && ucHelper.sData != UINT16_MAX) {
            for (int i = 0; i < 4; ++i) {
                enum state *pState = &ucHelper.stateList[i];
                uint8_t sGroup = ucHelper.sGroup[i];
                uint32_t *pTimer = &ucHelper.timerList[i];
                switch (*pState) {
                    case Start:
                        if ((sGroup & Step1) == Step1) {
                            *pState = FirstOn;
                            *pTimer = 0;
                        } else if ((sGroup & Step4) == Step4) {
                            *pState = FirstBack;
                            *pTimer = 0;
                        }
                        break;
                    case FirstOn:
                    case FirstBack:
                        if ((sGroup & Step2) == Step2 && *pState == FirstOn) {
                            *pState = SecondOn;
                            *pTimer = 0;
                        } else if ((sGroup & Step3) == Step3 && *pState == FirstBack) {
                            *pState = SecondBack;
                            *pTimer = 0;
                        }
                        break;
                    case SecondOn:
                    case SecondBack:
                        if ((sGroup & Step3) == Step3 && *pState == SecondOn) {
                            *pState = ThirdOn;
                            *pTimer = 0;
                        } else if ((sGroup & Step2) == Step2 && *pState == SecondBack) {
                            *pState = ThirdBack;
                            *pTimer = 0;
                        }
                        break;
                    case ThirdOn:
                    case ThirdBack:
                        if ((sGroup & Step4) == Step4 && *pState == ThirdOn) {
                            *pState = ForthOn;
                            *pTimer = 0;
                        } else if ((sGroup & Step1) == Step1 && *pState == ThirdBack) {
                            *pState = ForthBack;
                            *pTimer = 0;
                        }
                        break;
                    case ForthOn:
                    case ForthBack:
                        if ((sGroup & Step4) == 0 && *pState == ForthOn) {
                            *pState = EndOn;
                            *pTimer = 0;
                        } else if ((sGroup & Step1) == 0 && *pState == ForthBack) {
                            *pState = EndBack;
                            *pTimer = 0;
                        }
                        break;
                    case EndOn:
                    case EndBack:
                        handledData = (i % 2) == 0 ? &CarInfo.curX : &CarInfo.curY;
                        if (*pState == EndBack)addItem *= -1;
                        if (i == gDown || i == gLeft)addItem *= -1;
                        *handledData += addItem;
                        *pState = Start;
                        *pTimer = 0;
                        break;
                    default:;
                }

                *pTimer += UC_PollingPeriod;
                if (*pTimer >= UC_TimerResetPeriod) { *pState = Start; }
            }
        }

        LogStart(LogAboutInfrared);
        WarnConLog(ucHelper.sData == 0 || ucHelper.sData == UINT16_MAX, "Sensor Data ERROR");
        for (int i = 0; i < 16; ++i) {
            DataLog("%d", ucHelper.sData & (1 << i) ? 1 : 0);
        }
        printf("\n");
        for (int gName = 0; gName < 4; gName++) {
            if (ucHelper.sGroup[gName] != 0 && ucHelper.sGroup[gName] != 0x7F) {
                switch (gName) {
                    case gUp:
                        DataLog("gUp:    ");
                        break;
                    case gRight:
                        DataLog("gRight: ");
                        break;
                    case gDown:
                        DataLog("gDown:  ");
                        break;
                    case gLeft:
                        DataLog("gLeft:  ");
                        break;
                    default:;
                }
                for (int i = 0; i < 7; ++i) {
                    DataLog("%d", ucHelper.sGroup[gName] & (1 << i) ? 1 : 0);
                }
                DataLog("\n");
            }
        }
        LogEnd(LogAboutInfrared);

        osDelayUntil(UC_PollingPeriod);
    }
    /* USER CODE END UpdateCoordinatesEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

