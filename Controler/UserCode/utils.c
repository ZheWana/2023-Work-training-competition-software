/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include "tim.h"
#include "key.h"
#include "spi.h"
#include "utils.h"
#include <stddef.h>
#include "CommonKey/comKey.h"
#include "LetterShell/shell.h"
#include "./Compass/QMC5883L.h"
#include "./ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"

void __RunMainState(void) {
    int i = 0;
    switch (CarInfo.mainState) {
        case mStart:
            // TODO:
            //  升降电机初始化
            //  机械臂方向初始化

            //  按键按下初始化yaw坐标系
            // Hard Init
            QMC5883_Init();
//            ST7735_Backlight_On();
//            ST7735_Init();

            // Soft Init
            KeyInit();
            PID_Init(&CarInfo.aPid, 2000, 0, 0);// 7000 0 0

            CarInfo.mainState = mScan;
            CarInfo.aPidLock = aPidUnlocked;
            break;
        case mScan:// 扫描二维码,记录信息
            // TODO:
            //  出库,到达定点后发出Rdy信号
            //  接收摄像头扫描结果
            HAL_Delay(2000);

            CarInfo.mainState = mIdentify;
            break;
        case mIdentify:
            // TODO:
            //  移动车身位置
            //  识别三个物块颜色

            CarInfo.mainState = mFirstFetch;
            break;
        case mFirstFetch:// 第一次抓取
            if (CarInfo.RunFetchState != NULL) {
                CarInfo.RunFetchState();
            }
            break;
        case mFirstDrop:// 第一次放下
            if (CarInfo.RunDropState != NULL) {
                CarInfo.RunDropState();
            }
            break;
        case mEnd:
            // TODO:
            //  前往终点
            //  收起机械臂
            break;
    }

}


void __RunFetchState(void) {
    if (CarInfo.mainState == mFirstFetch) {
        switch (CarInfo.fetchState) {
            case fStart:

                CarInfo.fetchState += 1;
                break;
            case fFetch1:
                break;
            case fFetch2:
                break;
            case fFetch3:
                break;
            case fEnd:

                // 子状态机完成,主状态机转换
                CarInfo.mainState += 1;
                break;
        }
    }
}

void __RunDropState(void) {
    if (CarInfo.mainState == mFirstDrop) {
        switch (CarInfo.dropState) {
            case dStart:

                CarInfo.dropState += 1;
                break;
            case dDrop1:
                break;
            case dDrop2:
                break;
            case dDrop3:
                break;
            case dEnd:
                break;
        }
    }
}

CCB_Typedef CarInfo = {
        .psiCtr = 1,
        .spdLimit = 20,
        .order[Red] = 1,
        .order[Blue] = 2,
        .order[Green] = 3,
        .mainState = mStart,
        .fetchState = fStart,
        .dropState = dStart,
        .RunMainState = __RunMainState,
        .RunFetchState = __RunFetchState,
        .RunDropState = __RunDropState,
        .aPidPeriod = 50,
};
Shell shell;

/**
 * @brief 龙门支撑底座旋转
 * @param position 旋转角度,面向前方为0°(范围:-9.6~230.4)
 * @param time 旋转所需时间(单位:ms)
 */
void SupportRotation(float position, uint32_t time) {
    position = position > 230.4f ? 230.4f : position < -9.6f ? -9.6f : position;
    LobotSerialServoMove(SupportServoID, (int16_t) (960 - (position * 25 / 6)), time);
    HAL_Delay(time + 100);
}

/**
 * @brief 夹子电机旋转(夹子开关)
 * @param position 舵机位置单位(0-1000)
 * @param time 运行所需时间(单位:ms)
 */
void ClipRotition(float position, uint32_t time) {
    LobotSerialServoMove(ClipServoID, (int16_t) position, time);
    HAL_Delay(time + 100);
}

extern char chBuff;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        SPC_GetChar(chBuff);
        shellHandler(&shell, chBuff);
        HAL_UART_Receive_IT(huart, (uint8_t *) &chBuff, 1);
    }
}

