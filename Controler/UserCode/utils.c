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
#include "printf.h"
#include <stddef.h>
#include "cmsis_os2.h"
#include "CommonKey/comKey.h"
#include "LetterShell/shell.h"
#include "./Compass/QMC5883L.h"
#include "./ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"

extern osTimerId_t KeyTimerHandle;

void __RunMainState(void) {

    switch (CarInfo.mainState) {
        osStatus_t status;
        case mStart:
            // TODO:
            //  升降电机初始化
            //  机械臂方向初始化

            //  按键按下初始化yaw坐标系
            // Hard Init

            // Soft Init

            CarInfo.mainState = mScan;
            break;
        case mScan:// 扫描二维码,记录信息
            // TODO:
            //  出库,到达定点后发出Rdy信号
            //  接收摄像头扫描结果
            while (1);
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
        .gyroConfi = 0.8f,
        .infrDir = {0, 1, 2, 3},
        .mPsiCtr = 0,
        .cPsiCtr = 1,
        .spdLimit = {20, 20, 20, 20},
        .order[Red] = 1,
        .order[Blue] = 2,
        .order[Green] = 3,
        .mainState = mStart,
        .fetchState = fStart,
        .dropState = dStart,
        .SerialOutputEnable = 1,
        .RunMainState = __RunMainState,
        .RunFetchState = __RunFetchState,
        .RunDropState = __RunDropState,
        .aPidPeriod = 50,
//        .spdAim = {20, 20, 20, 20},
};
Shell shell;

void LCD_StringLayout(uint16_t maxY, char *buff, FontDef font, uint16_t color, uint16_t bgcolor) {
    static uint16_t y = 0;
    static uint8_t fullScreenFlag = 0;
    uint16_t add = 0;

    // End of page
    if (maxY == 0 && buff == NULL && color == 0 && bgcolor == 0) {
        fullScreenFlag = add = y = 0;
//        ST7735_FillScreen(ST7735_WHITE);
        return;
    }

    if (font.data == Font_11x18.data)
        add = 18;
    else if (font.data == Font_16x26.data)
        add = 26;
    else if (font.data == Font_7x10.data)
        add = 10;

    if (y + add <= maxY) {
        ST7735_DrawString(0, y, buff, font, color, bgcolor);
        y += add;
    } else {
        if (fullScreenFlag == 0)
            ST7735_DrawString(0, maxY - add, "Full Content   ", font, color, bgcolor);
    }
}

uint8_t IsVerticalFliped(void) {
    // 返回值传递信息，表示横竖线是否是反的：0正，1反
    if ((CarInfo.yaw > -M_PI / 4 && CarInfo.yaw <= M_PI / 4)
        || (CarInfo.yaw <= -M_PI * 3.0f / 4 && CarInfo.yaw > M_PI * 3.0f / 4))
        return 0;
    else return 1;

}

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

// GPIO中断，用于与树莓派通信，上升沿表示检测到校准线
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint8_t cntX = 0, cntY = 0;
    switch (GPIO_Pin) {
        case GPIO_PIN_3:// 横线触发信号
            if (IsVerticalFliped())goto Vertical;
        Horizontal:// 横线对应Y变化
            if (CarInfo.cpPidY.ctr.aim > CarInfo.curY)
                ++cntY;
            else
                --cntY;

            break;
        case GPIO_PIN_5:// 竖线触发信号
            if (IsVerticalFliped())goto Horizontal;
        Vertical:// 竖线对应X变化
            if (CarInfo.cpPidX.ctr.aim > CarInfo.curX)
                ++cntX;
            else
                --cntX;

            break;
        default:
            break;
    }
    printf("X:%d,%d\r\n", cntX, cntY);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {// shell用串口
        SPC_GetChar(chBuff);
        shellHandler(&shell, chBuff);
        HAL_UART_Receive_IT(huart, (uint8_t *) &chBuff, 1);
    }
    if (huart->Instance == UART4) {// 树莓派串口
        // 数据待处理
        // 接收函数没写
    }
}

