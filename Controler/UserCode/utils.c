/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include "tim.h"
#include "key.h"
#include "spi.h"
#include "move.h"
#include "usart.h"
#include "utils.h"
#include "printf.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "CommonKey/comKey.h"
#include "LetterShell/shell.h"
#include "./Compass/QMC5883L.h"
#include "./ST7735-HAL/st7735.h"
#include "SerialParaChanger/SPChanger.h"
#include "LobotSerialServo/LobotSerialServo.h"

extern osTimerId_t KeyTimerHandle;

union UARTRxBuffer {
    uint16_t u16[4];
    int16_t i16[4];
} UARTRxBuffer;

#define PI_uX (UARTRxBuffer.u16[1])
#define PI_uY (UARTRxBuffer.u16[2])

#define PI_X (UARTRxBuffer.i16[1])
#define PI_Y (UARTRxBuffer.i16[2])

int PIMaxError = 3;

static void __RunMainState(void) {

    switch (CarInfo.mainState) {
        osStatus_t status;
        case mStart:// 升降电机、机械臂初始化
            // TODO:
            //  升降电机初始化
            //  机械臂方向初始化

            // 初始化接收DMA
            memset(UARTRxBuffer.u16, 0, sizeof(int16_t) * 4);
            HAL_UART_Receive_DMA(&huart4, (uint8_t *) UARTRxBuffer.u16, sizeof(uint16_t) * 4);

            // 等待树莓派初始化完成
            while (HAL_GPIO_ReadPin(Pi_Ready_GPIO_Port, Pi_Ready_Pin) != 1);

            CarInfo.mainState = mScan;
            break;
        case mScan:// 扫描二维码,树莓派记录信息

            // 出库,到达定点后旋转车身并发出Switch信号
            MoveTo(2, 1.5f);
            TurnTo(ToRad(180));
            Pi_SwitchFromOS();

            // 等待树莓派扫码完成
            while (PI_uX != 0xFFFF || PI_uY != 0xFFFF);

            CarInfo.mainState = mFetch;
            break;
        case mFetch:
            //  移动车身位置 进行十字校准
            MoveTo(5.5f, 1.5f);
            Pi_SwitchFromOS();

            for (int i = 0;; i++) {
                // 移动到抓取物块准备位置
                MoveTo(5.5f, 0.5f);

                // 抓取物块
                Pi_SwitchFromOS();
                osDelay(100);
                while (1) {
                    if (PI_uX == 0xFFFF && PI_uY == 0xFFFF)goto ENDLoop;
                    CarInfo.cpPidX.ctr.aim += PI_X;
                    CarInfo.cpPidY.ctr.aim += PI_Y;

                    // 误差满足要求则关闭地图闭环，抓取物块
                    if (abs(PI_X) <= PIMaxError && abs(PI_Y) <= PIMaxError) {
                        CarPositionLoopSet(0);
                        Pi_SwitchFromOS();
                        // TODO:
                        //  阻塞抓取物块
                        printf("Fetch one thing\r\n");

                        CarPositionLoopSet(1);
                        break;
                    }
                }
                ENDLoop:;
            }

            CarInfo.mainState = mDrop;
            break;
        case mDrop:// 放下物块
            MoveTo(5.5f, 3.5f);


            CarInfo.mainState = mEnd;
            break;
        case mEnd:
            // TODO:
            //  前往终点
            //  收起机械臂
            break;
    }

}


CCB_Typedef CarInfo = {
        .gyroConfi = 0.8f,
        .mPsiCtr = 0,
        .cPsiCtr = 1,
        .spdLimit = {20, 20, 20, 20, 10},
        .mainState = mStart,
        .SerialOutputEnable = 0,
        .RunMainState = __RunMainState,
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

/**
 * @brief 龙门支撑底座旋转
 * @param position 旋转位置（0-1000）
 * @param time 旋转所需时间(单位:ms)
 */
void SupportRotation(float position, uint32_t time) {
    LobotSerialServoMove(SupportServoID, (int16_t) position, time);
}

void SupportRotationForOS(float dig, uint32_t time) {
    int16_t position = (int16_t) (dig * 77.0f / 18 + 10);
    LobotSerialServoMove(SupportServoID, (int16_t) position, time);
    osDelay(time + 100);
}

/**
 * @brief 夹子电机旋转(夹子开关)
 * @param position 舵机位置单位(0-1000)
 * @param time 运行所需时间(单位:ms)
 */
void ClipRotition(float position, uint32_t time) {
    LobotSerialServoMove(ClipServoID, (int16_t) position, time);
//    HAL_Delay(time + 100);
}

void ClipCloseForOS(void) {
    ClipRotition(CLIP_CLOSE, 700);
}

void ClipOpenForOS(void) {
    ClipRotition(CLIP_OPEN, 700);
}

extern char chBuff;

void MoveTo(float X, float Y) {
    CarInfo.cpPidY.ctr.aim = Y * PMW_Y_Grid;
    CarInfo.cpPidX.ctr.aim = X * PMW_X_Grid;

    while (1)if (IsCarStatic)return;
}

void TurnTo(float rad) {
    CarInfo.aPid.ctr.aim = rad;

    while (1)if (IsCarStatic)return;
}

void Pi_SwitchFromOS(void) {
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_RESET);
}

void Pi_SwitchFromHAL(void) {
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(Pi_Switch_GPIO_Port, Pi_Switch_Pin, GPIO_PIN_RESET);
}

void Pi_ResetFromOS(void) {
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_RESET);
}

void Pi_ResetFromHAL(void) {
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(Pi_Reset_GPIO_Port, Pi_Reset_Pin, GPIO_PIN_RESET);
}


void Data_ReFormatData(uint16_t *array, int len) {
    char record;
    uint16_t temp[len];
    int j = 0;

    if (array[0] == 0x5555) {
        return;
    } else {
        for (int i = 0;; i++) {
            if (array[i] == 0x5555) {
                record = i;
                break;
            }
        }
        for (int i = record; i < len; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < record; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < len; i++) {
            array[i] = temp[i];
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {// shell用串口
        SPC_GetChar(chBuff);
        shellHandler(&shell, chBuff);
        HAL_UART_Receive_IT(huart, (uint8_t *) &chBuff, 1);
    }
    if (huart->Instance == UART4) {// 树莓派串口
        //  帧头      数据X      数据Y       帧尾
        // 0x5555    int16     int16      0xAAAA
        Data_ReFormatData(UARTRxBuffer.u16, 4);
    }
}

