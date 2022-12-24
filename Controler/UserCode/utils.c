/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include "./ST7735-HAL/st7735.h"
#include "./Compass/QMC5883L.h"
#include "CommonKey/comKey.h"
#include "LobotSerialServo/LobotSerialServo.h"
#include "LogConfig.h"
#include "cmsis_os.h"
#include <stddef.h>
#include "utils.h"
#include "tim.h"
#include "key.h"
#include "spi.h"

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
            PID_Init(&CarInfo.aPid, 2000, 0, 0, 0);// 7000 0 0

            CarInfo.mainState = mScan;
            CarInfo.aPidLock = aPidUnlocked;
            break;
        case mScan:// 扫描二维码,记录信息
            // TODO:
            //  出库,到达定点后发出Rdy信号
            //  接收摄像头扫描结果
            MoveX(StepsEachGrid_X * 7);
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
        .unitSpeed = 100,
};


void Speed2MotorConverter(float vx, float vy,
                          float *m1Speed, float *m2Speed, float *m3Speed, float *m4Speed) {
    // TODO: Test converter
    // 轮子编号      0  1  2  3
    // 前进         1  1  1  1
    // 后退         0  0  0  0
    // 左移         0  1  1  0
    // 右移         1  0  0  1
    *m1Speed = vy - vx;
    *m2Speed = vy + vx;
    *m3Speed = vy + vx;
    *m4Speed = vy - vx;
}

uint8_t buff[16] = {[0] = 0x55, [15] = 0xAA};

int Step_MoveAsMotor(uint8_t motorID, float freq) {
    int res = -1;
    buff[1] = 0x04;
    buff[2] = motorID;
    if (freq < 0) {
        freq = -freq;
        buff[7] = 0;
    } else {
        buff[7] = 1;
    }
    *(uint32_t *) (&buff[3]) = *(uint32_t *) &freq;
    res = HAL_SPI_Transmit(&hspi1, (uint8_t *) buff, 16, HAL_MAX_DELAY);

    return res;
}

int Step_SetSpeed(uint8_t motorID, uint32_t startSpeed, uint32_t finalSpeed, uint32_t accTime) {
    int res = -1;
    buff[1] = 0x01;
    buff[2] = motorID;
    *(uint32_t *) (&buff[3]) = startSpeed;
    *(uint32_t *) (&buff[7]) = finalSpeed;
    *(uint32_t *) (&buff[11]) = accTime;
    res = HAL_SPI_Transmit(&hspi1, (uint8_t *) buff, 16, HAL_MAX_DELAY);
    return res;
}

int Step_MoveSteps(uint8_t motorID, uint8_t dir, uint8_t useDec, uint32_t stepNum) {
    int res = -1;
    buff[1] = 0x02;
    buff[2] = motorID;
    *(uint32_t *) (&buff[3]) = stepNum;
    buff[7] = dir;
    buff[8] = useDec;
    res = HAL_SPI_Transmit(&hspi1, (uint8_t *) buff, 16, HAL_MAX_DELAY);
    return res;
}

void Step_Abort(uint8_t motorID) {
    buff[1] = 0x03;
    buff[2] = motorID;
    HAL_SPI_Transmit(&hspi1, (uint8_t *) buff, 16, HAL_MAX_DELAY);
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

void MoveY(int64_t steps) {
    if (steps != 0) {
        int dir = (steps > 0);
        if (steps < 0)steps = -steps;
        Step_MoveSteps(0, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(1, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(2, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(3, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
    } else {
        return;
    }
}

void MoveX(int64_t steps) {
    if (steps != 0) {
        int dir = (steps > 0);
        if (steps < 0)steps = -steps;
        Step_MoveSteps(0, !dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(1, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(2, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(3, !dir, 1, steps);
        HAL_Delay(CommunicationInterval);
    } else {
        return;
    }
}

void Rotate(int64_t steps) {
    if (steps != 0) {
        int dir = (steps > 0);
        if (steps < 0)steps = -steps;
        Step_MoveSteps(0, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(1, !dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(2, dir, 1, steps);
        HAL_Delay(CommunicationInterval);
        Step_MoveSteps(3, !dir, 1, steps);
        HAL_Delay(CommunicationInterval);
    } else {
        return;
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

    }
}
