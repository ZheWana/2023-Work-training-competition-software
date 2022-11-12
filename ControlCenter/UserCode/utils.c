/**
 * @file utils.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#include <stddef.h>
#include "utils.h"
#include "tim.h"
#include "CommonKey/comKey.h"
#include "./ST7735/Inc/st7735.h"

void __RunMainState(void) {
    switch (CarInfo.mainState) {
        case mStart:
            // TODO:
            //  升降电机初始化
            //  按键初始化yaw坐标系

//            ST7735_Init();

            CarInfo.mainState = mScan;
            break;
        case mScan:// 扫描二维码,记录信息
            // 出库,到达定点后发出Rdy信号

            // 接收摄像头扫描结果

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
    // TODO: Complete converter
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

    }
}
