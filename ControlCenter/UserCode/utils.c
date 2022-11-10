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

void RunMainState(void) {
    switch (CarInfo.mainState) {
        case mStart:
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


void RunFetchState(void) {
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

void RunDropState(void) {
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
        .aPidPeriod = 50,
        .RunMainState = RunMainState,
        .RunFetchState = RunFetchState,
        .RunDropState = RunDropState
};

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

    }
}
