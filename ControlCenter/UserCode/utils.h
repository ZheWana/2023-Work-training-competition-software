/**
 * @file utils.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#ifndef CONTROLCENTER_UTILS_H
#define CONTROLCENTER_UTILS_H

#include "stdint.h"

typedef struct CarControlBlock {
    // 当前场内坐标
    float x, y;

    // 存储三色物块抓取和放置顺序
    uint8_t order[3];
    enum orderCommand {
        Red = 0, Blue, Green
    };

    // 状态机状态枚举
    enum mainState {// 主状态机
        mStart, mScan, mFirstFetch, mFirstDrop, mEnd
    } mainState;

    enum fetchState {// 抓取从状态机
        fStart, fFetch1, fFetch2, fFetch3, fEnd
    } fetchState;

    enum dropState {// 放置从状态机
        dStart, dDrop1, dDrop2, dDrop3, dEnd
    } dropState;


} CCB_Typedef;

void RunFetchState(void);

void RunDropState(void);

void RunMainState(void);

#endif //CONTROLCENTER_UTILS_H
