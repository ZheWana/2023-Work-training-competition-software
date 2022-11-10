/**
 * @file utils.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#ifndef CONTROLCENTER_UTILS_H
#define CONTROLCENTER_UTILS_H

#include "stdint.h"
#include "pid.h"

typedef struct CarControlBlock {
    // 状态数据
    float x, y;
    float yaw;// 弧度制

    // 存储三色物块抓取和放置顺序
    uint8_t order[3];
    enum orderCommand {
        Red = 0, Blue, Green
    } orderCommand;

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

    // PID姿态控制
    pid_t aPid;
    float aPidOut;
    const int aPidPeriod;
    enum aPidLock {
        aPidLocked, aPidUnlocked
    } aPidLock;

    // 状态机相关函数
    void (*RunMainState)(void);

    void (*RunFetchState)(void);

    void (*RunDropState)(void);
} CCB_Typedef;

extern CCB_Typedef CarInfo;

#endif //CONTROLCENTER_UTILS_H
