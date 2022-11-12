/**
 * @file utils.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#ifndef CONTROLCENTER_UTILS_H
#define CONTROLCENTER_UTILS_H

#include "stdint.h"
#include "PID/pid.h"

#define ToDig(rad) (rad * 57.295779513082320876798154814105)
#define ToRad(dig) (dig * 0.01745329251994329576923690768489)
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

typedef struct CarControlBlock {
    // 状态数据
    float curX, curY;
    float aimX, aimY;
    float yaw;// 弧度制
    float initYawOffset;

    const float unitSpeed;

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

void Speed2MotorConverter(float vx, float vy,
                          float *m1Speed, float *m2Speed, float *m3Speed, float *m4Speed);

#endif //CONTROLCENTER_UTILS_H
