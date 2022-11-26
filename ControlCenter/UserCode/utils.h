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
    float vx, vy;

    // 矫正航向角坐标系数据
    float initYawOffset;
    uint8_t isYawInited: 1;

    const float unitSpeed;

    // 存储三色物块抓取和放置顺序
    uint8_t order[3];
    enum orderCommand {
        Red = 0, Blue, Green
    } orderCommand;

    // 状态机状态枚举
    enum mainState {// 主状态机
        mStart, mScan, mIdentify, mFirstFetch, mFirstDrop, mEnd
    } mainState;

    enum fetchState {// 抓取从状态机
        fStart, fFetch1, fFetch2, fFetch3, fEnd
    } fetchState;

    enum dropState {// 放置从状态机
        dStart, dDrop1, dDrop2, dDrop3, dEnd
    } dropState;

    // PID姿态控制
    Pid_t aPid;
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

int Step_MoveAsMotor(uint8_t motorID, float freq);

int Step_SetSpeed(uint8_t motorID, uint32_t startSpeed, uint32_t finalSpeed, uint32_t accTime);

int Step_MoveSteps(uint8_t motorID, uint8_t dir, uint8_t useDec, uint32_t stepNum);

void Step_Abort(uint8_t motorID);

void SupportRotation(float position, uint32_t time);

void ClipRotition(float position, uint32_t time);

void MoveX(int64_t steps);

void MoveY(int64_t steps);

void Rotate(int64_t steps);

#endif //CONTROLCENTER_UTILS_H
