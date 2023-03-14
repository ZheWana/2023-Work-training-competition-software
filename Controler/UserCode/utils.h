/**
 * @file utils.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/8
  */
#ifndef CONTROLCENTER_UTILS_H
#define CONTROLCENTER_UTILS_H

#include "stdint.h"
#include "stdbool.h"
#include "PID/pid.h"
#include "PMW3901/PMW3901.h"
#include "Compass/QMC5883L.h"
#include "ICM42605/ICM42605.h"
#include "LetterShell/shell.h"
#include "ST7735-HAL/fonts.h"

#define ToDig(rad) (rad * 57.295779513082320876798154814105)
#define ToRad(dig) (dig * 0.01745329251994329576923690768489)

#define PMW_Grid 1280

#define IsCarStatic (!CarInfo.isCarMoving)

#define LimitFloat(input, min, max) do{ \
    input=input<min?min:input>max?max:input; \
}while(0)

#define VecRotate(x, y, theta) do{              \
    float tx = x,ty = y;                        \
    tx = x * cosf(theta) - y * sinf(theta);     \
    ty = x * sinf(theta) + y * cosf(theta);     \
    x = tx;                                     \
    y = ty;                                     \
}while(0)

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

typedef struct CarControlBlock {
    // 电机控制相关
    int16_t spd[4];
    float spdAim[4];
    Pid_t msPid[4]; // motor speed pid
    float psi[4];
    Pid_t mpPid[4]; // motor position pid
    float mpPIDout[4];
    float spdStep;
    float spdLimit[4];
    bool mPsiCtr: 1;// 启用电机位置环控制标志位
    bool firstPsiLoop: 1;// 首次进入位置环标志

    // 整车控制相关
    PMW3901 pmw;
    float dx, dy;
    float curX, curY;
    Pid_t cpPidX, cpPidY;// Car position pid
    float spdX, spdY;
    bool cPsiCtr: 1;
    bool SerialOutputEnable: 1;

    // 边界传感器数据
    union boundData {
        uint32_t rawData;
        uint8_t data[4];
    } inf;


    // 整车姿态相关数据
    hmcData_t
            hmc;
    icmData_t icm;
    float yaw;// 弧度制
    float initYawOffset;
    float initGxOffset;
    float initGyOffset;
    float initGzOffset;
    uint8_t isYawInited: 1;
    uint32_t isCarMoving;// 定时器中断中监测整车是否静止，用作步骤规划

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
    Pid_t avPid;// 角速度闭环
    Pid_t aPid;// 角度闭环
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

enum {
    inFront,// 0 -7 :front（inverse）
    inLeft,// 8 -15:left(inverse)
    inBack,// 16-23:back
    inRight,// 24-31:right
};

extern CCB_Typedef CarInfo;
extern Shell shell;

#define LCD_EOP 0,NULL,Font_7x10,0,0

void LCD_StringLayout(uint16_t maxY, char *buff, FontDef font, uint16_t color, uint16_t bgcolor);

void Speed2MotorConverter(float vx, float vy,
                          float *m1Speed, float *m2Speed, float *m3Speed, float *m4Speed);

void SupportRotation(float position, uint32_t time);

void ClipRotition(float position, uint32_t time);

#endif //CONTROLCENTER_UTILS_H
