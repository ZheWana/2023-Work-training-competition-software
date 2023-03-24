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

#define ToDig(rad) (rad * 57.295779513082320876798154814105f)
#define ToRad(dig) (dig * 0.01745329251994329576923690768489f)

#define PMW_X_Grid (800)
#define PMW_Y_Grid (800)

#define CLIP_CLOSE (1000)
#define CLIP_OPEN  (900)

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
    int16_t spd[5];
    float spdAim[5];
    Pid_t msPid[5]; // motor speed pid
    float psi[5];
    Pid_t mpPid[5]; // motor position pid
    float mpPIDout[5];
    float spdStep;
    float spdLimit[5];
    bool mPsiCtr: 1;// 启用电机位置环控制标志位
    bool firstPsiLoop: 1;// 首次进入位置环标志

    // 整车控制相关
    PMW3901 pmw;
    float dx, dy;
    volatile float curX, curY;
    Pid_t cpPidX, cpPidY;// Car position pid
    float spdX, spdY;
    bool cPsiCtr: 1;
    bool SerialOutputEnable: 1;
    bool Pi_Reset: 1;
    bool Start_State: 1;
    volatile bool PiReceiveFlag: 1;

    // 整车姿态相关数据
    hmcData_t hmc;
    icmData_t icm;
    float yaw;// 弧度制
    int16_t yawOverFlowTime;
    float gyroConfi;
    float initYawOffset;
    float initGxOffset;
    float initGyOffset;
    float initGzOffset;
    uint32_t isCarMoving;// 定时器中断中监测整车是否静止，用作步骤规划

    // 状态机状态枚举
    enum mainState {// 主状态机
        mStart, mScan, mFetch, mDrop, mEnd
    } mainState;

    // PID姿态控制
    Pid_t avPid;// 角速度闭环
    Pid_t aPid;// 角度闭环
    float avPidOut;

    // 状态机相关函数
    void (*RunMainState)(void);
} CCB_Typedef;

extern CCB_Typedef CarInfo;
extern Shell shell;

#define LCD_EOP 0,NULL,Font_7x10,0,0

void LCD_StringLayout(uint16_t maxY, char *buff, FontDef font, uint16_t color, uint16_t bgcolor);

void SupportRotation(float position, uint32_t time);

void SupportRotationForOS(float dig, uint32_t time);

void ClipRotition(float position, uint32_t time);

void MoveTo(float X, float Y);

void TurnTo(float rad);

void Pi_SwitchFromOS(void);

void Pi_SwitchFromHAL(void);

void Pi_ResetFromOS(void);

void Pi_ResetFromHAL(void);

void Data_ReFormatData(uint16_t *array, int len);

uint8_t Data_RoughlyEqual(double curY, double curX, double aimY, double aimX, double thre);

#endif //CONTROLCENTER_UTILS_H
