/**
 * @file move.c
 * @author ZheWana(zhewana.cn)
 * @brief 整车运动控制相关代码
 * @date 2023/1/3
  */
#include "move.h"
#include "motor.h"
#include "utils.h"
#include "cmsis_os2.h"

// 位置环开关
void PositionLoopSet(int status) {
    CarInfo.psiCtr = status;
}

// 整车速度控制
void MecanumSpeedSet(float spdY, float spdX) {
    // 验证并修改闭环模式
    if (CarInfo.psiCtr) {
        PositionLoopSet(0);
        for (int i = 0; i < 4; i++) {
            PID_Reset(&CarInfo.mPid[i]);
            PID_Reset(&CarInfo.pPid[i]);
        }
    }

    CarInfo.mPid[0].ctr.aim = spdY + spdX;
    CarInfo.mPid[1].ctr.aim = spdY - spdX;
    CarInfo.mPid[2].ctr.aim = spdY - spdX;
    CarInfo.mPid[3].ctr.aim = spdY + spdX;
}

// 相对位置控制
void MecanumMove(float disY, float disX, float spdLimit, bool waitUntilStop) {
    float aimPsi[4];

    // 验证并修改闭环模式
    if (!CarInfo.psiCtr) {
        CarInfo.firstPsiLoop = 1;
        PositionLoopSet(1);
        for (int i = 0; i < 4; i++) {
            PID_Reset(&CarInfo.mPid[i]);
            PID_Reset(&CarInfo.pPid[i]);
        }
    }

    // 速度环限速
    if (spdLimit == 0)
        CarInfo.spdLimit = 0x7FFF;
    else
        CarInfo.spdLimit = spdLimit;

    aimPsi[0] = CarInfo.pPid[0].ctr.aim + disY + disX;
    aimPsi[1] = CarInfo.pPid[1].ctr.aim + disY - disX;
    aimPsi[2] = CarInfo.pPid[2].ctr.aim + disY - disX;
    aimPsi[3] = CarInfo.pPid[3].ctr.aim + disY + disX;

    if (CarInfo.firstPsiLoop) {
        CarInfo.firstPsiLoop = 0;
        aimPsi[0] = CarInfo.psi[0] + disY + disX;
        aimPsi[1] = CarInfo.psi[1] + disY - disX;
        aimPsi[2] = CarInfo.psi[2] + disY - disX;
        aimPsi[3] = CarInfo.psi[3] + disY + disX;
    }

    for (int i = 0; i < 4; i++)
        CarInfo.pPid[i].ctr.aim = aimPsi[i];

    if (waitUntilStop) {
        CarInfo.isCarMoving = 1;

        JudgeStatic:
        if (IsCarStatic) {
            osDelay(10);
            if (!IsCarStatic)goto JudgeStatic;
        } else goto JudgeStatic;
    }
}
