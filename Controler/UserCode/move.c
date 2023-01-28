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

/**
 * @brief 车体坐标系下速度控制
 * @param spdY Y轴速度
 * @param spdX X轴速度
 */
void MecanumSpeedSet(float spdY, float spdX) {
    // 验证并修改闭环模式
    if (CarInfo.mPsiCtr) {
        MotorPositionLoopSet(0);
        for (int i = 0; i < 4; i++) {
            PID_Reset(&CarInfo.msPid[i]);
            PID_Reset(&CarInfo.mpPid[i]);
        }
    }

    CarInfo.msPid[0].ctr.aim = spdY + spdX;
    CarInfo.msPid[1].ctr.aim = spdY - spdX;
    CarInfo.msPid[2].ctr.aim = spdY - spdX;
    CarInfo.msPid[3].ctr.aim = spdY + spdX;
}

/**
 * @brief 车体坐标系下移动控制
 * @param disY Y轴移动距离
 * @param disX X轴移动距离
 * @param spdLimit 限制的最大速度
 * @param waitUntilStop 是否阻塞等待动作执行完成（OS内使用）
 */
void MecanumMove(float disY, float disX, float spdLimit, bool waitUntilStop) {
    float aimPsi[4];

    // 验证并修改闭环模式
    if (!CarInfo.mPsiCtr) {
        CarInfo.firstPsiLoop = 1;
        MotorPositionLoopSet(1);
        for (int i = 0; i < 4; i++) {
            PID_Reset(&CarInfo.msPid[i]);
            PID_Reset(&CarInfo.mpPid[i]);
        }
    }

    // 速度环限速
    if (spdLimit == 0)
        CarInfo.spdLimit = 0x7FFF;
    else
        CarInfo.spdLimit = spdLimit;

    float plusY = disY * MOVE_Y_Grid;
    float plusX = disX * MOVE_X_Grid;

    aimPsi[0] = CarInfo.mpPid[0].ctr.aim + plusY + plusX;
    aimPsi[1] = CarInfo.mpPid[1].ctr.aim + plusY - plusX;
    aimPsi[2] = CarInfo.mpPid[2].ctr.aim + plusY - plusX;
    aimPsi[3] = CarInfo.mpPid[3].ctr.aim + plusY + plusX;

    if (CarInfo.firstPsiLoop) {
        CarInfo.firstPsiLoop = 0;
        aimPsi[0] = CarInfo.mpPid[0].ctr.aim + plusY + plusX;
        aimPsi[1] = CarInfo.mpPid[1].ctr.aim + plusY - plusX;
        aimPsi[2] = CarInfo.mpPid[2].ctr.aim + plusY - plusX;
        aimPsi[3] = CarInfo.mpPid[3].ctr.aim + plusY + plusX;
    }

    for (int i = 0; i < 4; i++)
        CarInfo.mpPid[i].ctr.aim = aimPsi[i];

    if (waitUntilStop) {
        CarInfo.isCarMoving = 1;

        JudgeStatic:
        if (IsCarStatic) {
            osDelay(10);
            if (!IsCarStatic)goto JudgeStatic;
        } else goto JudgeStatic;
    }
}

/**
 * @brief 车体坐标系下旋转控制
 * @param dig 旋转角度（逆时针为正）
 * @param spdLimit 限制的最大速度
 * @param waitUntilStop 是否阻塞等待动作执行完成（OS内使用）
 */
void MecanumRotate(float dig, float spdLimit, bool waitUntilStop) {
    float aimPsi[4];

    // 验证并修改闭环模式
    if (!CarInfo.mPsiCtr) {
        CarInfo.firstPsiLoop = 1;
        MotorPositionLoopSet(1);
        for (int i = 0; i < 4; i++) {
            PID_Reset(&CarInfo.msPid[i]);
            PID_Reset(&CarInfo.mpPid[i]);
        }
    }

    // 速度环限速
    if (spdLimit == 0)
        CarInfo.spdLimit = 0x7FFF;
    else
        CarInfo.spdLimit = spdLimit;

    float plusRotate;

    aimPsi[0] = CarInfo.mpPid[0].ctr.aim - dig;
    aimPsi[1] = CarInfo.mpPid[1].ctr.aim + dig;
    aimPsi[2] = CarInfo.mpPid[2].ctr.aim - dig;
    aimPsi[3] = CarInfo.mpPid[3].ctr.aim + dig;

    if (CarInfo.firstPsiLoop) {
        CarInfo.firstPsiLoop = 0;
        aimPsi[0] = CarInfo.mpPid[0].ctr.aim - dig;
        aimPsi[1] = CarInfo.mpPid[1].ctr.aim + dig;
        aimPsi[2] = CarInfo.mpPid[2].ctr.aim - dig;
        aimPsi[3] = CarInfo.mpPid[3].ctr.aim + dig;
    }

    for (int i = 0; i < 4; i++)
        CarInfo.mpPid[i].ctr.aim = aimPsi[i];

    if (waitUntilStop) {
        CarInfo.isCarMoving = 1;

        JudgeStatic:
        if (IsCarStatic) {
            osDelay(10);
            if (!IsCarStatic)goto JudgeStatic;
        } else goto JudgeStatic;
    }
}

void MapSpeedSet(float spdY, float spdX) {
    VecRotate(spdX, spdY, -CarInfo.yaw);
    MecanumSpeedSet(spdY, spdX);
}

void MapMove(float disY, float disX, float spdLimit, bool waitUntilStop) {
    VecRotate(disX, disY, -CarInfo.yaw);
    MecanumMove(disY, disX, spdLimit, waitUntilStop);
}
