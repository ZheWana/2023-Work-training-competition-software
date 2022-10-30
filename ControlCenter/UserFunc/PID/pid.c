/**
 * @file pid.c
 * @author ZheWana
 * @brief PID控制器源文件
 * @version 0.1
 * @date 2022-03-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "pid.h"
#include "main.h"
#include "math.h"

// PID实现函数
float PID_Realize(pid_t* ctrl)
{
    ctrl->error.cur = ctrl->ctr.aim - ctrl->ctr.cur;
    ctrl->error.sum += ctrl->error.cur;

    // 中线附近屏蔽积分作用
    // if (fabsf(ctrl->error.cur) < 1) { // fabsf(ctrl->error.cur) < 1 ||
    //     ctrl->error.sum = 0;
    // }

    ctrl->error.bia = ctrl->error.cur - ctrl->error.pre;
    ctrl->error.pre = ctrl->error.cur;

    ctrl->ctr.pre = ctrl->ctr.cur;
    return ctrl->kp * ctrl->error.cur + ctrl->ki * ctrl->error.sum + ctrl->kd * ctrl->error.bia;
}

float PID_RealizeForAngle(pid_t* ctrl)
{
    ctrl->error.cur = ctrl->ctr.aim - ctrl->ctr.cur;
    if (fabsf(ctrl->error.cur) > M_PI && fabsf(ctrl->error.cur) < 2 * M_PI) { // 角度溢出处理
        if (ctrl->error.cur > 0) {
            ctrl->error.cur -= 2 * M_PI;
        } else if (ctrl->error.cur < 0) {
            ctrl->error.cur += 2 * M_PI;
        }
    }
    ctrl->error.sum += ctrl->error.cur;
    ctrl->error.bia = ctrl->error.cur - ctrl->error.pre;
    ctrl->error.pre = ctrl->error.cur;
    return ctrl->kp * ctrl->error.cur + ctrl->ki * ctrl->error.sum + ctrl->kd * ctrl->error.bia;
}

// PID初始化函数
void PID_Init(pid_t* ctrl, float kp, float ki, float kd, float aim)
{
    ctrl->kp = kp;
    ctrl->ki = ki;
    ctrl->kd = kd;
    ctrl->ctr.aim = aim;
}