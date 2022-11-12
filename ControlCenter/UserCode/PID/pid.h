/**
 * @file pid.h
 * @author ZheWana
 * @brief PID控制器头文件
 * @version 0.1
 * @date 2022-03-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PID_H
#define PID_H

#ifndef M_PI
#define M_PI 3.1415926535
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    struct ctr {
        float cur;
        float pre;
        float aim;
    } ctr;

    struct error {
        float cur;
        float pre;
        float sum;
        float bia;
    } error;

} Pid_t;

// PID实现函数
float PID_Realize(Pid_t *ctrl);

float PID_RealizeForAngle(Pid_t *ctrl);

// PID初始化函数
void PID_Init(Pid_t *ctrl, float kp, float ki, float kd, float aim);

#endif // PID_H