/**
 * @file stepHelper.h
 * @author ZheWana (zhewana@qq.com)
 * @brief help step motor control
 * @version 0.1
 * @date 2022-10-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef STEPHELPER_H
#define STEPHELPER_H
#include "stdint.h"
#include "tim.h"
#include "printf.h"


// predefinations

/*********************  Config Option Begin  ********************/

#define AutoInitBuffer (1)
#define AcclerateCurve (Curve_Trapezoidal)
#define BufferSize (512)

/*********************  Config Option End  ********************/

#ifndef __weak
#define __weak __attribute__((weak))
#endif // !__weak

#define Decelerate_USE 1
#define Decelerate_NOUSE 0

#define Curve_Trapezoidal (0x01)
#define Curve_S (0x02)

typedef struct stepTypedef {
    // portable
    TIM_HandleTypeDef* phtim;
    uint32_t channel;

    // Const
    float Fmin; // Hz
    float Fmax; // Hz
    float Tacc; // ms

    // var
    float t; // ms ----------------------internal use
    float Fcur; // Hz ----------------------internal use

    // buffer
    uint16_t buff0[BufferSize]; // ----------------------internal use
    uint16_t buff1[BufferSize]; // ----------------------internal use

    // states
    enum fillState {
        Acclerate,
        Constant,
        Decelerate,
        Stop
    } state; // ----------------------internal use

    enum runningLock {
        UNLOCK,
        LOCK
    } lock;

    uint32_t stepToGo;
    uint32_t buffToUse;
    uint32_t accStep;

    // flags
    uint8_t useDec : 1;
    uint8_t buffIndex : 1; // ----------------------internal use
    uint8_t buffRdy : 1; // ----------------------internal use
} stepTypedef;

void Step_Init(stepTypedef* hstep, TIM_HandleTypeDef* phtim, uint32_t channel, float Fmin, float Fmax, float Tacc);

void Step_BufferUsed(stepTypedef* hstep);

int Step_IsBuffRdy(stepTypedef* hstep);

uint16_t* Step_GetCurBuffer(stepTypedef* hstep);

uint32_t Step_BuffUsedLength(stepTypedef* hstep);

int Step_Lock(stepTypedef* hstep);

int Step_Unlock(stepTypedef* hstep);

int Step_Prefill(stepTypedef* hstep, int stepToGo, uint8_t useDec);

int Step_BuffFill(stepTypedef* hstep);

int Step_FillDecelerate(stepTypedef* hstep);

int Step_FillConstant(stepTypedef* hstep);

int Step_FillAccelerate(stepTypedef* hstep);

#endif // !STEPHELPER_H
