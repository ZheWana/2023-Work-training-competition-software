/**
 * @file stepHelper.c
 * @author ZheWana (zhewana@qq.com)
 * @brief 步进电机DMA数组生成器
 * @version 0.1
 * @date 2022-10-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "stepHelper.h"
#include "math.h"
#include "stm32f401xc.h"

#ifndef M_PI
#define M_PI (3.1415926535f)
#endif // !M_PI

/**
 * @brief 初始化构造器
 *
 * @param hstep step句柄
 * @param phtim 定时器句柄（HAL库）
 * @param Fmin 最小起始频率
 * @param Fmax 加速最大频率
 * @param Tacc 加速时间
 */
void Step_Init(stepTypedef* hstep, TIM_HandleTypeDef* phtim, uint32_t channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, float Fmin, float Fmax, float Tacc)
{
    hstep->phtim = phtim;
    hstep->channel = channel;

    hstep->gpioPort = GPIOx;
    hstep->gpioPin = GPIO_Pin;

    switch (hstep->channel) {
    case TIM_CHANNEL_1:
        hstep->phtim->Instance->CCR1 = 0.5f * hstep->phtim->Instance->ARR;
        break;
    case TIM_CHANNEL_2:
        hstep->phtim->Instance->CCR2 = 0.5f * hstep->phtim->Instance->ARR;
        break;
    case TIM_CHANNEL_3:
        hstep->phtim->Instance->CCR3 = 0.5f * hstep->phtim->Instance->ARR;
        break;
    case TIM_CHANNEL_4:
        hstep->phtim->Instance->CCR4 = 0.5f * hstep->phtim->Instance->ARR;
        break;

    default:
        break;
    }

    hstep->Fmin = Fmin;
    hstep->Fmax = Fmax;
    hstep->Tacc = Tacc;

    hstep->t = -(1000.0f / Fmin);
    hstep->Fcur = Fmin;
    hstep->buffIndex = 0;

    hstep->state = Stop;

    hstep->useDec = 1;

#if AutoInitBuffer
    for (int i = 0; i < BufferSize; i++) {
        hstep->buff0[i] = 0;
        hstep->buff1[i] = 0;
    }
#endif
}

/**
 * @brief 加速数组生成器
 *
 * @param hstep step句柄
 * @return int 运行步数
 */
int Step_FillAccelerate(stepTypedef* hstep)
{
    if (hstep->state != Acclerate)
        return 0;

    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;

    // Acclerate
    int i = 0;
    for (; hstep->t <= hstep->Tacc; i++) {
        // overflow check
        if (i >= buffToUse)
            break;

        // get t
        hstep->t += 1000.0f / hstep->Fcur;

        // get freq
        switch (AcclerateCurve) {
        case Curve_Trapezoidal:
            hstep->Fcur = (hstep->Fmax - hstep->Fmin) * 1.0f / (hstep->Tacc) * (hstep->t) + hstep->Fmin;
            break;
        case Curve_S:
            hstep->Fcur = 0.5f * hstep->Fmax * cosf(M_PI - M_PI * hstep->t / hstep->Tacc) + hstep->Fmin + hstep->Fmax * 0.5f;
            break;

        default:
            hstep->Fcur = hstep->Fmin;
            break;
        }

        // get PSC & fill
        buffPtr[i] = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->phtim->Instance->ARR + 1)) / hstep->Fcur - 1);
    }

    hstep->buffIndex = !hstep->buffIndex;

    // Interrupted
    if (i >= buffToUse) {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    // Constant
    hstep->state = Constant;
    hstep->Fcur = hstep->Fmax;
    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->phtim->Instance->ARR + 1)) / hstep->Fcur - 1);
    for (; i < buffToUse; i++) {
        buffPtr[i] = PSC;
    }
    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 匀速数组生成器
 *
 * @param hstep step句柄
 * @return int 运行步数
 */
int Step_FillConstant(stepTypedef* hstep)
{
    if (hstep->state != Constant) {
        return 0;
    }

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->phtim->Instance->ARR + 1)) / hstep->Fcur - 1);
    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;

    for (int i = 0; i < buffToUse; i++) {
        buffPtr[i] = PSC;
    }
    hstep->buffIndex = !hstep->buffIndex;

    // State transfer
    // if (hstep->useDec)
    //     hstep->state = Decelerate;
    // else
    //     hstep->state = Stop;

    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 减速数组生成器
 *
 * @param hstep step句柄
 * @return int 运行步数
 */
int Step_FillDecelerate(stepTypedef* hstep)
{
    if (hstep->state != Decelerate)
        return 0;

    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;

    // Decelerate
    int i = 0;
    for (; hstep->Fcur > hstep->Fmin; i++) {
        // overflow check
        if (i >= buffToUse)
            break;

        // get t
        hstep->t -= 1000.0f / hstep->Fcur;

        // get freq
        hstep->Fcur = (hstep->Fmax - hstep->Fmin) * 1.0f / (hstep->Tacc) * (hstep->t) + hstep->Fmin;

        // get PSC & fill
        buffPtr[i] = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->phtim->Instance->ARR + 1)) / hstep->Fcur - 1);
    }

    hstep->buffIndex = !hstep->buffIndex;

    // Interrupted
    if (i >= buffToUse) {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    // Constant
    // hstep->state = Constant;
    hstep->Fcur = hstep->Fmin;
    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->phtim->Instance->ARR + 1)) / hstep->Fcur - 1);
    for (; i < buffToUse; i++) {
        buffPtr[i] = PSC;
    }
    hstep->buffRdy = 1;

    return buffToUse;
}

/**
 * @brief DMA用过数组后调用
 *
 * @param hstep step句柄
 *
 * @note RTOS环境下请使用二值信号量实现Fill操作与DMA传输的同步
 */
__weak void Step_BufferUsed(stepTypedef* hstep)
{
    hstep->buffRdy = 0;
    hstep->stepToGo -= hstep->buffToUse;

    if (hstep->stepToGo <= 0)
        hstep->state = Stop;
    else if (hstep->stepToGo <= hstep->accStep) {
        if (hstep->useDec) {
            hstep->state = Decelerate;
        } else {
            hstep->state = Constant;
        }
    }
}

/**
 * @brief 查询生成器数据状态
 *
 * @param hstep step句柄
 * @return int 运行步数
 *
 * @note RTOS环境下请使用二值信号量实现Fill操作与DMA传输的同步
 */
__weak int Step_IsBuffRdy(stepTypedef* hstep)
{
    return hstep->buffRdy;
}

/**
 * @brief 获取当前已填充缓冲区
 *
 * @param hstep
 * @return uint16_t*
 */
uint16_t* Step_GetCurBuffer(stepTypedef* hstep)
{
    return hstep->buffIndex ? hstep->buff0 : hstep->buff1;
}

/**
 * @brief 获取所使用的缓冲区大小
 *
 * @param hstep step句柄
 * @return uint32_t 缓冲区大小
 */
uint32_t Step_BuffUsedLength(stepTypedef* hstep)
{
    return hstep->buffToUse;
}

/**
 * @brief 句柄运行时锁定
 *
 * @param hstep step句柄
 * @return int 成功返回0
 *             失败返回-1
 */
int Step_Lock(stepTypedef* hstep)
{
    if (hstep->lock == UNLOCK)
        hstep->lock = LOCK;
    else
        return -1;
    return 0;
}

/**
 * @brief 句柄运行时解锁
 *
 * @param hstep step句柄
 * @return int 成功返回0
 *             失败返回-1
 */
int Step_Unlock(stepTypedef* hstep)
{
    if (hstep->lock == LOCK)
        hstep->lock = UNLOCK;
    else
        return -1;
    return 0;
}

/**
 * @brief 中止运行
 *
 * @param hstep step句柄
 */
void Step_Abort(stepTypedef* hstep)
{
    HAL_TIM_PWM_Stop(hstep->phtim, hstep->channel);
    HAL_TIM_PWM_Stop_DMA(hstep->phtim, hstep->channel);
    Step_Unlock(hstep);
}

/**
 * @brief 首次填充缓冲区
 *
 * @param hstep step句柄
 * @param stepToGo 待运行步数
 */
int Step_Prefill(stepTypedef* hstep, int stepToGo, uint8_t dir, uint8_t useDec)
{
    Step_Lock(hstep);

    hstep->stepToGo = stepToGo;
    hstep->useDec = useDec;

    switch (AcclerateCurve) {
    case Curve_Trapezoidal:
        hstep->accStep = (hstep->Fmin + hstep->Fmax) * hstep->Tacc / 2000;
        break;
    case Curve_S:
        hstep->accStep = 0.5f * hstep->Fmax * hstep->Tacc * 0.001f + hstep->Fmin * hstep->Tacc * 0.001f;
        break;
    }

    hstep->buffToUse = 0;

    HAL_GPIO_WritePin(hstep->gpioPort, hstep->gpioPin, dir);

    if (hstep->Tacc == 0) { // 加速时间为0
        hstep->Fcur = hstep->Fmax;
        hstep->state = Constant;
        hstep->buffToUse = Step_FillConstant(hstep);
    } else if (hstep->accStep > stepToGo) { // 加速步数不足
        hstep->Fcur = hstep->Fmin;
        hstep->state = Constant;
        hstep->buffToUse = Step_FillConstant(hstep);
    } else { // 加速步数充足
        hstep->state = Acclerate;
        hstep->buffToUse = Step_FillAccelerate(hstep);
    }

    return hstep->buffToUse;
}

/**
 * @brief 填充缓冲区，循环调用
 *
 * @param hstep step句柄
 */
int Step_BuffFill(stepTypedef* hstep)
{
    hstep->buffToUse = 0;

    switch (hstep->state) {
    case Acclerate:
        hstep->buffToUse = Step_FillAccelerate(hstep);
        break;
    case Constant:
        hstep->buffToUse = Step_FillConstant(hstep);
        break;
    case Decelerate:
        hstep->buffToUse = Step_FillDecelerate(hstep);
        break;

    default:
        return -1;
    }

    return hstep->buffToUse;
}
