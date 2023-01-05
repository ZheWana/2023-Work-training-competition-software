/**
 * @file motor.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/1/2
  */
#include "motor.h"
#include "tim.h"
#include "main.h"


int Motor_Drive(int motorID, float duty, int dir) {
    volatile TIM_TypeDef *TIMx = TIM2;

    volatile uint32_t *CCRx = NULL;
    GPIO_TypeDef *GPIOx = NULL;
    uint32_t GPIO_Pinx = 0, pinState = 0;

    if (duty > 100)duty = 100;
    else if (duty < 0)duty = 0;

    if (motorID % 2 == 0)
        pinState = dir;
    else
        pinState = !dir;

    if (motorID == 0) {
        CCRx = &TIMx->CCR1;
        GPIOx = Dir2_GPIO_Port;
        GPIO_Pinx = Dir2_Pin;
    } else if (motorID == 1) {
        CCRx = &TIMx->CCR2;
        GPIOx = Dir1_GPIO_Port;
        GPIO_Pinx = Dir1_Pin;
    } else if (motorID == 2) {
        CCRx = &TIMx->CCR3;
        GPIOx = Dir4_GPIO_Port;
        GPIO_Pinx = Dir4_Pin;
    } else if (motorID == 3) {
        CCRx = &TIMx->CCR4;
        GPIOx = Dir3_GPIO_Port;
        GPIO_Pinx = Dir3_Pin;
    } else {
        return -1;
    }

    *CCRx = (uint32_t) ((float) (TIMx->ARR) * duty / 100);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pinx, pinState);

    return 0;
}
