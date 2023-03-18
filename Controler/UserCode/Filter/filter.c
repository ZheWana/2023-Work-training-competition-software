/**
 * @file filter.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/3/12
  */
#include "filter.h"

float Filter_MovingAvgf(FilterTypedef_t *fHandle, float input) {

    fHandle->buff[fHandle->index++] = input;
    if (fHandle->index == Filter_Window)
        fHandle->index = 0;

    float sum = 0;
    for (int i = 0; i < Filter_Window; i++) {
        sum += fHandle->buff[i];
    }
    return sum / Filter_Window;
}