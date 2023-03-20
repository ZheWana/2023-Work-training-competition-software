/**
 * @file filter.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/3/12
  */

#include <math.h>
#include "filter.h"

float Filter_MovingAvgf(MovingFilter_t *fHandle, float input) {

    fHandle->buff[fHandle->index++] = input;
    if (fHandle->index == Filter_Window)
        fHandle->index = 0;

    float sum = 0;
    for (int i = 0; i < Filter_Window; i++) {
        sum += fHandle->buff[i];
    }
    return sum / Filter_Window;
}

/**
 * @brief 数据平滑滤波器
 *
 * @param curData 当前数据
 * @param preData 上次数据
 * @param threshold 滤波阈值
 * @return float 当前滤波后数据
 */
float Filter_Smoothing(float curData, float *preData, float threshold) {
    if (*preData == 0) {
        return curData;
    } else {

        if (fabsf(curData - *preData) > threshold) {
            return *preData;
        } else {
            *preData = curData;
            return curData;
        }
    }
}