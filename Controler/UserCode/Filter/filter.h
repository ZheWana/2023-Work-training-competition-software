/**
 * @file filter.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/3/12
  */
#ifndef FILTER_H
#define FILTER_H

#define Filter_Window (100)

typedef struct MovingFilterTypedef {
    float buff[Filter_Window];
    int index;
} MovingFilter_t;

float Filter_MovingAvgf(MovingFilter_t *fHandle, float input);

float Filter_Smoothing(float curData, float *preData, float threshold);

#endif //FILTER_H
