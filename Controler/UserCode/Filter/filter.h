/**
 * @file filter.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2023/3/12
  */
#ifndef FILTER_H
#define FILTER_H

#define Filter_Window (100)

typedef struct FilterBuffer {
    float buff[Filter_Window];
    int index;
} FilterTypedef_t;

float Filter_MovingAvgf(FilterTypedef_t *fHandle, float input);

#endif //FILTER_H
