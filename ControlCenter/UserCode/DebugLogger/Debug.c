/**
 * @file Debug.c
 * @brief Debug相关函数定义
 * @author ZheWana
 * @date 2021/9/30
 */

#include "Debug.h"

#ifdef USE_HAL_DRIVER
#ifndef _PRINTF_H_
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    int length;
    char buffer[128];

    length = vsnprintf(buffer, 128, fmt, ap);

    HAL_UART_Transmit(huart, (uint8_t *) buffer, length, HAL_MAX_DELAY);

    va_end(ap);
    return length;
}
#endif
#endif//USE_HAL_DRIVER

#ifdef DeviceFamily_TM4C12x
//int UART_printf(const char *fmt, ...) {
//    va_list ap;
//    va_start(ap, fmt);
//    int length;
//    char buffer[256];
//    length = vsnprintf(buffer, 256, fmt, ap);
//    for (int i = 0; i < length; i++)
//        UARTCharPut(UART0_BASE, buffer[i]);
//    va_end(ap);
//    return length;
//}
#endif//DeviceFamily_TM4C12x