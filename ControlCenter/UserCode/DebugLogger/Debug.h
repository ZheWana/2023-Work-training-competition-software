/**
 * @file Debug.h
 * @brief Debug相关宏函数
 * @author ZheWana
 * @date 2021/9/30 003
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "printf.h"
#include "stdarg.h"

///////////////////////////////////////////    Log Config    ///////////////////////////////////////////
//#define LOG_LEVEL_NONE      1
#if !LOG_LEVEL_NONE
#define LOG_LEVEL_DATA      1
#define LOG_LEVEL_DEBUG     2
#define LOG_LEVEL_INFO      3
#define LOG_LEVEL_WARN      4
#define LOG_LEVEL_ERROR     5
#endif
/////////////////////////////////////////    Log Config    /////////////////////////////////////////////
#ifndef _PRINTF_H_
#ifdef USE_HAL_DRIVER
#include "stdio.h"
#include "usart.h"
#define DEBUG_UART_HANDLE huart5
int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
#endif//USE_HAL_DRIVER

#ifdef DeviceFamily_TM4C12x
#include "inc/headfile.h"
#include "utils/uartstdio.h"
int UART_printf(const char *fmt, ...);
#endif//DeviceFamily_TM4C12x
#endif

#define ConditionLog(__level, __condition, __msg, ...) do{if(__condition){  \
switch(__level){                                                            \
case LOG_LEVEL_DEBUG:DebugLog(__msg,##__VA_ARGS__);break;                   \
case LOG_LEVEL_INFO:InfoLog(__msg,##__VA_ARGS__);break;                     \
case LOG_LEVEL_WARN:WarnLog(__msg,##__VA_ARGS__);break;                     \
case LOG_LEVEL_ERROR:ErrorLog(__msg,##__VA_ARGS__);break;                   \
}}}while(0)

#define DebugConLog(__condition, __dbgmsg, ...) ConditionLog(LOG_LEVEL_DEBUG,__condition,__dbgmsg,##__VA_ARGS__)
#define InfoConLog(__condition, __infomsg, ...) ConditionLog(LOG_LEVEL_INFO,__condition,__infomsg,##__VA_ARGS__)
#define WarnConLog(__condition, __warnmsg, ...) ConditionLog(LOG_LEVEL_WARN,__condition,__warnmsg,##__VA_ARGS__)
#define ErrorConLog(__condition, __errmsg, ...) ConditionLog(LOG_LEVEL_ERROR,__condition,__errmsg,##__VA_ARGS__)

/*******************************************************************************************************************************************/
#ifdef USE_HAL_DRIVER

#if (LOG_LEVEL_DATA)
#define DataLog(__datamsg, ...) printf(__datamsg,##__VA_ARGS__)
#else
#define DataLog(...) do{}while(0)
#endif//(LOG_LEVEL_DATA)
#if (LOG_LEVEL_DEBUG) != 0
#define DebugLog(__dbgmsg, ...) printf("[Debug]:"__dbgmsg"\n",##__VA_ARGS__)
#else
#define DebugLog(...) do{}while(0)
#endif//(LOG_LEVEL_DEBUG)
#if (LOG_LEVEL_INFO)
#define InfoLog(__infomsg, ...) printf("[INFO][%s]:"__infomsg"\n",__FUNCTION__,##__VA_ARGS__)
#else
#define InfoLog(...) do{}while(0)
#endif//(LOG_LEVEL_INFO)
#if (LOG_LEVEL_WARN)
#define WarnLog(__warnmsg, ...) printf("[WARNING]["__FILE__"]\\n""\n[Func:%s][Line:%d]:"__warnmsg,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define WarnLog(...) do{}while(0)
#endif//(LOG_LEVEL_WARN)
#if (LOG_LEVEL_ERROR)
#define ErrorLog(__errmsg, ...) printf("[ERROR]["__FILE__"]\\n""\n[Func:%s][Line:%d]:"__errmsg,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define ErrorLog(...) do{}while(0)
#endif//(LOG_LEVEL_ERROR)

#endif//USE_HAL_DRIVER

#ifdef DeviceFamily_TM4C12x

#if (LOG_LEVEL& LOG_LEVEL_DATA)
#define DataLog(__datamsg, ...) UART_printf(__datamsg,##__VA_ARGS__)
#else
#endif//(LOG_LEVEL_DATA)
#define DataLog(...) do{}while(0)
#if (LOG_LEVEL& LOG_LEVEL_DEBUG)
#define DebugLog(__dbgmsg, ...) UART_printf("[Debug]:"__dbgmsg,##__VA_ARGS__)
#else
#define DebugLog(...) do{}while(0)
#endif//(LOG_LEVEL_DEBUG)
#if (LOG_LEVEL& LOG_LEVEL_INFO)
#define InfoLog(__infomsg, ...) UART_printf("[INFO][%s]:"__infomsg,__FUNCTION__,##__VA_ARGS__)
#else
#define InfoLog(...) do{}while(0)
#endif//(LOG_LEVEL_INFO)
#if (LOG_LEVEL& LOG_LEVEL_WARN)
#define WarnLog(__warnmsg, ...) UART_printf("[WARNING]["__FILE__"]\\n""\n[Func:%s][Line:%d]:"__warnmsg,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define WarnLog(...) do{}while(0)
#endif//(LOG_LEVEL_WARN)
#if (LOG_LEVEL& LOG_LEVEL_ERROR)
#define ErrorLog(__errmsg, ...) UART_printf("[ERROR]["__FILE__"]\\n""\n[Func:%s][Line:%d]:"__errmsg,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define ErrorLog(...) do{}while(0)
#endif//(LOG_LEVEL_ERROR)

#endif//DeviceFamily_TM4C12x

#endif //DEBUG_H
