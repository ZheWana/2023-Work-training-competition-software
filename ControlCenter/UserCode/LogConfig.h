/**
 * @file LogConfig.h
 * @author ZheWana(zhewana.cn)
 * @brief 配置模块是否使用Log输出日志
 * @date 2022/11/15
  */
#ifndef CONTROLCENTER_LOGCONFIG_H
#define CONTROLCENTER_LOGCONFIG_H

#include "DebugLogger/Debug.h"

#define LogStart(_describe) do{if(!_describe)goto _describe##Lable;}while(0)
#define LogEnd(_describe) _describe##Lable :

//#define DO_NOT_LOG_ALL
#ifndef DO_NOT_LOG_ALL
#define LogAboutQMC5883_GetData         (0)
#define LogAboutQMC5883_CordanateInit   (0)
#define LogAboutInfrared                (0)
#endif//DO_NOT_LOG_ALL


#endif //CONTROLCENTER_LOGCONFIG_H
