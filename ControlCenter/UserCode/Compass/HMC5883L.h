/**
 * @file HMC5883L.h
 * @brief 
 * @author ZheWana
 * @date 2022/3/22 022
 */

#ifndef _HMC5883L_H
#define _HMC5883L_H

#include "i2c.h"

#define HMC5883_Addr 0x3C

#define HMC_CFGA 0x00
#define HMC_CFGB 0x01
#define HMC_MDOEReg 0x02
#define HMC_DataXHSB 0x03
#define HMC_DataXLSB 0x04
#define HMC_DataYHSB 0x05
#define HMC_DataYLSB 0x06
#define HMC_DataZHSB 0x07
#define HMC_DataZLSB 0x08
#define HMC_StatusReg 0x09
#define HMC_IDRegA 0x0A
#define HMC_IDRegB 0x0B
#define HMC_IDRegC 0x0C

typedef struct {
    float Mx, My, Mz;
} hmcData_t;

int HMC5883_Init(void);

int HMC5883_GetData(hmcData_t *data);

#endif //_HMC5883L_H
