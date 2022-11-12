/**
 * @file HMC5883L.h
 * @brief 
 * @author ZheWana
 * @date 2022/3/22 022
 */

#ifndef _QMC5883L_H
#define _QMC5883L_H

#include "i2c.h"

#define QMC5883_Addr            (0x0D<<1)

#define QMC_DataXLSB            (0x00)
#define QMC_DataXHSB            (0x01)
#define QMC_DataYLSB            (0x02)
#define QMC_DataYHSB            (0x03)
#define QMC_DataZLSB            (0x04)
#define QMC_DataZHSB            (0x05)
#define QMC_StatusReg           (0x06)
#define QMC_TemperatureLSB      (0x07)
#define QMC_TemperatureHSB      (0x08)
#define QMC_ControlReg1         (0x09)
#define QMC_ControlReg2         (0x0A)
#define QMC_SETorRESETReg       (0x0B)
#define QMC_IDReg               (0x0D)

typedef struct {
    float Mx, My, Mz;
} hmcData_t;

int QMC5883_Init(void);

int QMC5883_GetData(hmcData_t *data);

#endif //_QMC5883L_H
