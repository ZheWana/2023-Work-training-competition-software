#ifndef __PMW3901_H
#define __PMW3901_H

#include "spi.h"
#include "main.h"

#define PMW3901_REG_Product_ID      0x00
#define PMW3901_REG_Revision_ID     0x01
#define PMW3901_REG_Motion          0x02
#define PMW3901_REG_Delta_X_L       0x03
#define PMW3901_REG_Delta_X_H       0x04
#define PMW3901_REG_Delta_Y_L       0x05
#define PMW3901_REG_Delta_Y_H       0x06
#define PMW3901_REG_Squal           0x07
#define PMW3901_REG_RawData_Sum     0x08
#define PMW3901_REG_RawData_Max     0x09
#define PMW3901_REG_RawData_Min_    0x0A
#define PMW3901_REG_Shutter_Lower   0x0B
#define PMW3901_REG_Shutter_Upper   0x0C
#define PMW3901_REG_Observation     0x15
#define PMW3901_REG_Motion_Burst    0x16
#define PMW3901_REG_Power_Up_Reset  0x3A
#define PMW3901_REG_Shutdown        0x3B
#define PMW3901_REG_RawData_Grab    0x58
#define PMW3901_REG_RawData_Grab_Status     0x59
#define PMW3901_REG_Inverse_Product_ID      0x5F

typedef struct PMW3901 {
    uint8_t isInited;

    uint8_t Motion;
    int16_t deltaX;
    int16_t deltaY;
} PMW3901;

uint8_t PMW3901_Init(PMW3901 *pmw3901);
uint8_t PMW3901_Read_Data(PMW3901 *pmw3901);

#endif