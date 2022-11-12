/**
 * @file HMC5883L.c
 * @brief 
 * @author ZheWana
 * @date 2022/3/22 022
 */

#include "QMC5883L.h"

int QMC5883_Init(void) {
    char check, data;
    // 检查身份标识
    if (HAL_I2C_Mem_Read(&hi2c1, QMC5883_Addr, QMC_IDReg, 1, (uint8_t *) &check, 1, 0xffff))
        return -1;
    if (check == 0xFF) {
        data = 0x01;
        if (HAL_I2C_Mem_Read(&hi2c1, QMC5883_Addr, QMC_SETorRESETReg, 1, (uint8_t *) &data, 1, 1000))
            return -1;
        //配置持续测量模式
        data = 0x1D;
        if (HAL_I2C_Mem_Read(&hi2c1, QMC5883_Addr, QMC_ControlReg1, 1, (uint8_t *) &data, 1, 1000))
            return -1;
    } else { return -2; }
    return 0;
}

int QMC5883_GetData(hmcData_t *data) {
    uint8_t temp[6];
    if (HAL_I2C_Mem_Read(&hi2c1, QMC5883_Addr, QMC_DataXLSB, 1, (uint8_t *) &temp, 6, 0xffff))
        return -1;
    data->Mx = (float) (int16_t) (temp[0] << 8 | temp[1]);
    data->Mz = (float) (int16_t) (temp[2] << 8 | temp[3]);
    data->My = (float) (int16_t) (temp[4] << 8 | temp[5]);
    return 0;
}
