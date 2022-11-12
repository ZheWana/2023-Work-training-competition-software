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
        if (HAL_I2C_Mem_Write(&hi2c1, QMC5883_Addr, QMC_SETorRESETReg, 1, (uint8_t *) &data, 1, 1000))
            return -1;
        data = 0x1D;
        if (HAL_I2C_Mem_Write(&hi2c1, QMC5883_Addr, QMC_ControlReg1, 1, (uint8_t *) &data, 1, 1000))
            return -1;
    } else { return -2; }
    return 0;
}

int QMC5883_GetData(hmcData_t *data) {
    uint8_t temp[6];
    if (HAL_I2C_Mem_Read(&hi2c1, QMC5883_Addr, QMC_DataXLSB, 1, (uint8_t *) &temp, 6, HAL_MAX_DELAY))
        return -1;
    data->Mx = (float) (int16_t) (temp[1] << 8 | temp[0]);
    data->Mz = (float) (int16_t) (temp[3] << 8 | temp[2]);
    data->My = (float) (int16_t) (temp[5] << 8 | temp[4]);
    return 0;
}
