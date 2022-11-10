/**
 * @file HMC5883L.c
 * @brief 
 * @author ZheWana
 * @date 2022/3/22 022
 */

#include "HMC5883L.h"

int HMC5883_Init(void) {
    char check, data;
    // 检查身份标识
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883_Addr, HMC_IDRegA, 1, &check, 1, 0xffff))
        return -1;
    if (check == 'H') {
        //配置持续测量模式
        data = 0x00;
        if (HAL_I2C_Mem_Read(&hi2c1, HMC5883_Addr, HMC_MDOEReg, 1, &data, 1, 1000))
            return -1;
    } else { return -2; }
    return 0;
}

int HMC5883_GetData(hmcData_t *data) {
    uint8_t temp[6];
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883_Addr, HMC_DataXHSB, 1, (uint8_t *) &temp, 6, 0xffff))
        return -1;
    data->Mx = (float) (int16_t) (temp[0] << 8 | temp[1]);
    data->Mz = (float) (int16_t) (temp[2] << 8 | temp[3]);
    data->My = (float) (int16_t) (temp[4] << 8 | temp[5]);
    return 0;
}
