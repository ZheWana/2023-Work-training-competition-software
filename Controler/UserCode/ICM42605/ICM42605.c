/**
 * @file ICM42605.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/5/10
  */
#include "ICM42605.h"
#include "printf.h"

#define reglog(reg)     do{printf(#reg":%x\n", spi_read_byte(reg));HAL_Delay(10);}while(0)


__attribute__((__noinline__)) uint8_t spi_write_byte(uint8_t reg, uint8_t data) {
    CS_LOW;
    uint8_t tx = reg & 0x7f;
    uint8_t rx;
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    tx = data;
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    CS_HIGH;
    return 1;
}

__attribute__((__noinline__)) uint8_t spi_read_byte(uint8_t reg) {
    CS_LOW;
    uint8_t tx = reg | 0x80;
    uint8_t rx;
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    CS_HIGH;
    return rx;
}

uint8_t spi_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t len) {
    CS_LOW;
    uint8_t tx = reg | 0x80;
    uint8_t rx;
    uint8_t tx_buf[14] = {};
    tx_buf[0] = tx;
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    if (HAL_SPI_TransmitReceive(&USE_SPI, tx_buf, buffer, len, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    CS_HIGH;
    return 1;
}

uint8_t spi_write_buffer(uint8_t reg, uint8_t *buffer, uint8_t len) {
    CS_LOW;
    uint8_t tx = reg & 0x7f;
    uint8_t rx;
    uint8_t rx_buf[14] = {};
    if (HAL_SPI_TransmitReceive(&USE_SPI, &tx, &rx, 1, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    if (HAL_SPI_TransmitReceive(&USE_SPI, buffer, rx_buf, len, HAL_MAX_DELAY) != HAL_OK) {
        CS_HIGH;
        return 0;
    }
    CS_HIGH;
    return 1;
}

uint8_t ICM42605_Init(void) {
    uint8_t data[2] = {0};
    //检测
    data[0] = 0;//bank0
    if (spi_write_byte(ICM42605_BANK_SEL, data[0]) == 0)
        return 9;

//    验证设备ID
//    reglog(ICM42605_WHO_AM_I);
    data[0] = spi_read_byte(ICM42605_WHO_AM_I);
    HAL_Delay(10);
    printf("ICM42605_WHO_AM_I:%x\n", data[0]);
    if (data[0] == 0) return 1;

    data[0] = 0;//bank0
    if (spi_write_byte(ICM42605_BANK_SEL, data[0]) == 0)
        return 2;
    HAL_Delay(10);
    data[0] = 0x01;//bank1
    if (spi_write_byte(ICM42605_BANK_SEL, data[0]) == 0)
        return 3;
    HAL_Delay(10);
    data[0] = 0x02;//4wire-spi
    if (spi_write_byte(ICM42605_INTF_CONFIG4, data[0]) == 0)
        return 4;
    HAL_Delay(10);
    reglog(ICM42605_INTF_CONFIG4);
    data[0] = 0;//bank0
    if (spi_write_byte(ICM42605_BANK_SEL, data[0]) == 0)
        return 5;
    HAL_Delay(10);
    //角速度计加速度计配置
    data[0] = 0b00000110;  //2000dps
    if (spi_write_byte(ICM42605_GYRO_CONFIG0, data[0]) == 0)
        return 6;
    HAL_Delay(10);
    reglog(ICM42605_GYRO_CONFIG0);
    data[0] = 0b00000011;   //16g
    if (spi_write_byte(ICM42605_ACCEL_CONFIG0, data[0]) == 0)
        return 7;
    HAL_Delay(10);
    reglog(ICM42605_ACCEL_CONFIG0);
    //电源管理
    data[0] = 0b00011111;
    if (spi_write_byte(ICM42605_PWR_MGMT0, data[0]) == 0)
        return 8;
    HAL_Delay(100);
    reglog(ICM42605_PWR_MGMT0);

    HAL_Delay(1000);

    return 0;
}

uint8_t ICM42605_GetData(icmData_t *icm, uint8_t MODE) {
    int16_t out;
    uint8_t data[6];

    //加速度计
    if (MODE & ICM_MODE_ACC) {
        if (spi_read_buffer(ICM42605_ACCEL_DATA_X1, data, 6) == 0)
            return 1;
        out = (int16_t) (data[0] << 8 | data[1]);
        icm->ax = (float) out * 16 / 32768.0f;
        out = (int16_t) (data[2] << 8 | data[3]);
        icm->ay = (float) out * 16 / 32768.0f;
        out = (int16_t) (data[4] << 8 | data[5]);
        icm->az = (float) out * 16 / 32768.0f;
    }

    //角速度计
    if (MODE & ICM_MODE_GYRO) {
        if (spi_read_buffer(ICM42605_GYRO_DATA_X1, data, 6) == 0)
            return 2;
        out = (int16_t) (data[0] << 8 | data[1]);
        icm->gx = (float) out * 2000.0f / 32768.0f;
        out = (int16_t) (data[2] << 8 | data[3]);
        icm->gy = (float) out * 2000.0f / 32768.0f;
        out = (int16_t) (data[4] << 8 | data[5]);
        icm->gz = (float) out * 2000.0f / 32768.0f;
    }

    //温度计
    if (MODE & ICM_MODE_TEMP) {
        if (spi_read_buffer(ICM42605_TEMP_DATA0, data, 2) == 0)
            return 3;
        out = (int16_t) (data[0] << 8 | data[1]);
        icm->temp = 25.0f + (float) out * 1.0f / 132.48f;
    }

    return 0;
}