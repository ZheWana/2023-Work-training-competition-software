/**
 * @file ICM42605.h
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/5/10
  */
#ifndef _ICM42605_H
#define _ICM42605_H

#include "stdint.h"
#include "gpio.h"
#include "spi.h"

#define CS_PORT SPI3_CS_GPIO_Port
#define CS_PIN SPI3_CS_Pin
#define CS_HIGH HAL_GPIO_WritePin(CS_PORT,CS_PIN,1)
#define CS_LOW HAL_GPIO_WritePin(CS_PORT,CS_PIN,0)
#define USE_SPI hspi3

#define ICM42605_DEVICE_CONFIG               (0x11)
#define ICM42605_DRIVE_CONFIG                (0x13)
#define ICM42605_ACCEL_DATA_X1               (0x1F)
#define ICM42605_ACCEL_DATA_X0               (0x20)
#define ICM42605_ACCEL_DATA_Y1               (0x21)
#define ICM42605_ACCEL_DATA_Y0               (0x22)
#define ICM42605_ACCEL_DATA_Z1               (0x23)
#define ICM42605_ACCEL_DATA_Z0               (0x24)
#define ICM42605_GYRO_DATA_X1                (0x25)
#define ICM42605_GYRO_DATA_X0                (0x26)
#define ICM42605_GYRO_DATA_Y1                (0x27)
#define ICM42605_GYRO_DATA_Y0                (0x28)
#define ICM42605_GYRO_DATA_Z1                (0x29)
#define ICM42605_GYRO_DATA_Z0                (0x2A)
#define ICM42605_TEMP_DATA1                  (0x1D)
#define ICM42605_TEMP_DATA0                  (0x1E)
#define ICM42605_PWR_MGMT0                   (0x4E)
#define ICM42605_GYRO_CONFIG0                (0x4F)
#define ICM42605_ACCEL_CONFIG0               (0x50)
#define ICM42605_GYRO_CONFIG1                (0x51)
#define ICM42605_GYRO_ACCEL_CONFIG0          (0x52)
#define ICM42605_ACCEL_CONFIG1               (0x53)
#define ICM42605_WHO_AM_I                    (0x75)
#define ICM42605_BANK_SEL                    (0x76)
#define ICM42605_INTF_CONFIG4                (0x7A)

#define ICM_MODE_ACC                         (1<<0)
#define ICM_MODE_GYRO                        (1<<1)
#define ICM_MODE_TEMP                        (1<<2)

typedef struct icmData_Typedef {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} icmData_t;

uint8_t spi_write_byte(uint8_t reg, uint8_t data);

uint8_t spi_read_byte(uint8_t reg);

uint8_t spi_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t len);

uint8_t spi_write_buffer(uint8_t reg, uint8_t *buffer, uint8_t len);

uint8_t ICM42605_Init(void);

uint8_t ICM42605_GetData(icmData_t *icm, uint8_t MODE);

#endif //_ICM42605_H
