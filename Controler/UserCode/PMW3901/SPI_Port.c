#include "SPI_Port.h"

extern int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);

uint8_t MY_SPI_Write_Len(uint8_t reg, uint8_t len, uint8_t *data) {
    CS_L;
    uint8_t TX = reg;
    if (HAL_SPI_Transmit(&USE_SPI, &TX, 1, 0xFF) != HAL_OK)
        return 1;
    if (HAL_SPI_Transmit(&USE_SPI, data, len, 0xFF) != HAL_OK)
        return 1;
    CS_H;
    return 0;
}

uint8_t MY_SPI_Read_Len(uint8_t reg, uint8_t len, uint8_t *data) {
    CS_L;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t TX = reg + i;
        if (HAL_SPI_Transmit(&USE_SPI, &TX, 1, 0xFF) != HAL_OK)
            return 1;
        if (HAL_SPI_Receive(&USE_SPI, data + i, 1, 0xFF) != HAL_OK)
            return 1;
    }
    CS_H;
    return 0;
}

uint8_t MY_SPI_Write_Byte(uint8_t reg, uint8_t data) {
    CS_L;
    uint8_t TX = reg;
    if (HAL_SPI_Transmit(&USE_SPI, &TX, 1, 0xFF) != HAL_OK)
        return 1;
    if (HAL_SPI_Transmit(&USE_SPI, &data, 1, 0xFF) != HAL_OK)
        return 1;
    CS_H;
    return 0;
}

uint8_t MY_SPI_Read_Byte(uint8_t reg, uint8_t *data) {
    CS_L;
    uint8_t TX = reg;
    uint8_t RX;
    if (HAL_SPI_Transmit(&USE_SPI, &TX, 1, 0xFF) != HAL_OK)
        return 1;
    if (HAL_SPI_Receive(&USE_SPI, &RX, 1, 0xFF) != HAL_OK)
        return 1;
    CS_H;
    *data = RX;
    return 0;
}