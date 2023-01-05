#ifndef __SPI_PORT_H
#define __SPI_PORT_H

#include "stdint.h"
#include "main.h"
#include "spi.h"

#define USE_SPI hspi1

#ifndef PMW3901_CS_GPIO_Port
#define PMW3901_CS_GPIO_Port GPIOA
#endif
#ifndef PMW3901_CS_Pin
#define PMW3901_CS_Pin GPIO_PIN_1
#endif

#define CS_L    HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port,PMW3901_CS_Pin,GPIO_PIN_RESET)
#define CS_H    HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port,PMW3901_CS_Pin,GPIO_PIN_SET)

uint8_t MY_SPI_Write_Len(uint8_t reg, uint8_t len, uint8_t *data);

uint8_t MY_SPI_Read_Len(uint8_t reg, uint8_t len, uint8_t *data);

uint8_t MY_SPI_Write_Byte(uint8_t reg, uint8_t data);

uint8_t MY_SPI_Read_Byte(uint8_t reg, uint8_t *data);

#endif//__SPI_PORT_H