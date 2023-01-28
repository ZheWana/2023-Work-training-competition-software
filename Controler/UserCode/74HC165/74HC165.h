#ifndef __74HC165_H
#define __74HC165_H

#include "spi.h"
#include "main.h"
#include "usart.h"

//#define USE_SPI//LSB first

#define LD_L HAL_GPIO_WritePin(LD_GPIO_Port,LD_Pin,GPIO_PIN_RESET)
#define LD_H HAL_GPIO_WritePin(LD_GPIO_Port,LD_Pin,GPIO_PIN_SET)

#ifndef USE_SPI
#define CLK_L HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET)
#define CLK_H HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET)
#define SER_R HAL_GPIO_ReadPin(SER_GPIO_Port,SER_Pin)
#endif

uint8_t HC165_Get_Data(uint32_t *Data);

#endif