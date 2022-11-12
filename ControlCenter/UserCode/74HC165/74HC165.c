#include "74HC165.h"

extern int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...);

uint8_t Form[16] = {8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7};

uint8_t HC165_Get_Data(uint16_t *Data) {
#ifdef USE_SPI
    LD_L;
    LD_H;
    uint8_t Rd_Data[2];
    HAL_SPI_Receive(&hspi1, Rd_Data, 2, 0xF);
    *Data = Rd_Data[1] << 8 | Rd_Data[0];
#else
    uint8_t Data_1[16];
    uint16_t Data_2 = 0;
    LD_L;
    LD_H;
    for (uint8_t i = 0; i < 16; i++) {
        if (SER_R == GPIO_PIN_SET)
            Data_1[i] = 1;
        else
            Data_1[i] = 0;
        Data_2 = Data_2 | (Data_1[i] << Form[i]);
        CLK_L;
        CLK_H;
    }
    *Data = Data_2;
#endif
		return 0;
}