#include "PMW3901.h"
#include "printf.h"
#include "usart.h"
#include "./SPI_Port.h"

uint8_t PMW3901_Init(PMW3901 *pmw3901) {
    pmw3901->isInited = 0;
    uint8_t Data = 0;
    uint8_t res = 0;

    CS_L;
    HAL_Delay(100);
    CS_H;
    HAL_Delay(100);

    if (MY_SPI_Write_Byte(PMW3901_REG_Power_Up_Reset, 0x5A))
        return 1;
    HAL_Delay(100);
    if (MY_SPI_Read_Byte(PMW3901_REG_Product_ID, &Data) || Data != 0x49) {
        printf("0x%x\n", Data);
        return 1;
    }
    if (MY_SPI_Read_Byte(PMW3901_REG_Inverse_Product_ID, &Data) || Data != 0xB6) {
        printf("0x%x\n", Data);
        return 1;
    }
    PMW3901_Read_Data(pmw3901);
    MY_SPI_Write_Byte(0x7F, 0x00);
    MY_SPI_Write_Byte(0x61, 0xAD);
    MY_SPI_Write_Byte(0x7F, 0x03);
    MY_SPI_Write_Byte(0x40, 0x00);
    MY_SPI_Write_Byte(0x7F, 0x05);
    MY_SPI_Write_Byte(0x41, 0xB3);
    MY_SPI_Write_Byte(0x43, 0xF1);
    MY_SPI_Write_Byte(0x45, 0x14);
    MY_SPI_Write_Byte(0x5B, 0x32);
    MY_SPI_Write_Byte(0x5F, 0x34);
    MY_SPI_Write_Byte(0x7B, 0x08);
    MY_SPI_Write_Byte(0x7F, 0x06);
    MY_SPI_Write_Byte(0x44, 0x1B);
    MY_SPI_Write_Byte(0x40, 0xBF);
    MY_SPI_Write_Byte(0x4E, 0x3F);
    MY_SPI_Write_Byte(0x7F, 0x08);
    MY_SPI_Write_Byte(0x65, 0x20);
    MY_SPI_Write_Byte(0x6A, 0x18);
    MY_SPI_Write_Byte(0x7F, 0x09);
    MY_SPI_Write_Byte(0x4F, 0xAF);
    MY_SPI_Write_Byte(0x5F, 0x40);
    MY_SPI_Write_Byte(0x48, 0x80);
    MY_SPI_Write_Byte(0x49, 0x80);
    MY_SPI_Write_Byte(0x57, 0x77);
    MY_SPI_Write_Byte(0x60, 0x78);
    MY_SPI_Write_Byte(0x61, 0x78);
    MY_SPI_Write_Byte(0x62, 0x08);
    MY_SPI_Write_Byte(0x63, 0x50);
    MY_SPI_Write_Byte(0x7F, 0x0A);
    MY_SPI_Write_Byte(0x45, 0x60);
    MY_SPI_Write_Byte(0x7F, 0x00);
    MY_SPI_Write_Byte(0x4D, 0x11);
    MY_SPI_Write_Byte(0x55, 0x80);
    MY_SPI_Write_Byte(0x74, 0x1F);
    MY_SPI_Write_Byte(0x75, 0x1F);
    MY_SPI_Write_Byte(0x4A, 0x78);
    MY_SPI_Write_Byte(0x4B, 0x78);
    MY_SPI_Write_Byte(0x44, 0x08);
    MY_SPI_Write_Byte(0x45, 0x50);
    MY_SPI_Write_Byte(0x64, 0xFF);
    MY_SPI_Write_Byte(0x65, 0x1F);
    MY_SPI_Write_Byte(0x7F, 0x14);
    MY_SPI_Write_Byte(0x65, 0x60);
    MY_SPI_Write_Byte(0x66, 0x08);
    MY_SPI_Write_Byte(0x63, 0x78);
    MY_SPI_Write_Byte(0x7F, 0x15);
    MY_SPI_Write_Byte(0x48, 0x58);
    MY_SPI_Write_Byte(0x7F, 0x07);
    MY_SPI_Write_Byte(0x41, 0x0D);
    MY_SPI_Write_Byte(0x43, 0x14);
    MY_SPI_Write_Byte(0x4B, 0x0E);
    MY_SPI_Write_Byte(0x45, 0x0F);
    MY_SPI_Write_Byte(0x44, 0x42);
    MY_SPI_Write_Byte(0x4C, 0x80);
    MY_SPI_Write_Byte(0x7F, 0x10);
    MY_SPI_Write_Byte(0x5B, 0x02);
    MY_SPI_Write_Byte(0x7F, 0x07);
    MY_SPI_Write_Byte(0x40, 0x41);
    MY_SPI_Write_Byte(0x70, 0x00);
    HAL_Delay(10);
    MY_SPI_Write_Byte(0x32, 0x44);
    MY_SPI_Write_Byte(0x7F, 0x07);
    MY_SPI_Write_Byte(0x40, 0x40);
    MY_SPI_Write_Byte(0x7F, 0x06);
    MY_SPI_Write_Byte(0x62, 0xf0);
    MY_SPI_Write_Byte(0x63, 0x00);
    MY_SPI_Write_Byte(0x7F, 0x0D);
    MY_SPI_Write_Byte(0x48, 0xC0);
    MY_SPI_Write_Byte(0x6F, 0xd5);
    MY_SPI_Write_Byte(0x7F, 0x00);
    MY_SPI_Write_Byte(0x5B, 0xa0);
    MY_SPI_Write_Byte(0x4E, 0xA8);
    MY_SPI_Write_Byte(0x5A, 0x50);
    MY_SPI_Write_Byte(0x40, 0x80);


    pmw3901->isInited = 1;
    return 0;
}

uint8_t PMW3901_Read_Data(PMW3901 *pmw3901) {
    if (pmw3901->isInited) {
        uint8_t Data[4] = {0, 0, 0, 0};
        MY_SPI_Read_Byte(PMW3901_REG_Motion, &Data[0]);
        MY_SPI_Read_Len(PMW3901_REG_Delta_X_L, 4, Data);

        pmw3901->deltaX = (int16_t) ((Data[1] << 8) | Data[0]);
        pmw3901->deltaY = (int16_t) ((Data[3] << 8) | Data[2]);
    } else {
        return 1;
    }
    return 0;
}