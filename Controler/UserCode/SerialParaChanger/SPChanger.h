/**
 * @file SPChanger.h
 * @author ZheWana(zhewana.cn)
 * @brief 使用总线的参数修改器
 * @date 2023/1/5
  */
#ifndef SPCHANGER_H
#define SPCHANGER_H

typedef struct SPC_TypeDef spChanger_t;
struct SPC_TypeDef {
    char pdStr[6]; // Parameter description String
    int iPart; // Integer part
    int dPart; // Decimal part

    int counter;

    float *paraPtr; // Parameter pointer

    char isInvalid;
    spChanger_t *next;
};

void SPC_ParaRegister(spChanger_t *instance, const char *paraDescription, float *paraPtr);

void SPC_GetChar(char ch);

void SPC_Handler();


#endif //SPCHANGER_H
