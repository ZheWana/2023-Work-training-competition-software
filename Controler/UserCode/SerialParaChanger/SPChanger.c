/**
 * @file SPChanger.c
 * @author ZheWana(zhewana.cn)
 * @brief 使用总线的参数修改器
 * @date 2023/1/5
  */
#include <stdint.h>
#include "SPChanger.h"
#include "string.h"


static spChanger_t *head = NULL, *tail = NULL;

void SPC_ParaRegister(spChanger_t *instance, const char *paraDescription, float *paraPtr) {
    for (int i = 0; i < 6 || paraDescription[i] != '\0'; i++)
        instance->pdStr[i] = paraDescription[i];
    instance->paraPtr = paraPtr;
    instance->isInvalid = 0;
    instance->iPart = 0;
    instance->dPart = 0;
    instance->counter = 0;
    instance->next = NULL;

    if (head == NULL) {
        head = instance;
        tail = instance;
    } else {
        tail->next = instance;
        tail = tail->next;
    }
}

// 构建简易环形缓冲区:Head入队，Tail出队
struct ringBuffTypedef {
    char ringBuff[128], *buffHead, *buffTail;
    uint32_t chToHandle;
} ringBuffBlock = {
        .chToHandle = 0,
        .buffHead= ringBuffBlock.ringBuff,
        .buffTail = ringBuffBlock.ringBuff,
};

void SPC_GetChar(char ch) {
    *ringBuffBlock.buffHead++ = ch;
    ringBuffBlock.chToHandle++;
    if (ringBuffBlock.buffHead == (ringBuffBlock.ringBuff + 128))ringBuffBlock.buffHead = ringBuffBlock.ringBuff;
}

void SPC_Handler() {
    static enum {
        CheckProtocol,
        GetInteger,
        GetDecimal,
        GetFloat
    } State = CheckProtocol;

    if (ringBuffBlock.chToHandle == 0) return;
    ringBuffBlock.chToHandle--;

    if (ringBuffBlock.buffTail == ringBuffBlock.buffHead)return;

    char ch = *ringBuffBlock.buffTail++;
    if (ringBuffBlock.buffTail == (ringBuffBlock.ringBuff + 128))ringBuffBlock.buffTail = ringBuffBlock.ringBuff;

    if (State == CheckProtocol) { // 验证协议头
        CheckProtocolLable:
        for (spChanger_t *index = head; index != NULL; index = index->next) {
            if (index->isInvalid)continue;
            if (ch <= '9' && ch >= '0') {
                State = GetInteger;
                goto GetIntegerLable;
            }
            if (ch == ' ') {
                if (index->counter > 0)
                    State++;
                continue;
            }
            if (index->pdStr[index->counter++] != ch)index->isInvalid = 1;
        }
    } else if (State != GetFloat) { // 提取整数部分
        GetIntegerLable:
        for (spChanger_t *index = head; index != NULL; index = index->next) {
            if (index->isInvalid)continue;
            if (ch <= '9' && ch >= '0') {
                if (State == GetInteger)
                    index->iPart = index->iPart * 10 + ch - '0';
                else if (State == GetDecimal) {
                    index->dPart = index->dPart * 10 + ch - '0';
                    index->counter *= 10;
                }

                if (ringBuffBlock.buffHead == ringBuffBlock.buffTail) {
                    ch = ' ';
                    goto GetFloatLable;
                }
            } else if (ch == '.') {
                State = GetDecimal;
                index->counter = 1;
            } else {
                State = GetFloat;
                goto GetFloatLable;
            }
        }
    } else if (State == GetFloat) {
        GetFloatLable:
        for (spChanger_t *index = head; index != NULL; index = index->next) {

            if (!index->isInvalid)
                *index->paraPtr = index->iPart + index->dPart * 1.0f / index->counter;
            index->iPart = index->dPart = State = index->counter = index->isInvalid = 0;
        }
        if (ch != ' ')goto CheckProtocolLable;
    }
}
