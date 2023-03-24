/**
 * @file key.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/9
  */
#include "key.h"
#include "CommonKey/comKey.h"
#include "printf.h"
#include "PID/pid.h"
#include "utils.h"

comkey_t keys[3];

void ComKey_SyncValue(comkey_t *key) {
    // if your key was pressed,"key->val" should be 1.
    if (key == &keys[0]) {
        key->val = !HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
    }
    if (key == &keys[1]) {
        key->val = !HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    }
    if (key == &keys[2]) {
        key->val = !HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
    }

    // IMPORTANT！！！DO NOT MODIFIED!!!
    key->preVal = key->val;
}

void KeyInit(void) {
    for (int i = 0; i < 3; i++) { ComKey_Init(&keys[i], Key_PollingPeriod); }
}

void ComKey_FirstLongTriggerCallback(comkey_t *key) {
}

void ComKey_MultipleClickCallback(comkey_t *key) {
    if (key == &keys[2]) {
        static uint8_t record = 0;
        if (record == 0) {
            record = 1;
            for (int i = 0; i < 4; i++) {
                PID_Init(&CarInfo.msPid[i], 8, 0.45f, 0);//0, 0.2, 0,
                PID_Init(&CarInfo.mpPid[i], 0.15f, 0, 0);// 0.033, 0, 0,
            }
            PID_Init(&CarInfo.cpPidX, 0.1f, 0, 0);
            PID_Init(&CarInfo.cpPidY, 0.1f, 0, 0);
            PID_Init(&CarInfo.avPid, 0.2f, 0.005f, 0);
            PID_Init(&CarInfo.aPid, 150, 0, 0);
        } else {
            record = 0;
            for (int i = 0; i < 4; i++) {
                PID_Init(&CarInfo.msPid[i], 0, 0, 0);
                PID_Init(&CarInfo.mpPid[i], 0, 0, 0);
                PID_Reset(&CarInfo.msPid[i]);
                PID_Reset(&CarInfo.mpPid[i]);
            }
            PID_Init(&CarInfo.cpPidX, 0, 0, 0);
            PID_Init(&CarInfo.cpPidY, 0, 0, 0);
            PID_Init(&CarInfo.avPid, 0, 0, 0);
            PID_Init(&CarInfo.aPid, 0, 0, 0);
            PID_Reset(&CarInfo.cpPidX);
            PID_Reset(&CarInfo.cpPidY);
            PID_Reset(&CarInfo.avPid);
            PID_Reset(&CarInfo.aPid);
        }
    } else if (key == &keys[1]) {
        CarInfo.Pi_Reset = 1;
    } else if (key == &keys[0]) {
        CarInfo.Start_State = 1;
    }
}