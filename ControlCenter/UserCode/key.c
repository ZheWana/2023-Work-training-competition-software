/**
 * @file key.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/9
  */
#include "key.h"
#include "CommonKey/comKey.h"
#include "DebugLogger/Debug.h"

comkey_t keys[3];

void ComKey_SyncValue(comkey_t *key) {
    // if your key was pressed,"key->val" should be 1.
    if (key == &keys[0]) {
        key->val = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
    }
    if (key == &keys[1]) {
        key->val = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    }
    if (key == &keys[2]) {
        key->val = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
    }

    // IMPORTANT！！！DO NOT MODIFIED!!!
    key->preVal = key->val;
}

void KeyInit(void) {
    for (int i = 0; i < 3; i++) { ComKey_Init(&keys[i], 1); }
}

void ComKey_FirstLongTriggerCallback(comkey_t *key) {
//    for (int i = 0; i < 3; i++) {
//        if (key == &keys[i]) {
//            DebugLog("key%d", i);
//        }
//    }
}

void ComKey_MultipleClickCallback(comkey_t *key) {
    for (int i = 0; i < 3; i++) {
        if (key == &keys[i]) {
            DebugLog("key%d", i);
        }
    }
}