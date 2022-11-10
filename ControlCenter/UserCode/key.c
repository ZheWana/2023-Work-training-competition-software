/**
 * @file key.c
 * @author ZheWana(zhewana.cn)
 * @brief 
 * @date 2022/11/9
  */
#include "key.h"
#include "CommonKey/comKey.h"

comkey_t testKey;

void ComKey_SyncValue(comkey_t *key) {
    // if your key was pressed,"key->val" should be 1.
    if (key == &testKey) {
        key->val = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    }

    // IMPORTANT！！！DO NOT MODIFIED!!!
    key->preVal = key->val;
}

void ComKey_FirstLongTriggerCallback(comkey_t *key) {
    if (key == &testKey) {

    }
}
