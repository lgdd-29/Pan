#include "Key.h"
#include "stdlib.h"
#include <stdint.h>
uint8_t Key_Scan(KEY_Driver *key,uint8_t num)
{
    for(uint8_t i = 0; i < num; i++)
    {
        if(HAL_GPIO_ReadPin(key[i].GPIOx, key[i].GPIO_Pin) == GPIO_PIN_RESET)
        {
            uint16_t time = 0;
            while(HAL_GPIO_ReadPin(key[i].GPIOx, key[i].GPIO_Pin) == GPIO_PIN_RESET)
            {
                time++;
                HAL_Delay(10);
                if(time>200) break; // 长按超过2秒，退出循环
            }
            HAL_Delay(10); // 消抖
            if(time>50) return key->num=2*(i+1); // 长按返回编号+num，区分短按长按
            return 2*(i+1)-1;
        }
    }
    return 0;
}

KEY_Driver* Key_Create(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    KEY_Driver* key=(KEY_Driver*)malloc(sizeof(KEY_Driver));
    key->Key_Scan=Key_Scan;
    key->GPIOx = GPIOx;
    key->GPIO_Pin = GPIO_Pin;
    key->num=0;
    return key;
}