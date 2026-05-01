#ifndef __KEY_H
#define __KEY_H
#include "stm32f4xx_hal.h"
typedef struct KEY_Driver KEY_Driver;
struct KEY_Driver
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
};
uint8_t Key_Scan(KEY_Driver *key,uint8_t num);
KEY_Driver Key_Create(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
#endif
