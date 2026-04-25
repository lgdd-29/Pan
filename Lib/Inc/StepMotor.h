#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "stm32f4xx_hal.h"

typedef struct StepMotor_Driver StepMotor_Driver;
typedef struct Motor_fun Motor_fun;
typedef struct Motor_var Motor_var;
typedef struct Motor_per Motor_per;
typedef struct Motor_setval Motor_setval;

struct Motor_fun{
    void (*Set_Zero)(StepMotor_Driver *driver);
    void (*Init)(StepMotor_Driver *driver);
    void (*Move)(StepMotor_Driver *driver, uint8_t dir, uint16_t val);
    void (*Stop)(StepMotor_Driver *driver);
};

struct Motor_var{

};

struct Motor_per{

};
struct Motor_setval{
    UART_HandleTypeDef *huart;
    uint8_t adder;
};


struct StepMotor_Driver {
    Motor_fun *fun;
    Motor_var var;
    Motor_per *per;
    Motor_setval setval;
}; 

StepMotor_Driver* StepMotor_Create(UART_HandleTypeDef *huart,uint8_t adder);

#endif /* __STEPMOTOR_H */
