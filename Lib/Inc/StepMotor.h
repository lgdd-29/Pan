#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "stm32f4xx_hal.h"

typedef struct StepMotor_Driver StepMotor_Driver;
typedef struct Motor_fun Motor_fun;
typedef struct Motor_var Motor_var;
typedef struct Motor_per Motor_per;
typedef struct Motor_setval Motor_setval;
typedef struct Step_PID Step_PID;
struct Step_PID{
    float Kp;
    float Ki;
    float Kd;
    float now;
    float target;
    float error;
    float err_prev;
    float last_error;
    float integral;
    float out;
};
struct Motor_fun{
    void (*Set_Zero)(StepMotor_Driver *driver);
    void (*Init)(StepMotor_Driver *driver);
    void (*Move)(StepMotor_Driver *driver,uint16_t val);
    void (*Stop)(StepMotor_Driver *driver);
    void (*PID_OUT)(StepMotor_Driver *driver,float target,float now);
    void (*PID_SET)(StepMotor_Driver *driver,float Kp,float Ki,float Kd);
};

struct Motor_var{
    Step_PID pid;
    float Move_max;
};

struct Motor_setval{
    UART_HandleTypeDef *huart;
    uint8_t adder;
};


struct StepMotor_Driver {
    Motor_fun *fun;
    Motor_var var;
    Motor_setval setval;
}; 

StepMotor_Driver* StepMotor_Create(UART_HandleTypeDef *huart,uint8_t adder);

#endif /* __STEPMOTOR_H */
