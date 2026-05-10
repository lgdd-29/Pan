#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx_hal.h"
typedef struct Motor_Driver Motor_Driver;
typedef struct Motor_FUN Motor_FUN;
typedef struct Motor_VAR Motor_VAR;
typedef struct Motor_PER Motor_PER;
struct Motor_FUN
{
    void (*Motor_Init)(Motor_Driver *motor);
    void (*Motor_Move)(Motor_Driver *motor, float target);
    void (*Motor_Cali)(Motor_Driver *motor);
    int (*Motor_ReadMove)(Motor_Driver *motor);
    void (*MPID_OUT)(Motor_Driver *motor,float now);
    void (*PID_SET)(Motor_Driver *motor,float Kp,float Ki,float Kd);    
};

struct Motor_VAR
{
    double now;
    double target;
    double error;
    double last_error;
    double sec_last_error;
    double integral;
    double max_integral;
    double Kp;
    double Ki;
    double Kd;
    double out;
    int16_t Move_max;
    int16_t Move_min;
    uint16_t middle_pos;
};

struct Motor_PER
{
    int Motor_ID;
};
struct Motor_Driver
{
    Motor_FUN *fun;
    Motor_VAR var;
    Motor_PER per;
};

Motor_Driver* Motor_Create(int Motor_ID,int16_t max,int16_t min,uint16_t middle_pos);
#endif /* __MOTOR_H */

