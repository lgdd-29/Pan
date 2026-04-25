#include "Motor.h"
#include "SMS_STS.h"
#include "uart.h"
#include <stdint.h>
#include <stdlib.h>
void PanMotor_Init(Motor_Driver *motor)
{
    motor->var.Kp = 0.01f; // 根据需要调整PID参数
    motor->var.Ki = 0.0f;
    motor->var.Kd = 0.0f;
    motor->var.now = 0.0f;
    motor->var.integral = 0.0f;
    motor->var.last_error = 0.0f;
    motor->var.target = 0.0f;
    motor->var.error = 0.0f;
    motor->var.out = 0.0f;
}


// 位置控制指令
void PanMotor_Move(Motor_Driver *motor, float target)
{
    if(target > motor->var.Move_max) {
        target = motor->var.Move_max;
    } else if (target < motor->var.Move_min) {
        target = motor->var.Move_min;
    }
    WritePosEx(motor->per.Motor_ID, (int16_t)target+motor->var.middle_pos, 5, 0);
}

// 读取当前位置指令
int PanMotor_ReadMove(Motor_Driver *motor)
{
    return ReadPos(motor->per.Motor_ID);
}

// 舵机校准指令
void PanMotor_Cali(Motor_Driver *motor)
{
    CalibrationOfs(motor->per.Motor_ID); // 调用中位校准函数
}

void PanMPID_OUT(Motor_Driver *motor,float target,float now)
{
    motor->var.now=now;
    motor->var.target=target;
    motor->var.error = motor->var.target - motor->var.now;
    motor->var.integral += motor->var.error;
    float derivative = motor->var.error - motor->var.last_error;
    motor->var.out += motor->var.Kp * motor->var.error + motor->var.Ki * motor->var.integral + motor->var.Kd * derivative;
    PanMotor_Move(motor, (int16_t)motor->var.out);
    motor->var.last_error = motor->var.error;
}



Motor_Driver* Motor_Create(int Motor_ID,int16_t max,int16_t min,uint16_t middle_pos)
{
    Motor_Driver* motor=(Motor_Driver*)malloc(sizeof(Motor_Driver));
    motor->fun = (Motor_FUN *)malloc(sizeof(Motor_FUN));
    motor->fun->Motor_Init = PanMotor_Init;
    motor->fun->Motor_Move = PanMotor_Move;
    motor->fun->Motor_Cali = PanMotor_Cali;
    motor->fun->Motor_ReadMove = PanMotor_ReadMove;
    motor->fun->MPID_OUT = PanMPID_OUT;

    motor->per.Motor_ID = Motor_ID;
    motor->var.Move_max=max;
    motor->var.Move_min=min;
    motor->var.middle_pos=middle_pos;
    return motor;
}
