#include "Motor.h"
#include "SMS_STS.h"
#include "uart.h"
#include <stdint.h>
#include <stdlib.h>
void PanMotor_Init(Motor_Driver *motor)
{
    motor->var.Kp = 0; // 根据需要调整PID参数
    motor->var.Ki = 0;
    motor->var.Kd = 0;
    motor->var.now = 0.0f;
    motor->var.integral = 0.0f;
    motor->var.max_integral = 0.0f;
    motor->var.last_error = 0.0f;
    motor->var.target = 0.0f;
    motor->var.error = 0.0f;
    motor->var.out = 0.0f;
    motor->var.sec_last_error = 0.0f;
}


// 速度控制指令
void PanMotor_Move(Motor_Driver *motor, float target)
{
    WritePosEx(motor->per.Motor_ID, (int16_t)target+motor->var.middle_pos, 90, 30);
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

void PanMotor_PID_SET(Motor_Driver *motor,float Kp,float Ki,float Kd)
{
    motor->var.Kp = Kp;
    motor->var.Ki = Ki;
    motor->var.Kd = Kd;
    if(motor->var.Ki!=0) motor->var.max_integral=50/Ki; // 根据Ki参数动态调整积分限幅，防止积分过大导致系统不稳定
}

void PanMPID_OUT(Motor_Driver *motor,float now)
{
    // 1. 当前值（保持你原来的负号逻辑）
    motor->var.now = -now;

    // 2. 本次误差 e(k) （你原来没有target，error直接=当前值）
    motor->var.error = motor->var.now;

    // ===================== 增量式PID 核心公式 =====================
    float delta_u = motor->var.Kp * (motor->var.error - motor->var.last_error)
                  + motor->var.Ki * motor->var.error
                  + motor->var.Kd * (motor->var.error - 2 * motor->var.last_error + motor->var.sec_last_error);

    // 4. 输出 = 上一次输出 + 增量
    motor->var.out += delta_u;

    // 5. 输出限幅（舵机必须加，防止超量程）
    if(motor->var.out > 100)  motor->var.out = 100;
    if(motor->var.out < -100) motor->var.out = -100;

    // 6. 更新历史误差
    motor->var.sec_last_error = motor->var.last_error;  // 保存 e(k-1) → 变成下一次 e(k-2)
    motor->var.last_error = motor->var.error;       // 保存 e(k)   → 变成下一次 e(k-1)
}



Motor_Driver* Motor_Create(int Motor_ID,int16_t max,int16_t min,uint16_t middle_pos)
{
    Motor_Driver* motor=(Motor_Driver*)malloc(sizeof(Motor_Driver));
    if (motor == NULL) return NULL;
    
    motor->fun = (Motor_FUN *)malloc(sizeof(Motor_FUN));
    if (motor->fun == NULL) return NULL;

    motor->fun->Motor_Init = PanMotor_Init;
    motor->fun->Motor_Move = PanMotor_Move;
    motor->fun->Motor_Cali = PanMotor_Cali;
    motor->fun->Motor_ReadMove = PanMotor_ReadMove;
    motor->fun->MPID_OUT = PanMPID_OUT;
    motor->fun->PID_SET = PanMotor_PID_SET;

    motor->per.Motor_ID = Motor_ID;
    motor->var.Move_max=max;
    motor->var.Move_min=min;
    motor->var.middle_pos=middle_pos;
    return motor;
}
