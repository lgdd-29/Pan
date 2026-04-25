#include "StepMotor.h"
#include "stdlib.h"
#include "EMM_V5.h"

void Motor_Set_Zero(StepMotor_Driver *driver)
{
    Emm_V5_Origin_Set_O(driver, 1);
}

void Motor_Move(StepMotor_Driver *driver, uint16_t val)
{
    if(val>=0)
    Emm_V5_Vel_Control(driver,1,val,0,0);
    else 
    Emm_V5_Vel_Control(driver,0,(-val),0,0);
}

void Motor_Stop(StepMotor_Driver *driver)
{
    Emm_V5_Stop_Now(driver, 0);
}

void Motor_Init(StepMotor_Driver *driver)
{
    Emm_V5_En_Control(driver, 1, 0); // 使能电机，非同步模式
    driver->var.pid.Kp = 0; 
    driver->var.pid.Ki = 0;
    driver->var.pid.Kd = 0;
    driver->var.pid.now = 0;
    driver->var.pid.target = 0;
    driver->var.pid.error = 0;
    driver->var.pid.err_prev=0.0f;
    driver->var.pid.last_error = 0.0f;
    driver->var.pid.integral = 0.0f;
    driver->var.pid.out = 0.0f;    
    driver->var.Move_max=1000.0f; // 根据实际情况设置最大移动范围
}

void Step_PIDOUT(StepMotor_Driver *driver,float target,float now)
{
    driver->var.pid.target = target;
    driver->var.pid.now = now;
    driver->var.pid.error = driver->var.pid.target - driver->var.pid.now; 
    
    // 增量式公式（无累计，只算变化量）
    float increment =  driver->var.pid.Kp*(driver->var.pid.error - driver->var.pid.last_error) 
                    + driver->var.pid.Ki*driver->var.pid.error 
                    + driver->var.pid.Kd*(driver->var.pid.error - 2*driver->var.pid.last_error +driver->var.pid.err_prev);
    
    driver->var.pid.out += increment;  // 输出 = 上一次输出 + 增量（平滑调速）
    if(driver->var.pid.out > driver->var.Move_max) {
        driver->var.pid.out = driver->var.Move_max;
    } else if (driver->var.pid.out < -driver->var.Move_max) {
        driver->var.pid.out = -driver->var.Move_max;
    }
    // 误差更新
    driver->var.pid.err_prev = driver->var.pid.last_error;
    driver->var.pid.last_error = driver->var.pid.error;
}

void Step_PID_SET(StepMotor_Driver *driver,float Kp,float Ki,float Kd)
{
    driver->var.pid.Kp = Kp;
    driver->var.pid.Ki = Ki;
    driver->var.pid.Kd = Kd;
}

StepMotor_Driver* StepMotor_Create(UART_HandleTypeDef *huart,uint8_t adder)
{
    StepMotor_Driver *driver = (StepMotor_Driver *)malloc(sizeof(StepMotor_Driver));
    if (driver == NULL) {
        while(1); // 内存分配失败，进入死循环
    }

    // 初始化函数指针
    driver->fun=(Motor_fun *)malloc(sizeof(Motor_fun));
    driver->fun->Set_Zero=Motor_Set_Zero;
    driver->fun->Init=Motor_Init;
    driver->fun->Move=Motor_Move;
    driver->fun->Stop=Motor_Stop;
    driver->fun->PID_OUT=Step_PIDOUT;
    driver->fun->PID_SET=Step_PID_SET;
    

    // 初始化参数结构体指针
    driver->setval.huart = huart;
    driver->setval.adder = adder;



    return driver;
}