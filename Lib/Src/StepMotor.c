#include "StepMotor.h"
#include "stdlib.h"
#include "EMM_V5.h"

void Motor_Set_Zero(StepMotor_Driver *driver)
{
    Emm_V5_Origin_Set_O(driver, 1);
}

void Motor_Move(StepMotor_Driver *driver, float val)
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
    driver->var.Move_max=200.0f; // 根据实际情况设置最大移动范围
}


void Step_PID_SET(StepMotor_Driver *driver,float Kp,float Ki,float Kd)
{
    EMM_V5_PIDSET(driver, 0, (uint32_t)(Kp*1000), (uint32_t)(Ki*1000), (uint32_t)(Kd*1000));
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
    driver->fun->PID_SET=Step_PID_SET;
    

    // 初始化参数结构体指针
    driver->setval.huart = huart;
    driver->setval.adder = adder;



    return driver;
}