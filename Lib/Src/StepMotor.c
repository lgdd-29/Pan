#include "StepMotor.h"
#include "stdlib.h"
#include "EMM_V5.h"

void Motor_Set_Zero(StepMotor_Driver *driver)
{
    Emm_V5_Origin_Set_O(driver, 1);
}

void Motor_Move(StepMotor_Driver *driver, uint8_t dir, uint16_t val)
{
    Emm_V5_Vel_Control(driver,dir,val,0,0);
}

void Motor_Stop(StepMotor_Driver *driver)
{
    Emm_V5_Stop_Now(driver, 0);
}

void Motor_Init(StepMotor_Driver *driver)
{
    Emm_V5_En_Control(driver, 1, 0); // 使能电机，非同步模式
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

    // 初始化参数结构体指针
    driver->setval.huart = huart;
    driver->setval.adder = adder;



    return driver;
}