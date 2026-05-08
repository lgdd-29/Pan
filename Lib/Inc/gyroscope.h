#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H

#include "stm32F4xx_hal.h"

/*陀螺仪*/
#define ACC_UPDATE   0x01  //加速度计更新标志
#define GYRO_UPDATE  0x02  //陀螺仪更新标志
#define ANGLE_UPDATE 0x04  //角度更新标志
#define MAG_UPDATE   0x08  //磁力计更新标志
#define READ_UPDATE  0x80  //读取更新标志
typedef struct GyroPID GyroPID;
typedef struct GyroData_t GyroData_t;
typedef struct GyroFun GyroFun;
struct GyroFun
{
    void (*OUT)(GyroData_t *gyro,float target);
    void (*PID_SET)(GyroPID *pid,float Kp,float Ki,float Kd);
    void (*Check_Update)(GyroData_t *pGyroData,uint32_t update_flag);
};
struct GyroPID
{
    float Kp;
    float Ki;
    float Kd;
    float now;
    float target;
    float error;
    float err_prev;
    float integral;
    float out;    
    float differential;
};
struct GyroData_t
{
    float fAcc[3];
    float fGyro[3];
    float fAngle[3];
    float myyaw;
    float mypitch;
    uint8_t Gyro_Updata_Flag;
    GyroPID pid;
    GyroFun *fun;
};
void gyroscope_Init(GyroData_t *pGyroData);
void GetAttitudeData(void);
#endif
