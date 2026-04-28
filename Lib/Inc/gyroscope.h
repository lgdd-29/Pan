#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H

#include "stm32F4xx_hal.h"

/*陀螺仪*/
#define ACC_UPDATE   0x01  //加速度计更新标志
#define GYRO_UPDATE  0x02  //陀螺仪更新标志
#define ANGLE_UPDATE 0x04  //角度更新标志
#define MAG_UPDATE   0x08  //磁力计更新标志
#define READ_UPDATE  0x80  //读取更新标志

typedef struct 
{
    float fAcc[3];
    float fGyro[3];
    float fAngle[3];
}GyroData_t;
void gyroscope_Init(GyroData_t *pGyroData);
void GetAttitudeData(void);
float Gyro_YawPID(float target,float now,float Kp,float Ki,float Kd);
float Gyro_PitchPID(float target,float now,float Kp,float Ki,float Kd);
#endif
