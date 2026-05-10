#include "gyroscope.h"
#include "wit_c_sdk.h"
#include "uart4.h"
#include "gyroscope.h"
#include "stdlib.h"
extern UART_HandleTypeDef huart4;

static volatile char s_cDataUpdate = 0;

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

static GyroData_t *s_GyroData;

//FUNCTION 陀螺仪环
void Gyro_YawPID(GyroData_t* GyroData,double target)
{
    GyroData->pid.now=GyroData->myyaw;  // 当前角度（从陀螺仪数据更新）
    // ===================== 1. 目标累加与限幅（防止超飞导致超调）=====================
    GyroData->pid.target=target; // 目标角度（函数参数传入）

    // ===================== 2. 360°循环角度误差计算（核心！）=====================
    GyroData->pid.error = GyroData->pid.now-GyroData->pid.target; // 计算当前偏差 (目标值 - 当前值)
    // 处理循环角：误差超过180°或小于-180°时，取最短路径
    if(GyroData->pid.error > 180)
        GyroData->pid.error -= 360;
    else if(GyroData->pid.error < -180)
        GyroData->pid.error += 360;

    // ===================== 3. 积分项 + 积分分离（防急停过冲）=====================
    // 只在靠近目标时（误差小于10度）才进行积分累积
    if (GyroData->pid.error < 10.0 && GyroData->pid.error > -10.0) {
         GyroData->pid.integral += GyroData->pid.error;
    } else {
         GyroData->pid.integral = 0;
    }

    // ===================== 4. 微分项（真实物理阻尼！）=====================
    // 原始 D 项容易受跳变影响，我们直接使用陀螺仪本身的物理角速度作为强大的刹车阻尼项
    GyroData->pid.differential = GyroData->fGyro[2];
    
    // ===================== 5. PID输出计算 =====================
    GyroData->pid.out = GyroData->pid.Kp * GyroData->pid.error + GyroData->pid.Ki * GyroData->pid.integral + GyroData->pid.Kd * GyroData->pid.differential;
    // ===================== 7. 更新历史误差 =====================
    GyroData->pid.err_prev = GyroData->pid.error;
}

void Gyro_PID_SET(GyroPID *pid,float Kp,float Ki,float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;   
}

// 参数传入你需要检查的掩码，例如 ANGLE_UPDATE
void Check_Sensor_Update(GyroData_t *pGyroData,uint32_t update_flag)
 {
    if (s_cDataUpdate & update_flag) 
    {
        s_cDataUpdate &= ~update_flag; // 清除对于的标志位
        pGyroData->Gyro_Updata_Flag = 1;
    }
}

void gyroscope_Init(GyroData_t *pGyroData)
{
    s_GyroData = pGyroData;
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    pGyroData->fun=(GyroFun *)malloc(sizeof(GyroFun));
    if (pGyroData->fun == NULL) while(1); // 内存分配失败，直接死循环
    pGyroData->fun->OUT=Gyro_YawPID;
    pGyroData->fun->PID_SET=Gyro_PID_SET;
    pGyroData->fun->Check_Update=Check_Sensor_Update;
     // 初始化PID参数，根据实际情况调整

    pGyroData->pid.Kp = 0.0f;  // 根据实际情况调整PID参数
    pGyroData->pid.Ki = 0.0f;
    pGyroData->pid.Kd = 0.0f;
    pGyroData->pid.now = 0.0f;
    pGyroData->pid.target = 0.0f;
    pGyroData->pid.integral=0;
    pGyroData->pid.error = 0.0f;
    pGyroData->pid.err_prev = 0.0f;
    pGyroData->pid.err_prev_2 = 0.0f;
    pGyroData->pid.out = 0.0f;
    pGyroData->pid.differential = 0.0f;
    pGyroData->myyaw=0;
    pGyroData->mypitch=0;
    pGyroData->Gyro_Updata_Flag=0;
}


static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    Uart4Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

// 数据更新标志位，使用位掩码表示不同类型的数据更新状态
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
            //            case AX:
            //            case AY:
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
            //            case GX:
            //            case GY:
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
            //            case HX:
            //            case HY:
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
            //            case Roll:
            //            case Pitch:
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static int i;
void GetAttitudeData(void)
{
    if (s_cDataUpdate)
    {
        for (i = 0; i < 3; i++)
        {
            s_GyroData->fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
            s_GyroData->fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
            s_GyroData->fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
            s_GyroData->myyaw=s_GyroData->fAngle[2];
            s_GyroData->mypitch=s_GyroData->fAngle[0];
        }
        if (s_cDataUpdate & ACC_UPDATE)
        {
            s_cDataUpdate &= ~ACC_UPDATE;
        }
        if (s_cDataUpdate & GYRO_UPDATE)
        {
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
        if (s_cDataUpdate & ANGLE_UPDATE)
        {
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        if (s_cDataUpdate & MAG_UPDATE)
        {
            s_cDataUpdate &= ~MAG_UPDATE;
        }
    }
}
