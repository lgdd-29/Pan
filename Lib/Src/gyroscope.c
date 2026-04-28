#include "gyroscope.h"
#include "wit_c_sdk.h"
#include "uart4.h"
extern UART_HandleTypeDef huart4;

static volatile char s_cDataUpdate = 0;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

static GyroData_t *s_GyroData;
void gyroscope_Init(GyroData_t *pGyroData)
{
    s_GyroData = pGyroData;
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    // AutoScanSensor();
}

static void AutoScanSensor(void)
{
    int i, iRetry;

    for (i = 1; i < 10; i++)
    {
        Usart4Init(c_uiBaud[i]);
        iRetry = 2; 
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            HAL_Delay(100);
            if (s_cDataUpdate != 0)
            {
                // printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
                return;
            }
            iRetry--;
        } while (iRetry);
    }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    Uart4Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

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

float Gyro_YawPID(float target, float now, float Kp, float Ki, float Kd)
{
    // 静态变量（保存历史值）
    static float error_prev = 0.0f;  // 上一次误差
    static float integral  = 0.0f;   // 积分累加值
    
    float error;        // 当前角度误差
    float differential; // 微分项
    float output;       // 最终输出

    // ===================== 1. 360°循环角度误差计算（核心！）=====================
    error = target - now;
    // 处理循环角：误差超过180°或小于-180°时，取最短路径
    if(error > 180)
        error -= 360;
    else if(error < -180)
        error += 360;

    // ===================== 2. 积分项 + 积分限幅（防饱和）=====================
    integral += error;
    // 积分限幅（根据你的电机/舵机调整大小，一般±100~±500）
    if(integral > 200)  integral = 200;
    if(integral < -200) integral = -200;

    // ===================== 3. 微分项（标准PID）=====================
    differential = error - error_prev;

    // ===================== 4. PID输出计算 =====================
    output = Kp * error + Ki * integral + Kd * differential;

    // ===================== 5. 输出限幅（防止电机超量程）=====================
    if(output > 1000)  
        output = 1000;
    if(output < -1000) 
        output = -1000;

    // ===================== 6. 更新历史误差 =====================
    error_prev = error;

    return output;
}

float Gyro_PitchPID(float target, float now, float Kp, float Ki, float Kd)
{
    // 静态变量：保存上一次误差 & 积分值
    static float error_prev = 0.0f;
    static float integral  = 0.0f;
    
    float error;        // 当前误差
    float differential; // 微分项
    float output;       // 输出

    // 1. 计算误差（直接相减，因为是线性角度）
    error = target - now;

    // 2. 积分项 + 积分限幅（防止积分饱和）
    integral += error;
    if(integral > 150)  integral = 150;   // 积分上限（小角度可调小）
    if(integral < -150) integral = -150;  // 积分下限

    // 3. 微分项（标准PID，本次误差 - 上一次误差）
    differential = error - error_prev;

    // 4. 计算PID输出
    output = Kp * error + Ki * integral + Kd * differential;

    // 5. 输出限幅（保护舵机/电机，小角度不需要太大输出）
    if(output > 200)  output = 200;
    if(output < -200) output = -200;

    // 6. 更新上一次误差
    error_prev = error;

    return output;
}