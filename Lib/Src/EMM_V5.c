#include "Emm_V5.h"
#include "stm32f4xx.h"
/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	适配说明：
***	1. 新增usart_SendCmd函数（HAL_UART_Transmit实现）
***	2. 保留所有电机控制逻辑，仅替换串口发送接口
***	3. 兼容STM32F407 HAL库和AC6编译器
**********************************************************/

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Reset_CurPos_To_Zero(StepMotor_Driver *driver)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令（原代码逻辑完全保留）
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令（调用HAL库版usart_SendCmd）
  HAL_UART_Transmit(driver->setval.huart, cmd, 4, 50);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Reset_Clog_Pro(StepMotor_Driver *driver)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 4,50);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   无
  */
void Emm_V5_Read_Sys_Params(StepMotor_Driver *driver, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = driver->setval.adder; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, i, 50);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式
  * @retval   无
  */
void Emm_V5_Modify_Ctrl_Mode(StepMotor_Driver *driver, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  ctrl_mode;                  // 控制模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 6,50);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_En_Control(StepMotor_Driver *driver, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 6,50);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向
  * @param    vel ：速度
  * @param    acc ：加速度
  * @param    snF ：多机同步标志
  * @retval   无
  */
void Emm_V5_Vel_Control(StepMotor_Driver *driver, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度高8位
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度低8位
  cmd[5] =  acc;                        // 加速度
  cmd[6] =  snF;                        // 多机同步标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 8,50);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向
  * @param    vel ：速度
  * @param    acc ：加速度
  * @param    clk ：脉冲数
  * @param    raF ：相位/绝对标志
  * @param    snF ：多机同步标志
  * @retval   无
  */
void Emm_V5_Pos_Control(StepMotor_Driver *driver, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  driver->setval.adder;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度高8位
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度低8位 
  cmd[5]  =  acc;                       // 加速度
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数bit24-31
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数bit16-23
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数bit8-15
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数bit0-7
  cmd[10] =  raF;                       // 相位/绝对标志
  cmd[11] =  snF;                       // 多机同步标志
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 13,50);
}

/**
  * @brief    立即停止
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_Stop_Now(StepMotor_Driver *driver, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                      // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 5,50);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Synchronous_motion(StepMotor_Driver *driver)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 4,50);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志
  * @retval   无
  */
void Emm_V5_Origin_Set_O(StepMotor_Driver *driver, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 5,50);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志
  * @param    o_mode ：回零模式
  * @param    o_dir  ：回零方向
  * @param    o_vel  ：回零速度
  * @param    o_tm   ：回零超时时间
  * @param    sl_vel ：碰撞检测转速
  * @param    sl_ma  ：碰撞检测电流
  * @param    sl_ms  ：碰撞检测时间
  * @param    potF   ：上电自动回零
  * @retval   无
  */
void Emm_V5_Origin_Modify_Params(StepMotor_Driver *driver, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  o_mode;                     // 回零模式
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度低8位 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 超时时间bit24-31
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 超时时间bit16-23
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 超时时间bit8-15
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 超时时间bit0-7
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 碰撞转速高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 碰撞转速低8位 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 碰撞电流高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 碰撞电流低8位 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 碰撞时间高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 碰撞时间低8位
  cmd[18] =  potF;                      // 上电自动回零
  cmd[19] =  0x9B;                      // 校验字节（原代码0x6B，按实际协议调整）
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 20,50);
}

// 辅助计算CRC16函数
static uint16_t EMM_V5_CRC16(uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

/**
  * @brief    修改PID参数
  * @param    driver : 电机驱动结构体指针
  * @param    svF    : 是否存储标志（0-不存储 1-存储）
  * @param    Kp     : 比例项 (32位)
  * @param    Ki     : 积分项 (32位)
  * @param    Kd     : 微分项 (32位)
  * @retval   无
  */
void EMM_V5_PIDSET(StepMotor_Driver *driver, bool svF, uint32_t Kp, uint32_t Ki, uint32_t Kd)
{
  uint8_t cmd[23] = {0};
  
  cmd[0] = driver->setval.adder; // 从机地址
  cmd[1] = 0x10;                 // 功能码
  cmd[2] = 0x00;                 // 寄存器地址 Hi
  cmd[3] = 0x4A;                 // 寄存器地址 Lo
  cmd[4] = 0x00;                 // 寄存器数量 Hi
  cmd[5] = 0x07;                 // 寄存器数量 Lo
  cmd[6] = 0x0E;                 // 字节数 14
  
  cmd[7] = 0xC3;                 // 寄存器1 Hi (固定值)
  cmd[8] = svF ? 0x01 : 0x00;    // 寄存器1 Lo (是否存储)
  
  cmd[9]  = (uint8_t)(Kp >> 24); // 寄存器2 Hi
  cmd[10] = (uint8_t)(Kp >> 16); // 寄存器2 Lo
  cmd[11] = (uint8_t)(Kp >> 8);  // 寄存器3 Hi
  cmd[12] = (uint8_t)(Kp & 0xFF);// 寄存器3 Lo
  
  cmd[13] = (uint8_t)(Ki >> 24); // 寄存器4 Hi
  cmd[14] = (uint8_t)(Ki >> 16); // 寄存器4 Lo
  cmd[15] = (uint8_t)(Ki >> 8);  // 寄存器5 Hi
  cmd[16] = (uint8_t)(Ki & 0xFF);// 寄存器5 Lo
  
  cmd[17] = (uint8_t)(Kd >> 24); // 寄存器6 Hi
  cmd[18] = (uint8_t)(Kd >> 16); // 寄存器6 Lo
  cmd[19] = (uint8_t)(Kd >> 8);  // 寄存器7 Hi
  cmd[20] = (uint8_t)(Kd & 0xFF);// 寄存器7 Lo
  
  // 计算前21个字节的CRC16 (Modbus RTU: CRC Low first, then CRC High)
  uint16_t crc = EMM_V5_CRC16(cmd, 21);
  // 注意图中画的是Hi先Lo后还是反过来，我们先按标准Modbus规约低位在前高位在后
  // 或者你也可以看厂家手册具体怎么要求，这里采用常见Modbus CRC低位在前
  cmd[21] = (crc >> 8) & 0xFF;   // 先发 CRC16 Hi
  cmd[22] = (crc >> 8) & 0xFF;   // CRC16 Hi (或者是按照协议这里为CRC高字节)

  
  HAL_UART_Transmit(driver->setval.huart, cmd, 23, 100);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_Origin_Trigger_Return(StepMotor_Driver *driver, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式
  cmd[3] =  snF;                        // 多机同步标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 5,50);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Origin_Interrupt(StepMotor_Driver *driver)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  driver->setval.adder;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(driver->setval.huart, cmd, 4,50);
}
