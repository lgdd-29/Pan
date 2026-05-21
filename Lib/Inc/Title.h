#ifndef __Title_H
#define __Title_H
#include "stdint.h"
#include "stdlib.h"
typedef struct title_Driver title_Driver;
typedef struct title_var title_var;
typedef struct title_fun title_fun;
typedef struct title_xy title_xy;
typedef struct title_PID title_PID;
typedef union {
    uint8_t bytes[12];
    float f;
} FloatConvert;
struct title_var
{
  uint8_t Start_Flag[3]; // 启动标志，包含3个按键的状态
  uint8_t RxState; // 接收状态：0-等待0xA5，1-接收数据，2-接收完成
  uint8_t pRxPacket; // 接收数据包的索引
  uint8_t rx_byte;  //接收的字节
  uint8_t Serial_RxPacket[12]; // 接收存储数据包
  uint8_t car_rx_buffer[6];
  uint8_t Serial_RxFlag;  // 接收完成标志
  uint8_t Xready;  //X坐标已经准备好了
  uint8_t Yready;  //Y坐标已经准备好了
  uint8_t ready;  //视觉那边已经准备好了
  uint8_t number; //题目编号
  uint8_t tim_flag;  //定时器标志
  uint8_t uart_flag; //串口标志 
  uint8_t mode; //模式选择
  uint8_t mode_next; //模式选择
};


struct title_PID
{
    float Kp;
    float Ki;
    float Kd;
    double now;
    double target;
    double error;
    double integral;
    double last_error;
    double out;    
    double ff_out;
};

struct title_fun
{
    void (*Init)(title_Driver *title); // 初始化函数指针
    void (*Data_receive)(title_Driver *title); // 数据处理函数指针，根据不同的题目调用不同的处理函数
    void (*Data_deal)(title_Driver *title); // 数据处理函数指针，根据不同的题目调用不同的处理函数
    void (*X_PIDOUT)(title_Driver *title); // X坐标位置控制函数指针
    void (*X_PIDSET)(title_Driver *title,float Kp,float Ki,float Kd); // X坐标PID参数设置函数指针
    void (*Laser_offset)(title_Driver *title); // 激光补偿计算函数指针
    void (*XYRead)(title_Driver *title);
};

struct title_xy
{
    float h;
    float h_var;
    double L;
    float frame_x;
    float last_frame_x;
    float frame_y;
    double y_offset;
    float laser_x;
    float laser_y;
    double x;
    double y;
    float mypitch;
    float myyaw;
    float k;
    float V_ff; // 固定无变化速度前馈
    uint8_t lost_laser; // 激光丢失标志
};
struct title_Driver
{
    title_var var;
    title_fun *fun;
    title_xy xy;
    title_PID pid;
};
title_Driver* Titile_Create(void);

#endif



