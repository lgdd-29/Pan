#ifndef __Title_H
#define __Title_H
#include "stdint.h"
#include "stdlib.h"
typedef struct title_Driver title_Driver;
typedef struct title_var title_var;
typedef struct title_fun title_fun;
typedef struct title_xy title_xy;
typedef union {
    uint8_t bytes[8];
    float f;
} FloatConvert;
struct title_var
{
  uint8_t Start_Flag[3]; // 启动标志，包含3个按键的状态
  uint8_t RxState; // 接收状态：0-等待0xA5，1-接收数据，2-接收完成
  uint8_t pRxPacket; // 接收数据包的索引
  uint8_t rx_byte;  //接收的字节
  uint8_t Serial_RxPacket[8]; // 接收存储数据包
  uint8_t Serial_RxFlag;  // 接收完成标志
  uint8_t ready;  //视觉那边已经准备好了
  uint8_t number; //题目编号
  uint8_t tim_flag;  //定时器标志
};

struct title_fun
{
    void (*Init)(title_Driver *title); // 初始化函数指针
    void (*Data_receive)(title_Driver *title); // 数据处理函数指针，根据不同的题目调用不同的处理函数
    void (*Data_deal)(title_Driver *title); // 数据处理函数指针，根据不同的题目调用不同的处理函数
};

struct title_xy
{
    float h;
    float L;
    float y_offset;
    float x;
    float y;
    float mypitch;
};
struct title_Driver
{
    title_var var;
    title_fun *fun;
    title_xy xy;
};
title_Driver* Titile_Create(void);

#endif



