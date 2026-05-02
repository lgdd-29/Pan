#include "Title.h"
#include <stdlib.h>
#include <stdint.h>
#include "math.h"
#define PI 3.14159265f
static FloatConvert conv;
void Title_Init(title_Driver *title)
{
    title->var.RxState=0;
    title->var.pRxPacket=0;
    title->var.Serial_RxFlag=0;
    title->var.tim_flag=0;
    title->var.ready=0;
    title->var.number=0;
    title->var.rx_byte=0;
    title->var.Serial_RxPacket[0]=0;
    title->var.Serial_RxPacket[1]=0;
    title->var.Serial_RxPacket[2]=0;
    title->var.Serial_RxPacket[3]=0;
    title->var.Serial_RxPacket[4]=0;
    title->var.Serial_RxPacket[5]=0;
    title->var.Serial_RxPacket[6]=0;
    title->var.Serial_RxPacket[7]=0;
    title->var.Serial_RxPacket[8]=0;
    title->var.Serial_RxPacket[9]=0;
    title->var.Serial_RxPacket[10]=0;
    title->var.Serial_RxPacket[11]=0;
    title->var.Start_Flag[0]=0xA5;
    title->var.Start_Flag[1]=0;
    title->var.Start_Flag[2]=0x5A;
    title->xy.x=0;
    title->xy.y=0;
    title->xy.y_offset=0;
    title->xy.h=22;//-10.0f;  //1m=-16.0   1.4m=-13.5
    title->xy.h_var=2.5;
    title->xy.L=0;
    title->xy.k=0;
    title->xy.mypitch=0;
    title->xy.myyaw=0;
}

void DataX_PIDOUT(title_Driver *title)
{
    if (title == NULL) return;

    title->pid.now=title->xy.x;  // 当前X坐标（从接收的数据更新）
    // 1. 计算当前偏差 (目标值 - 当前值)
    title->pid.error =title->pid.now-0;

    // 2. 积分累加 (建议后续根据需要加入积分限幅防饱和)
    title->pid.integral += title->pid.error;

    // 3. 位置式 PID 计算：Out = Kp*e + Ki*Integral + Kd*(e - last_e)
    title->pid.out = (title->pid.Kp * title->pid.error) + 
                     (title->pid.Ki * title->pid.integral) + 
                     (title->pid.Kd * (title->pid.error - title->pid.last_error));

    // 4. 更新上次偏差，用于下次微分计算
    title->pid.last_error = title->pid.error;
}

void DataX_PIDSET(title_Driver *title,float Kp,float Ki,float Kd)
{
    title->pid.Kp=Kp;
    title->pid.Ki=Ki;
    title->pid.Kd=Kd;
}

//TODO h补偿
void Laser_offset(title_Driver *title)
{
  title->xy.L=(title->xy.h+title->xy.k*title->xy.h_var)/cos(title->xy.mypitch*PI/180.0f);
  title->xy.y_offset=title->xy.y+title->xy.L;
}

//TODO 获取坐标原始数据
void Data_0xB6(title_Driver *title)
{
  if(title->var.RxState==1)
  {
    title->var.Serial_RxPacket[title->var.pRxPacket++]=title->var.rx_byte;
    if(title->var.pRxPacket>=12)
    {
      title->var.RxState=2;
    }
  }
  else if(title->var.RxState==2)
  {
    if(title->var.rx_byte==0x6B)
    {
      conv.bytes[0]=title->var.Serial_RxPacket[0];
      conv.bytes[1]=title->var.Serial_RxPacket[1];  
      conv.bytes[2]=title->var.Serial_RxPacket[2];
      conv.bytes[3]=title->var.Serial_RxPacket[3];
      title->xy.x=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[4];
      conv.bytes[1]=title->var.Serial_RxPacket[5];
      conv.bytes[2]=title->var.Serial_RxPacket[6];
      conv.bytes[3]=title->var.Serial_RxPacket[7];
      title->xy.y=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[8];
      conv.bytes[1]=title->var.Serial_RxPacket[9];
      conv.bytes[2]=title->var.Serial_RxPacket[10];
      conv.bytes[3]=title->var.Serial_RxPacket[11];
      //title->xy.k=conv.f;      ////////////////////
      Laser_offset(title);
    }

    title->var.RxState = 0;
    title->var.pRxPacket = 0;
  }
  else if(title->var.rx_byte==0xB6)
  {
    title->var.RxState=1;
    title->var.pRxPacket=0;
  }
}

//TODO 判断枕头帧尾
void Data_receive(title_Driver *title)
{
    if(title->var.RxState==0)
    {
        if(title->var.rx_byte==0xB6)
        {
            title->fun->Data_deal=Data_0xB6;  //坐标
            title->var.ready=0x01;
        }
    }
    if(title->var.ready==1) title->fun->Data_deal(title);
}

title_Driver* Titile_Create(void)
{
    title_Driver* title = (title_Driver*)malloc(sizeof(title_Driver));
    if(title != NULL)
    {
        title->fun = (title_fun*)malloc(sizeof(title_fun));
        if(title->fun != NULL)
        {
            title->fun->Init = Title_Init;
            title->fun->Data_receive = Data_receive;
            title->fun->X_PIDOUT = DataX_PIDOUT;
            title->fun->X_PIDSET = DataX_PIDSET;    
            title->fun->Data_deal = NULL; // 初始时没有数据处理函数，等接收到数据后根据题目类型再设置
        }
    }
    return title;
}