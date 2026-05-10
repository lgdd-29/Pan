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
    title->var.uart_flag=0;
    title->var.ready=0;
    title->var.number=0;
    title->var.rx_byte=0;
    title->var.mode=0;
    title->var.mode_next=0;
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
    title->xy.frame_x=0;
    title->xy.frame_y=0;
    title->xy.laser_x=0;
    title->xy.laser_y=0;
    title->xy.y_offset=0;
    title->xy.h=22;//-10.0f;  //1m=-16.0   1.4m=-13.5
    title->xy.h_var=2.5;
    title->xy.L=0;
    title->xy.k=0;
    title->xy.mypitch=0;
    title->xy.myyaw=0;
}

void XYRead(title_Driver *title)
{
  static uint8_t mytime=0;
  if((title->xy.x<1&&title->xy.x>-1)&&(title->xy.y>-1&&title->xy.y<1)) mytime++;
  if(mytime>1) title->var.ready=1;
}

//FUNCTION x坐标环
void DataX_PIDOUT(title_Driver *title)
{
    if (title == NULL) return;
    title->pid.error =title->xy.x;  // 目标值为0，所以偏差就是当前坐标的负值


    // 3. 稳态速度式累加：Out += (Kp*e + Ki*Integral + Kd*delta_e)
    title->pid.out += (title->pid.Kp * title->pid.error);
}

void DataX_PIDSET(title_Driver *title,float Kp,float Ki,float Kd)
{
    title->pid.Kp=Kp;
    title->pid.Ki=Ki;
    title->pid.Kd=Kd;
}

//FUNCTION h补偿
void Laser_offset(title_Driver *title)
{
  title->xy.L=(title->xy.h+title->xy.k*title->xy.h_var)/cos(title->xy.mypitch*PI/180.0f);
  title->xy.y_offset=title->xy.frame_y+title->xy.L;
}

//FUNCTION 获取框坐标原始数据
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
      title->xy.frame_x=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[4];
      conv.bytes[1]=title->var.Serial_RxPacket[5];
      conv.bytes[2]=title->var.Serial_RxPacket[6];
      conv.bytes[3]=title->var.Serial_RxPacket[7];
      title->xy.frame_y=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[8];
      conv.bytes[1]=title->var.Serial_RxPacket[9];
      conv.bytes[2]=title->var.Serial_RxPacket[10];
      conv.bytes[3]=title->var.Serial_RxPacket[11];
      //title->xy.k=conv.f;      ////////////////////
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

//FUNCTION 获取激光坐标原始数据
void Data_0xA5(title_Driver *title)
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
    if(title->var.rx_byte==0x5A)
    {
      conv.bytes[0]=title->var.Serial_RxPacket[0];
      conv.bytes[1]=title->var.Serial_RxPacket[1];  
      conv.bytes[2]=title->var.Serial_RxPacket[2];
      conv.bytes[3]=title->var.Serial_RxPacket[3];
      title->xy.laser_x=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[4];
      conv.bytes[1]=title->var.Serial_RxPacket[5];
      conv.bytes[2]=title->var.Serial_RxPacket[6];
      conv.bytes[3]=title->var.Serial_RxPacket[7];
      title->xy.laser_y=conv.f;
      conv.bytes[0]=title->var.Serial_RxPacket[8];
      conv.bytes[1]=title->var.Serial_RxPacket[9];
      conv.bytes[2]=title->var.Serial_RxPacket[10];
      conv.bytes[3]=title->var.Serial_RxPacket[11];
      title->xy.k=conv.f;      ////////////////////
    }

    title->var.RxState = 0;
    title->var.pRxPacket = 0;
  }
  else if(title->var.rx_byte==0xA5)
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
            title->var.mode_next=1;  //选择模式1
        }
        else if(title->var.rx_byte==0xA5)
        {
          title->fun->Data_deal=Data_0xA5;
          title->var.mode_next=2;  //选择模式2
        }
    }
    if(title->var.mode_next!=0) title->fun->Data_deal(title);
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
            title->fun->Laser_offset = Laser_offset;   
            title->fun->XYRead=XYRead;
            title->fun->Data_deal = NULL; // 初始时没有数据处理函数，等接收到数据后根据题目类型再设置
        } else {
            free(title);
            return NULL;
        }
    }
    return title;
}