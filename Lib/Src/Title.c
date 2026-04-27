#include "Title.h"
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
    title->var.Start_Flag[0]=0xA5;
    title->var.Start_Flag[1]=0;
    title->var.Start_Flag[2]=0x5A;
}

void Data_0xB6(title_Driver *title)
{
  if(title->var.RxState==1)
  {
    title->var.Serial_RxPacket[title->var.pRxPacket++]=title->var.rx_byte;
    if(title->var.pRxPacket>=8)
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

void Data_receive(title_Driver *title)
{
    if(title->var.RxState==0)
    {
        if(title->var.rx_byte==0xB6)
        {
            title->fun->Data_deal=Data_0xB6;  //坐标
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
            title->fun->Data_deal = NULL; // 初始时没有数据处理函数，等接收到数据后根据题目类型再设置
            title->fun->Init(title); // 初始化题目数据
        }
    }
    return title;
}