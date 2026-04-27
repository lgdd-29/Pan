#include "Title.h"
void Title_Init(title_Driver *title)
{
    title->var.RxState=0;
    title->var.pRxPacket=0;
    title->var.Serial_RxFlag=0;
    title->var.tim_flag=0;
    title->var.ready=0;
    title->var.number=0;
    title->var.rx_byte=0;
    title->var.Serial_RxPacket=0;
    title->var.Start_Flag[0]=0xA5;
    title->var.Start_Flag[1]=0;
    title->var.Start_Flag[2]=0x5A;
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
            title->fun->Data_receive = ;
            title->fun->Init(title); // 初始化题目数据
        }
    }
    return title;
}