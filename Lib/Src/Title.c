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
    title->var.Start_Flag[0]=0xA5;
    title->var.Start_Flag[1]=0;
    title->var.Start_Flag[2]=0x5A;
    title->xy.x=0;
    title->xy.y=0;
    title->xy.y_offset=0;
    title->xy.h=100;
    title->xy.L=0;
    title->xy.mypitch=0;
}

// 0~90度余弦值查表，放大10000倍
// 对应公式: cos_table[i] = round(cos(i * PI / 180) * 10000)
static const int16_t cos_table[91] = {
    10000, 9998, 9993, 9986, 9975, 9961, 9945, 9925, 9902, 9876, 
    9848, 9816, 9781, 9743, 9702, 9659, 9612, 9563, 9510, 9455, 
    9396, 9335, 9271, 9205, 9135, 9063, 8987, 8910, 8829, 8746, 
    8660, 8571, 8480, 8386, 8290, 8191, 8090, 7986, 7880, 7771, 
    7660, 7547, 7431, 7313, 7193, 7071, 6946, 6819, 6691, 6560, 
    6427, 6293, 6156, 6018, 5877, 5735, 5591, 5446, 5299, 5150, 
    5000, 4848, 4694, 4539, 4383, 4226, 4067, 3907, 3746, 3583, 
    3420, 3255, 3090, 2923, 2756, 2588, 2419, 2249, 2079, 1908, 
    1736, 1564, 1391, 1218, 1045,  871,  697,  523,  348,  174, 
       0
};


/**
 * @brief  快速余弦查表函数
 * @param  angle: 角度值，支持负数、大于360度的任意角度（如 -45, 0, 90, 370）
 * @retval 角度对应的余弦值（已放大10000倍）。如传入60，返回5000
 */
int16_t Fast_Cos(int32_t angle)
{
    // 将角度限制在 0 ~ 359 之内
    angle = angle % 360;
    if (angle < 0) {
        angle += 360;
    }

    // 利用三角函数的象限对称性映射到 0 ~ 90 度
    if (angle <= 90) {               // 第一象限: 0~90
        return cos_table[angle];
    } 
    else if (angle <= 180) {         // 第二象限: 90~180
        return -cos_table[180 - angle];
    } 
    else if (angle <= 270) {         // 第三象限: 180~270
        return -cos_table[angle - 180];
    } 
    else {                           // 第四象限: 270~360
        return cos_table[360 - angle];
    }
}

/**
 * @brief  快速正弦查表函数 (附加：有了余弦，正弦也可直接转换)
 * @param  angle: 角度值
 * @retval 角度对应的正弦值（已放大10000倍）。
 */
int16_t Fast_Sin(int32_t angle)
{
    // sin(x) = cos(x - 90)
    return Fast_Cos(angle - 90);
}

void Laser_offset(title_Driver *title)
{
  title->xy.L=title->xy.h/cos(title->xy.mypitch*PI/180.0f);
  title->xy.y_offset=title->xy.y-title->xy.L;
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
      title->xy.x=conv.f-320;
      conv.bytes[0]=title->var.Serial_RxPacket[4];
      conv.bytes[1]=title->var.Serial_RxPacket[5];
      conv.bytes[2]=title->var.Serial_RxPacket[6];
      conv.bytes[3]=title->var.Serial_RxPacket[7];
      title->xy.y=240-conv.f;
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
            title->fun->Data_deal = NULL; // 初始时没有数据处理函数，等接收到数据后根据题目类型再设置
            title->fun->Init(title); // 初始化题目数据
        }
    }
    return title;
}