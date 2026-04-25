#include "menu.h"
#include "OLED.h"     // 只在这里include，对外隐藏
#include "stdio.h"    // sprintf
#include "Motor.h"     // 只在这里include，对外隐藏
#include "main.h"

#define PAGE_COUNT 3  //一共几页菜单

static int16_t page_index = 1;  // 当前选中参数索引
static int8_t menu_index = 0;  // 当前选中参数索引
static int8_t start_line = 0;   // 菜单窗口起始行（新加！）




static MenuItem M_VAR_menu_items[]=
{
    {"M1-P",     NULL, PARAM_FLOAT, 0,100,0.01},
    {"M1-I",     NULL, PARAM_FLOAT, 0,50,0.01},
    {"M1-D",     NULL, PARAM_FLOAT, 0,50,0.01},
    {"M1-target", NULL, PARAM_FLOAT, -200,200,5},
    {"M2-P",     NULL, PARAM_FLOAT, 0,100,0.01},
    {"M2-I",     NULL, PARAM_FLOAT, 0,50,0.01},
    {"M2-D",     NULL, PARAM_FLOAT, 0,50,0.01},
    {"M2-target", NULL, PARAM_FLOAT, -200,200,5},
};
//观测数据菜单
static MenuItem M_VAL_menu_items[] = {
    {"M1-target", NULL, PARAM_FLOAT, -200,200,5},
    {"M1-now",     NULL, PARAM_FLOAT, 0,0,0},
    {"M1-PIDOUT",     NULL, PARAM_FLOAT, 0,0,0},
    {"M1-ERR",     NULL, PARAM_FLOAT, 0,0,0},
    {"M2-target", NULL, PARAM_FLOAT, -200,200,5},
    {"M2-now",     NULL, PARAM_FLOAT, 0,0,0},
    {"M2-PIDOUT",     NULL, PARAM_FLOAT, 0,0,0},
    {"M2-ERR",     NULL, PARAM_FLOAT, 0,0,0},
};
//题目数据
static MenuItem Problem_menu_items[] = {
    {"x",NULL,PARAM_FLOAT,-2000,2000,10},
    {"y",NULL,PARAM_FLOAT,-2000,2000,10},
    {"number",NULL,PARAM_UINT16,0,6,1}
};
// 初始化：把外部实例的成员地址填进去，不使用全局变量！
void Menu_Init(Menu* menu, void* motor1,void* motor2,void* question)
{
    OLED_Init(); // 初始化OLED显示屏

    menu->current_menu = 1;
    Motor_Driver* m1 = (Motor_Driver*)motor1;
    Motor_Driver* m2 = (Motor_Driver*)motor2;
    question_Driver* q1 = (question_Driver*)question;

    menu->items1 = M_VAR_menu_items;
    menu->count1 = sizeof(M_VAR_menu_items)/sizeof(MenuItem);
    menu->items2 = M_VAL_menu_items;
    menu->count2 = sizeof(M_VAL_menu_items)/sizeof(MenuItem);
    menu->items3 = Problem_menu_items;
    menu->count3 = sizeof(Problem_menu_items)/sizeof(MenuItem);


    menu->items1[0].addr = &m1->var.Kp;
    menu->items1[1].addr = &m1->var.Ki;
    menu->items1[2].addr = &m1->var.Kd;
    menu->items1[3].addr = &m1->var.target;
    menu->items1[4].addr = &m2->var.Kp;
    menu->items1[5].addr = &m2->var.Ki;
    menu->items1[6].addr = &m2->var.Kd;
    menu->items1[7].addr = &m2->var.target;

    menu->items2[0].addr = &m1->var.target;
    menu->items2[1].addr = &m1->var.now;
    menu->items2[2].addr = &m1->var.out;
    menu->items2[3].addr = &m1->var.error;
    menu->items2[4].addr = &m2->var.target;
    menu->items2[5].addr = &m2->var.now;
    menu->items2[6].addr = &m2->var.out;
    menu->items2[7].addr = &m2->var.error;
    menu->items3[0].addr = &q1->x;
    menu->items3[1].addr = &q1->y;
    menu->items3[2].addr = &q1->num;

}

// ===================== 切换菜单（核心！） =====================
void Menu_Switch(Menu* menu, uint8_t menu_num)
{
    if(menu_num >= 1 && menu_num <= PAGE_COUNT){
        menu->current_menu = menu_num;
        // 切换时重置光标
        menu_index = 0;
        start_line = 0;
    }
}

// ===================== 获取当前菜单的内容 =====================
static MenuItem* Menu_GetCurrentItems(Menu* menu)
{
    if(menu->current_menu == 1) return M_VAR_menu_items;
    else if(menu->current_menu == 2) return M_VAL_menu_items;
    else if(menu->current_menu == 3) return Problem_menu_items;
    return NULL;
}

static uint8_t Menu_GetCurrentCount(Menu* menu)
{
    if(menu->current_menu == 1) return sizeof(M_VAR_menu_items)/sizeof(MenuItem);
    else if(menu->current_menu == 2) return sizeof(M_VAL_menu_items)/sizeof(MenuItem);
    else if(menu->current_menu == 3) return sizeof(Problem_menu_items)/sizeof(MenuItem);
    return 0;
}



// 获取当前选中参数的值
float Menu_GetCurrentValue(Menu* menu,uint8_t index)
{
    MenuItem* item = &Menu_GetCurrentItems(menu)[index];
    if(item->type == PARAM_FLOAT)
        return *(float*)item->addr;
    else if(item->type == PARAM_UINT16)
        return *(uint16_t*)item->addr;
    else
        return *(int*)item->addr;
}

// 增加
void Menu_Increase(Menu* menu,uint8_t index)
{
    MenuItem* item = &Menu_GetCurrentItems(menu)[index];
    float val = Menu_GetCurrentValue(menu,index);
    val += item->step;
    if(val > item->max) val = item->max;

    if(item->type == PARAM_FLOAT)
        *(float*)item->addr = val;
    else if(item->type == PARAM_UINT16)
        *(uint16_t*)item->addr = (uint16_t)val;
    else
        *(int*)item->addr = (int)val;
}

// 减少
void Menu_Decrease(Menu* menu,uint8_t index)
{
    MenuItem* item = &Menu_GetCurrentItems(menu)[index];
    float val = Menu_GetCurrentValue(menu,index);
    val -= item->step;
    if(val < item->min) val = item->min;

    if(item->type == PARAM_FLOAT)
        *(float*)item->addr = val;
    else if(item->type == PARAM_UINT16)
        *(uint16_t*)item->addr = (uint16_t)val;
    else
        *(int*)item->addr = (int)val;
}

// 显示菜单：根据传入的按键编号更新显示
void Menu_Show(Menu* menu,uint8_t key)
{
    if(key==1) menu_index--;
    else if(key==3) menu_index++;
    else if(key==4) Menu_Decrease(menu,menu_index);
    else if(key==2) Menu_Increase(menu,menu_index);
    else if(key==5) 
    {
        page_index++;
        if(page_index > PAGE_COUNT) 
        {
            page_index = 1;
        }
        Menu_Switch(menu, page_index);
    }
    else if(key==6) 
    {
        page_index--;
        if(page_index < 1) page_index = PAGE_COUNT;
        Menu_Switch(menu, page_index);
    }

    MenuItem* items = Menu_GetCurrentItems(menu);
    uint8_t count = Menu_GetCurrentCount(menu);

    if(menu_index >= count) menu_index = 0;
    else if(menu_index < 0) menu_index = count - 1;


    if(menu_index >= start_line + 8)  
    start_line = menu_index - 8 + 1;  
    if(menu_index < start_line)
    start_line = menu_index;

    OLED_Clear();
    for(int i=0;i<count;i++)
    {
        int real_idx = start_line + i;  // 真实索引
        if(real_idx >= count) break;
        // 显示参数名称
        if(real_idx == menu_index)
            OLED_ShowString(0, i*8, ">", OLED_6X8);
        else
            OLED_ShowString(0, i*8, " ", OLED_6X8);

        OLED_ShowString(8, i*8, items[real_idx].name, OLED_6X8);

        // 获取当前参数值
        float val = Menu_GetCurrentValue(menu,real_idx);
        
        // 根据参数类型格式化显示
        if(items[real_idx].type == PARAM_FLOAT)
        {
            int zs = (int)val;               // 整数 123
            int xs = (int)((val-zs)*1000);   // 小数 456
            if(xs<0) xs = -xs;
            OLED_Printf(64, i*8, OLED_6X8, ": %d.%03d", zs, xs);
        }
        else
            OLED_Printf(64, i*8, OLED_6X8, ": %d",(int)val);
    }
    OLED_Update();
}