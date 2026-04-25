#ifndef __MENU_H
#define __MENU_H

#include "stdint.h"

// 参数类型
typedef enum {
    PARAM_FLOAT,
    PARAM_INT,
    PARAM_UINT16,
} ParamType;

// 菜单条目：只存【指针 + 类型 + 范围】
typedef struct {
    const char* name;
    void*       addr;      // 变量指针（核心）
    ParamType   type;
    float       min;
    float       max;
    float       step;
} MenuItem;

// 菜单实例（持有所有参数 + 指向对应对象的指针）
typedef struct {
    MenuItem*  items1;  // 参数列表
    uint8_t    count1;  // 参数数量

    // 当前使用哪个菜单 1或2
    uint8_t current_menu;
} Menu;

// 初始化菜单（把外部实例传进来，无全局变量）
void Menu_Init(Menu* menu,void* title);

// 菜单操作
float Menu_GetCurrentValue(Menu* menu,uint8_t index);
void  Menu_Increase(Menu* menu,uint8_t index);
void  Menu_Decrease(Menu* menu,uint8_t index);
void  Menu_Show(Menu* menu,uint8_t key);
void Menu_Switch(Menu* menu, uint8_t menu_num);

#endif