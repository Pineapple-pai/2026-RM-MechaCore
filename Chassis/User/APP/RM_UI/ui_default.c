//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_default.h"
#include "UI_Data_Bridge.h"
#include <stdio.h>

#define TOTAL_FIGURE 3 
#define TOTAL_STRING 8

static ui_interface_figure_t ui_default_figures[TOTAL_FIGURE];
static uint8_t ui_default_figures_dirty[TOTAL_FIGURE];

static ui_interface_string_t ui_default_strings[TOTAL_STRING];
static uint8_t ui_default_strings_dirty[TOTAL_STRING];

// 状态缓存
static int last_cap_percent = -1;
static int last_pwr_percent = -1;
static int last_ammo_count = -1;
static int last_mode = -1;
static int last_sup = -1;
static int last_fri = -1;

#define SCAN_AND_SEND() ui_scan_and_send(ui_default_figures, ui_default_figures_dirty, ui_default_strings, ui_default_strings_dirty, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_default() {
    memset(ui_default_figures, 0, sizeof(ui_default_figures));
    memset(ui_default_figures_dirty, 0, sizeof(ui_default_figures_dirty));
    memset(ui_default_strings, 0, sizeof(ui_default_strings));
    memset(ui_default_strings_dirty, 0, sizeof(ui_default_strings_dirty));

    // --- 图形 ---
    
    // 0. 超级电容圆弧 (左侧, 橙色)
    memcpy(ui_default_figures[0].figure_name, "cap", 3);
    ui_default_figures[0].operate_type = 1; // 添加
    ui_default_figures[0].figure_type = 5; // 圆弧
    ui_default_figures[0].layer = 1; 
    ui_default_figures[0].color = 3; // 橙色
    ui_default_figures[0].width = 5;
    ui_default_figures[0].start_x = 960; 
    ui_default_figures[0].start_y = 540; 
    ui_default_figures[0]._a = 135; // 起始角度
    ui_default_figures[0]._b = 135; // 结束角度
    ui_default_figures[0]._c = 0;
    ui_default_figures[0]._d = 150; // 半径 X
    ui_default_figures[0]._e = 150; // 半径 Y
    ui_default_figures_dirty[0] = 1;

    // 1. 功率圆弧 (右侧, 红色)
    memcpy(ui_default_figures[1].figure_name, "pwr", 3);
    ui_default_figures[1].operate_type = 1; 
    ui_default_figures[1].figure_type = 5; // 圆弧
    ui_default_figures[1].layer = 1; 
    ui_default_figures[1].color = 0; // 红色
    ui_default_figures[1].width = 5;
    ui_default_figures[1].start_x = 960; 
    ui_default_figures[1].start_y = 540; 
    ui_default_figures[1]._a = 45; // 起始
    ui_default_figures[1]._b = 45; // 结束
    ui_default_figures[1]._c = 0;
    ui_default_figures[1]._d = 150; 
    ui_default_figures[1]._e = 150; 
    ui_default_figures_dirty[1] = 1;

    // 2. 弹丸计数 (整数)
    memcpy(ui_default_figures[2].figure_name, "amo", 3);
    ui_default_figures[2].operate_type = 1; 
    ui_default_figures[2].figure_type = 6; // 整数
    ui_default_figures[2].layer = 1;
    ui_default_figures[2].color = 2; // 绿色
    ui_default_figures[2].width = 2; 
    ui_default_figures[2].start_x = 1750;
    ui_default_figures[2].start_y = 800;
    ui_default_figures[2]._a = 30; // 字号
    ui_default_figures[2]._b = 0; 
    {
        ui_interface_number_t* num = (ui_interface_number_t*)&ui_default_figures[2];
        num->number = 0;
        num->font_size = 30;
    }
    ui_default_figures_dirty[2] = 1;

    // --- 字符串 ---
    for(int i=0; i<TOTAL_STRING; i++) {
        ui_default_strings[i].operate_type = 1; // 添加
        ui_default_strings[i].figure_type = 7; // 字符串
        ui_default_strings[i].layer = 1;
        ui_default_strings[i].color = 8; // 白色
        ui_default_strings[i].font_size = 20;
        ui_default_strings[i].width = 2; 
        ui_default_strings_dirty[i] = 1;
    }

    // 0. 普通模式
    memcpy(ui_default_strings[0].figure_name, "nor", 3);
    ui_default_strings[0].start_x = 700;
    ui_default_strings[0].start_y = 300;
    strcpy(ui_default_strings[0].string, "Normal");
    ui_default_strings[0].str_length = strlen("Normal");

    // 1. 跟随模式
    memcpy(ui_default_strings[1].figure_name, "fol", 3);
    ui_default_strings[1].start_x = 850;
    ui_default_strings[1].start_y = 300;
    strcpy(ui_default_strings[1].string, "Follow");
    ui_default_strings[1].str_length = strlen("Follow");

    // 2. 小陀螺
    memcpy(ui_default_strings[2].figure_name, "rot", 3);
    ui_default_strings[2].start_x = 1000;
    ui_default_strings[2].start_y = 300;
    strcpy(ui_default_strings[2].string, "Spin");
    ui_default_strings[2].str_length = strlen("Spin");
    
    // 3. 键鼠模式
    memcpy(ui_default_strings[3].figure_name, "key", 3);
    ui_default_strings[3].start_x = 1100;
    ui_default_strings[3].start_y = 300;
    strcpy(ui_default_strings[3].string, "KeyBd");
    ui_default_strings[3].str_length = strlen("KeyBd");

    // 4. 停止模式
    memcpy(ui_default_strings[4].figure_name, "stp", 3);
    ui_default_strings[4].start_x = 1200;
    ui_default_strings[4].start_y = 300;
    strcpy(ui_default_strings[4].string, "Stop");
    ui_default_strings[4].str_length = strlen("Stop");

    // 5. 超级电容
    memcpy(ui_default_strings[5].figure_name, "sup", 3);
    ui_default_strings[5].start_x = 900;
    ui_default_strings[5].start_y = 700;
    strcpy(ui_default_strings[5].string, "Sup");
    ui_default_strings[5].str_length = strlen("Sup");

    // 6. 摩擦轮
    memcpy(ui_default_strings[6].figure_name, "fri", 3);
    ui_default_strings[6].start_x = 1000;
    ui_default_strings[6].start_y = 700;
    strcpy(ui_default_strings[6].string, "Fri");
    ui_default_strings[6].str_length = strlen("Fri");

    // 7. 发送标签
    memcpy(ui_default_strings[7].figure_name, "lab", 3);
    ui_default_strings[7].start_x = 1650;
    ui_default_strings[7].start_y = 800;
    strcpy(ui_default_strings[7].string, "Sent:");
    ui_default_strings[7].str_length = strlen("Sent:");

    SCAN_AND_SEND();
}

void ui_update_default() {
    bool data_updated = false;

    // 1. 电容圆弧
    float cap_p = UI_Bridge_Get_SuperCap_Percent(); 
    if ((int)cap_p != last_cap_percent) {
        last_cap_percent = (int)cap_p;
        ui_default_figures[0]._b = 135 + (int)(cap_p * 0.9f); 
        ui_default_figures[0].operate_type = 2; // 修改
        ui_default_figures_dirty[0] = 1;
        data_updated = true;
    }

    // 2. 功率圆弧
    float pwr_p = UI_Bridge_Get_Chassis_Power_Percent();
    if ((int)pwr_p != last_pwr_percent) {
        last_pwr_percent = (int)pwr_p;
        int end = 45 - (int)(pwr_p * 0.9f);
        if (end < 0) end += 360;
        ui_default_figures[1]._b = end;
        ui_default_figures[1].operate_type = 2;
        ui_default_figures_dirty[1] = 1;
        data_updated = true;
    }

    // 3. 弹丸数
    int ammo = UI_Bridge_Get_Fired_Ammo_Count();
    if (ammo != last_ammo_count) {
        last_ammo_count = ammo;
        ui_interface_number_t* num = (ui_interface_number_t*)&ui_default_figures[2];
        num->number = ammo;
        ui_default_figures[2].operate_type = 2;
        ui_default_figures_dirty[2] = 1;
        data_updated = true;
    }

    // 4. 模式
    int mode = UI_Bridge_Get_Chassis_Mode(); 
    
    if (mode != last_mode) {
        last_mode = mode;
        // 重置所有为白色
        for(int i=0; i<5; i++) {
            ui_default_strings[i].color = 8; // 白色
            ui_default_strings[i].operate_type = 2;
            ui_default_strings_dirty[i] = 1;
        }
        // 高亮当前模式
        int target = -1;
        if (mode == 1) target = 0; // 普通
        else if (mode == 2) target = 1; // 跟随
        else if (mode == 3) target = 2; // 小陀螺
        else if (mode == 4) target = 3; // 键鼠
        else if (mode == 5) target = 4; // 停止
        
        if (target != -1) {
            ui_default_strings[target].color = 2; // 绿色
            ui_default_strings_dirty[target] = 1;
        }
        data_updated = true;
    }

    // 5. 超级电容/摩擦轮
    bool sup = UI_Bridge_Get_SuperCap_Status();
    if (sup != (last_sup == 1)) {
        last_sup = sup ? 1 : 0;
        ui_default_strings[5].color = sup ? 1 : 8; // 开启为黄色，关闭为白色
        ui_default_strings[5].operate_type = 2;
        ui_default_strings_dirty[5] = 1;
        data_updated = true;
    }

    bool fri = UI_Bridge_Get_Friction_Status();
    if (fri != (last_fri == 1)) {
        last_fri = fri ? 1 : 0;
        ui_default_strings[6].color = fri ? 1 : 8; // 开启为黄色
        ui_default_strings[6].operate_type = 2;
        ui_default_strings_dirty[6] = 1;
        data_updated = true;
    }

    SCAN_AND_SEND();
}
