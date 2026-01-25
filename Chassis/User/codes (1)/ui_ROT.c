//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include <stdio.h>
#include "ui_interface.h"
#include "ui_ROT.h"
#include "../APP/RM_UI/UI_Data_Bridge.h"

#define TOTAL_FIGURE 4
#define TOTAL_STRING 6

ui_interface_figure_t ui_ROT_now_figures[TOTAL_FIGURE];
uint8_t ui_ROT_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_now_strings[TOTAL_STRING];
uint8_t ui_ROT_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_ROT_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_ROT_now_figures, ui_ROT_dirty_figure, ui_ROT_now_strings, ui_ROT_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_ROT() {
    ui_ROT_Ungroup_Function2->figure_type = 1;
    ui_ROT_Ungroup_Function2->operate_type = 1;
    ui_ROT_Ungroup_Function2->layer = 0;
    ui_ROT_Ungroup_Function2->color = 8;
    ui_ROT_Ungroup_Function2->start_x = 761;
    ui_ROT_Ungroup_Function2->start_y = 170;
    ui_ROT_Ungroup_Function2->width = 1;
    ui_ROT_Ungroup_Function2->end_x = 1161;
    ui_ROT_Ungroup_Function2->end_y = 270;

    ui_ROT_Gimbal_Function1->figure_type = 1;
    ui_ROT_Gimbal_Function1->operate_type = 1;
    ui_ROT_Gimbal_Function1->layer = 0;
    ui_ROT_Gimbal_Function1->color = 8;
    ui_ROT_Gimbal_Function1->start_x = 661;
    ui_ROT_Gimbal_Function1->start_y = 70;
    ui_ROT_Gimbal_Function1->width = 1;
    ui_ROT_Gimbal_Function1->end_x = 1261;
    ui_ROT_Gimbal_Function1->end_y = 170;

    ui_ROT_Chassis_SC->figure_type = 4;
    ui_ROT_Chassis_SC->operate_type = 1;
    ui_ROT_Chassis_SC->layer = 0;
    ui_ROT_Chassis_SC->color = 3;
    ui_ROT_Chassis_SC->start_x = 854;
    ui_ROT_Chassis_SC->start_y = 539;
    ui_ROT_Chassis_SC->width = 1;
    ui_ROT_Chassis_SC->start_angle = 220;
    ui_ROT_Chassis_SC->end_angle = 320;
    ui_ROT_Chassis_SC->rx = 286;
    ui_ROT_Chassis_SC->ry = 352;

    ui_ROT_Chassis_Power->figure_type = 4;
    ui_ROT_Chassis_Power->operate_type = 1;
    ui_ROT_Chassis_Power->layer = 0;
    ui_ROT_Chassis_Power->color = 0;
    ui_ROT_Chassis_Power->start_x = 1064;
    ui_ROT_Chassis_Power->start_y = 539;
    ui_ROT_Chassis_Power->width = 1;
    ui_ROT_Chassis_Power->start_angle = 40;
    ui_ROT_Chassis_Power->end_angle = 140;
    ui_ROT_Chassis_Power->rx = 286;
    ui_ROT_Chassis_Power->ry = 352;

    ui_ROT_Ungroup_Super->figure_type = 7;
    ui_ROT_Ungroup_Super->operate_type = 1;
    ui_ROT_Ungroup_Super->layer = 1;
    ui_ROT_Ungroup_Super->color = 2;
    ui_ROT_Ungroup_Super->start_x = 830;
    ui_ROT_Ungroup_Super->start_y = 221;
    ui_ROT_Ungroup_Super->width = 2;
    ui_ROT_Ungroup_Super->font_size = 20;
    ui_ROT_Ungroup_Super->str_length = 3;
    strcpy(ui_ROT_Ungroup_Super->string, "Sup");

    ui_ROT_Ungroup_Fri->figure_type = 7;
    ui_ROT_Ungroup_Fri->operate_type = 1;
    ui_ROT_Ungroup_Fri->layer = 1;
    ui_ROT_Ungroup_Fri->color = 2;
    ui_ROT_Ungroup_Fri->start_x = 1031;
    ui_ROT_Ungroup_Fri->start_y = 221;
    ui_ROT_Ungroup_Fri->width = 2;
    ui_ROT_Ungroup_Fri->font_size = 20;
    ui_ROT_Ungroup_Fri->str_length = 3;
    strcpy(ui_ROT_Ungroup_Fri->string, "Fri");

    ui_ROT_Shoot_leave->figure_type = 7;
    ui_ROT_Shoot_leave->operate_type = 1;
    ui_ROT_Shoot_leave->layer = 1;
    ui_ROT_Shoot_leave->color = 8;
    ui_ROT_Shoot_leave->start_x = 1456;
    ui_ROT_Shoot_leave->start_y = 735;
    ui_ROT_Shoot_leave->width = 3;
    ui_ROT_Shoot_leave->font_size = 26;
    ui_ROT_Shoot_leave->str_length = 12;
    strcpy(ui_ROT_Shoot_leave->string, "已发射：");

    ui_ROT_Gimbal_Normal->figure_type = 7;
    ui_ROT_Gimbal_Normal->operate_type = 1;
    ui_ROT_Gimbal_Normal->layer = 1;
    ui_ROT_Gimbal_Normal->color = 2;
    ui_ROT_Gimbal_Normal->start_x = 730;
    ui_ROT_Gimbal_Normal->start_y = 126;
    ui_ROT_Gimbal_Normal->width = 2;
    ui_ROT_Gimbal_Normal->font_size = 22;
    ui_ROT_Gimbal_Normal->str_length = 3;
    strcpy(ui_ROT_Gimbal_Normal->string, "Nor");

    ui_ROT_Gimbal_Follow->figure_type = 7;
    ui_ROT_Gimbal_Follow->operate_type = 1;
    ui_ROT_Gimbal_Follow->layer = 1;
    ui_ROT_Gimbal_Follow->color = 2;
    ui_ROT_Gimbal_Follow->start_x = 1130;
    ui_ROT_Gimbal_Follow->start_y = 124;
    ui_ROT_Gimbal_Follow->width = 2;
    ui_ROT_Gimbal_Follow->font_size = 22;
    ui_ROT_Gimbal_Follow->str_length = 3;
    strcpy(ui_ROT_Gimbal_Follow->string, "FOL");

    ui_ROT_Gimbal_Rotating->figure_type = 7;
    ui_ROT_Gimbal_Rotating->operate_type = 1;
    ui_ROT_Gimbal_Rotating->layer = 1;
    ui_ROT_Gimbal_Rotating->color = 2;
    ui_ROT_Gimbal_Rotating->start_x = 930;
    ui_ROT_Gimbal_Rotating->start_y = 124;
    ui_ROT_Gimbal_Rotating->width = 2;
    ui_ROT_Gimbal_Rotating->font_size = 22;
    ui_ROT_Gimbal_Rotating->str_length = 3;
    strcpy(ui_ROT_Gimbal_Rotating->string, "ROT");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
#endif
        ui_ROT_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
#endif
        ui_ROT_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].operate_type = 2;
    }
}

void ui_update_ROT() {
    // --- Update Logic Start ---
    
    // 1. Chassis Power (Right Red Arc, Limit 40-140)
    float power_pct = UI_Bridge_Get_Chassis_Power_Percent();
    int power_angle = 40 + (int)power_pct;
    if (power_angle > 140) power_angle = 140;
    ui_ROT_Chassis_Power->end_angle = power_angle;

    // 2. SuperCap (Left Orange Arc, Limit 220-320)
    float cap_pct = UI_Bridge_Get_SuperCap_Percent();
    int cap_angle = 220 + (int)cap_pct;
    if (cap_angle > 320) cap_angle = 320;
    ui_ROT_Chassis_SC->end_angle = cap_angle;

    // 3. Ammo Count (Text)
    // Note: Use sprintf with caution on length. struct has 30 chars.
    sprintf(ui_ROT_Shoot_leave->string, "已发射:%d", UI_Bridge_Get_Fired_Ammo_Count());

    // 4. Chassis Mode Indicators (NOR, FOL, ROT)
    int mode = UI_Bridge_Get_Chassis_Mode();
    // Reset all to Green (2)
    ui_ROT_Gimbal_Normal->color = 2;
    ui_ROT_Gimbal_Follow->color = 2;
    ui_ROT_Gimbal_Rotating->color = 2;
    
    if (mode == 1) ui_ROT_Gimbal_Normal->color = 1; // Yellow
    else if (mode == 2) ui_ROT_Gimbal_Follow->color = 1;
    else if (mode == 3) ui_ROT_Gimbal_Rotating->color = 1;

    // 5. Functional Indicators (SUP, FRI)
    bool sup = UI_Bridge_Get_SuperCap_Status();
    bool fri = UI_Bridge_Get_Friction_Status();
    
    ui_ROT_Ungroup_Super->color = sup ? 1 : 2; // Yellow if active
    ui_ROT_Ungroup_Fri->color = fri ? 1 : 2;   // Yellow if active

    // --- Update Logic End ---


#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_ROT_now_figures[i], &ui_ROT_last_figures[i], sizeof(ui_ROT_now_figures[i])) != 0) {
            ui_ROT_dirty_figure[i] = 1;
            ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_ROT_now_strings[i], &ui_ROT_last_strings[i], sizeof(ui_ROT_now_strings[i])) != 0) {
            ui_ROT_dirty_string[i] = 1;
            ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
