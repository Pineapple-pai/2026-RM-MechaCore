#include "UI_Data_Bridge.h"
#include "../../BSP/Power/PM01.hpp"
#include "../../BSP/SuperCap/SuperCap.hpp"
#include "../../Task/CommunicationTask.hpp"

extern "C" {

float UI_Bridge_Get_Chassis_Power_Percent(void) {
    if (BSP::Power::pm01.lim_cin_power <= 0.1f) return 0.0f;
    float percent = (BSP::Power::pm01.cin_power / BSP::Power::pm01.lim_cin_power) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

float UI_Bridge_Get_SuperCap_Percent(void) {
    // 逻辑改编自 ui_ROT.cpp
    // 映射 16V-18V 到 0-100%
    float voltage = BSP::SuperCap::cap.getCapVoltage();
    float percent = 0.0f;
    
    if (voltage > 16.0f) {
        if (voltage > 18.0f) {
            percent = 100.0f;
        } else {
             percent = (voltage - 16.0f) * 50.0f; // (v-16)/2 * 100
        }
    } else {
        percent = 0.0f;
    }
    
    return percent;
}

int UI_Bridge_Get_Fired_Ammo_Count(void) {
    return Gimbal_to_Chassis_Data.getProjectileCount();
}

int UI_Bridge_Get_Chassis_Mode(void) {
    if (Gimbal_to_Chassis_Data.getUniversal()) return 1;
    if (Gimbal_to_Chassis_Data.getFollow()) return 2;
    if (Gimbal_to_Chassis_Data.getRotating()) return 3;
    if (Gimbal_to_Chassis_Data.getKeyBoard()) return 4;
    if (Gimbal_to_Chassis_Data.getStop()) return 5;
    return 0;
}

bool UI_Bridge_Get_SuperCap_Status(void) {
    // 检查电容是否开启
    // BSP::SuperCap::cap.getCapSwitch() 返回 Enum Switch::ENABLE 或 DISABLE
    return (BSP::SuperCap::cap.getCapSwitch() == BSP::SuperCap::LH_Cap::Switch::ENABLE);
}

bool UI_Bridge_Get_Friction_Status(void) {
    return Gimbal_to_Chassis_Data.getFrictionEnabled();
}

}
