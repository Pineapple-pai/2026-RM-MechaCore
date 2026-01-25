#ifndef UI_DATA_BRIDGE_H
#define UI_DATA_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取底盘功率百分比 (0.0 - 100.0)
 */
float UI_Bridge_Get_Chassis_Power_Percent(void);

/**
 * @brief 获取超级电容电压百分比 (0.0 - 100.0)
 */
float UI_Bridge_Get_SuperCap_Percent(void);

/**
 * @brief 获取已发射弹丸数量
 */
int UI_Bridge_Get_Fired_Ammo_Count(void);

/**
 * @brief 获取底盘模式
 * 0: 无
 * 1: 普通 (云台跟随)
 * 2: 跟随 (底盘跟随)
 * 3: 小陀螺 (自旋)
 */
int UI_Bridge_Get_Chassis_Mode(void);

/**
 * @brief 获取超级电容状态
 * true: 开启
 * false: 关闭
 */
bool UI_Bridge_Get_SuperCap_Status(void);

/**
 * @brief 获取摩擦轮状态
 * true: 开启
 * false: 关闭
 */
bool UI_Bridge_Get_Friction_Status(void);

#ifdef __cplusplus
}
#endif

#endif // UI_DATA_BRIDGE_H
