#include "User/Task/Inc/motorPidTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

void MotorPidTask(void *argument)
{
    static TASK::MotorPid::MotorPid motorPid;

    for (;;)
    {
        motorPid.UpData();  // 修正函数名
        osDelay(MOTOR_CONTROL_HZ);
    }
}

namespace TASK::MotorPid
{
MotorPid::MotorPid()
    : // 位置pid - 提供完整的6个参数
      pid_pos(0.0f, 0.0f, 0.0f, 1000.0f, 100.0f, 5.0f),
      // 速度pid - 提供完整的6个参数
      pid_vel(0.0f, 0.0f, 0.0f, 1000.0f, 100.0f, 5.0f)
      // 注释掉adrc_vel，因为没有找到对应的类定义
      // adrc_vel(ALG::LADRC::TDquadratic(100, 0.001), 0.0, 0.0, 0.0, 0.001f, 0.0)
{
    // 获取CAN设备实例
    auto &chassis_can = CAN_INSTANCE.get_device(CHASSIS_CAN);
    // 注册电机的Parse函数作为CAN接收回调
    Motor6020.registerCallback(&chassis_can);
}

// 修正函数名 UpDate -> UpData
void MotorPid::UpData()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

    switch (Now_Status_Serial)
    {
    case (MotorPid_Status::DISABLE): {
        Disable();

        break;
    }
    case (MotorPid_Status::PID): {
        PidUpData();  // 修正函数名

        break;
    }
    case (MotorPid_Status::ADRC): {
        AdrcUpData();  // 修正函数名

        break;
    }
    }

    sendCan();
}

void MotorPid::Disable(void)
{
    // 读取当前状态作为目标
    target_pos = Motor6020.getAngleDeg(2);
    target_vel = Motor6020.getVelocityRads(2);

    // 目标跟随当前值，控制器输出为0
    // 位置环：目标位置 -> 期望速度
    float current_pos = Motor6020.getAngleDeg(2);
    pid_pos.setTarget(target_pos);
    pid_pos.UpDate(target_pos, current_pos);  // 提供target和feedback两个参数

    // 速度环：期望速度 -> 控制量
    float current_vel = Motor6020.getVelocityRads(2);
    pid_vel.setTarget(pid_pos.getOutput());
    pid_vel.UpDate(pid_pos.getOutput(), current_vel);  // 提供target和feedback两个参数
}

void MotorPid::PidUpData()  // 修正函数名
{
    // 读取电机反馈
    float cur_pos = Motor6020.getAngleDeg(2);
    float cur_vel = Motor6020.getVelocityRads(2);

    // 位置环：目标位置 -> 期望速度
    pid_pos.setTarget(target_pos);
    pid_pos.UpDate(target_pos, cur_pos);  // 提供target和feedback两个参数

    // 速度环：期望速度 -> 控制量
    pid_vel.setTarget(pid_pos.getOutput());
    pid_vel.UpDate(pid_pos.getOutput(), cur_vel);  // 提供target和feedback两个参数
}

void MotorPid::AdrcUpData()  // 修正函数名
{
    // 读取电机反馈
    float cur_pos = Motor6020.getAngleDeg(2);
    float cur_vel = Motor6020.getVelocityRads(2);

    // 位置环：目标位置 -> 期望速度
    pid_pos.setTarget(target_pos);
    pid_pos.UpDate(target_pos, cur_pos);  // 提供target和feedback两个参数

    // 速度环(ADRC)：期望速度 -> 控制量
    // 由于缺少ADRC类定义，暂时使用PID代替
    pid_vel.setTarget(pid_pos.getOutput());
    pid_vel.UpDate(pid_pos.getOutput(), cur_vel);  // 提供target和feedback两个参数
}

void MotorPid::sendCan(void)
{
    // 发送数据 - 不再需要传递参数，由sendCAN内部处理
    Motor6020.sendCAN(0);  // 传递默认邮箱参数
}
} // namespace TASK::MotorPid