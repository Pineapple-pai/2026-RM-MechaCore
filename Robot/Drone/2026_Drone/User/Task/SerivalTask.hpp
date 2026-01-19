#ifndef SERIVALTASK_HPP
#define SERIVALTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/BSP/RemoteControl/DT7.hpp"
#include "../core/HAL/UART/uart_hal.hpp"
#include "../core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../core/BSP/Motor/DM/DmMotor.hpp"

typedef struct
{
    float target_velocity;
    float target_yaw;   
    float target_pitch;
    float target_dial;
    float target_surgewheel[2];
    float normalized_feedback_yaw;
    float normalized_feedback_pitch;
    float Test_data_1;
    float Test_data_2;
    float Test_data_3;
}ControlTask;

typedef struct
{
    float out_yaw;
    float out_pitch;
}Output_gimbal;

extern ControlTask gimbal_target;

extern BSP::REMOTE_CONTROL::RemoteController DR16;
extern uint8_t dr16_rx_buffer[18];

//extern BSP::Motor::Dji::GM6020<1> Motor6020;

#endif // DEBUG