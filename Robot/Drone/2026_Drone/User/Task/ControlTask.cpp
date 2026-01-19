#include "ControlTask.hpp"

Gimbal_FSM gimbal_fsm;

ALG::PID::PID yaw_angle_pid(7.0f, 0.0f, 0.00f, 25000.0f, 0.0f, 0.0f);
ALG::PID::PID yaw_velocity_pid(70.0f, 0.0f, 0.0f, 25000.0f, 0.0f, 0.0f);

ALG::PID::PID pitch_angle_pid(0.1f, 0.0f, 0.0f, 12.56f, 0.0f, 0.0f);
ALG::PID::PID pitch_velocity_pid(0.07f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);

ControlTask gimbal_target;
Output_gimbal gimbal_output;

void gimbal_fsm_init()
{
    gimbal_fsm.Init();
}

bool check_online()
{
    if(!Motor6020.isConnected(1,2)||!DR16.isConnected()||!MotorJ4310.isConnected(1,2))
    {
        return false;
    }
    return true;
}

void SetTarget_gimbal()
{
    gimbal_target.target_yaw += DR16.get_right_x();
    gimbal_target.target_pitch -= DR16.get_right_y();

    if(gimbal_target.target_yaw > 45.0f)
    {
        gimbal_target.target_yaw = 45.0f;
    }
    else if(gimbal_target.target_yaw < -45.0f)
    {
        gimbal_target.target_yaw = -45.0f;
    }

    if(gimbal_target.target_pitch > 33.0f)
    {
        gimbal_target.target_pitch = 33.0f;
    }
    else if(gimbal_target.target_pitch < -29.0f)
    {
        gimbal_target.target_pitch = -29.0f;
    }
}

void gimbal_stop()
{
    if(MotorJ4310.getIsenable())
    {
        MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(false);
    }
    MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
    MotorJ4310.setIsenable(false);

    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();

    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch = 0.0f;
}

void gimbal_manual()
{

    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(0x01,BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }
    gimbal_target.Test_data_1 = pitch_angle_pid.getOutput();
    gimbal_target.Test_data_2 = Motor6020.getVelocityRpm(2);
    float pitch_feedback = MotorJ4310.getAngleDeg(1);
    pitch_angle_pid.UpDate(gimbal_target.target_pitch, (MotorJ4310.getAngleDeg(1)/3.8f));
    pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRpm(1));

    float yaw_feedback = Motor6020.getAngleDeg(1);
    gimbal_target.normalized_feedback_yaw = yaw_feedback > 180.0f ? yaw_feedback - 360.0f : yaw_feedback;
    yaw_angle_pid.UpDate(gimbal_target.target_yaw, gimbal_target.normalized_feedback_yaw);
    yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(),Motor6020.getVelocityRpm(2));

    gimbal_output.out_yaw = yaw_velocity_pid.getOutput();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online)
{
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online);
    SetTarget_gimbal();

    switch (gimbal_fsm.Get_Now_State())
    {
        case STOP:
            gimbal_stop();
            break;
        case VISION:
            // Vision control logic here
            break;
        case MANUAL:
            gimbal_manual();
            break;
        case KEYBOARD:
            // Keyboard control logic here
            break;
        default:
            break;
    }
}
extern "C" void Control(void const * argument)
{
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
    MotorJ4310.setIsenable(false);
    //MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
    //MotorJ4310.Off(0x03,BSP::Motor::DM::MIT);
    gimbal_fsm_init();
    for(;;)
    {
        main_loop_gimbal(DR16.get_s1(), DR16.get_s2(), check_online());
        osDelay(1);
    }    
}