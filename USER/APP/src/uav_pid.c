#include "uav_pid.h"
#include "MavLinkAPP.h"
#include "arm_math.h"


UAV_PID_HandleTypeDef huav_pid = {0};

// 角度归一化
static float fast_normalize_angle(float angle)
{
    // 先处理到[-2π, 2π]范围
    angle = fmod(-angle, 2.0f * M_PI);
    
    // 转换到[0, 2π]
    return angle >= 0 ? angle : angle + 2.0f * M_PI;
}

// 初始化PID控制器参数
void uav_pid_init(void)
{
    pid_init(&huav_pid.PositionPID[0], PID_XY_POSITION_P, PID_XY_POSITION_I, PID_XY_POSITION_D);
    pid_init(&huav_pid.PositionPID[1], PID_XY_POSITION_P, PID_XY_POSITION_I, PID_XY_POSITION_D);
    pid_init(&huav_pid.SpeedPID[0], PID_XY_VELOCITY_P, PID_XY_VELOCITY_I, PID_XY_VELOCITY_D);
    pid_init(&huav_pid.SpeedPID[1], PID_XY_VELOCITY_P, PID_XY_VELOCITY_I, PID_XY_VELOCITY_D);

    pid_init(&huav_pid.PositionPID[2], PID_Z_POSITION_P, PID_Z_POSITION_I, PID_Z_POSITION_D);
    pid_init(&huav_pid.SpeedPID[2], PID_Z_VELOCITY_P, PID_Z_VELOCITY_I, PID_Z_VELOCITY_D);
}

// 定时器更新(5ms,200Hz)，更新PID控制器
void uav_pid_update(void)
{
    static uint32_t pos_counter = 0;

    float cw = fast_normalize_angle(mavlink_struct.odom_yaw_rad);

    huav_pid.PositionPID[0].SetPoint = mavlink_struct.set_position_x; // 期望位置X
    huav_pid.PositionPID[1].SetPoint = mavlink_struct.set_position_y; // 期望位置Y
    huav_pid.PositionPID[2].SetPoint = mavlink_struct.set_position_z; // 期望位置Z

    // 从外部更新位置和速度反馈值
    huav_pid.PositionFeedback[0] = mavlink_struct.odom_position_x;
    huav_pid.PositionFeedback[1] = mavlink_struct.odom_position_y;
    huav_pid.PositionFeedback[2] = mavlink_struct.odom_position_z;
    huav_pid.SpeedFeedback[0]    = mavlink_struct.odom_velocity_x;
    huav_pid.SpeedFeedback[1]    = mavlink_struct.odom_velocity_y;
    huav_pid.SpeedFeedback[2]    = mavlink_struct.odom_velocity_z;

    if(++pos_counter >= 2) // 100Hz
    {
        // 外环(位置环)不需要积分
        huav_pid.PositionOutput[0] = positional_pid_ctrl(&huav_pid.PositionPID[0], huav_pid.PositionFeedback[0],0.0f,0.0f);
        huav_pid.PositionOutput[1] = positional_pid_ctrl(&huav_pid.PositionPID[1], huav_pid.PositionFeedback[1],0.0f,0.0f);
        huav_pid.PositionOutput[2] = positional_pid_ctrl(&huav_pid.PositionPID[2], huav_pid.PositionFeedback[2],200.0f,0.3f);

        // 将位置PID输出作为速度PID的期望输入
        // 输出限幅，防止过大的控制信号
        if(huav_pid.PositionOutput[0] >=  1.5f) huav_pid.PositionOutput[0] =  1.5f;
        if(huav_pid.PositionOutput[0] <= -1.5f) huav_pid.PositionOutput[0] = -1.5f;
        if(huav_pid.PositionOutput[1] >=  1.5f) huav_pid.PositionOutput[1] =  1.5f;
        if(huav_pid.PositionOutput[1] <= -1.5f) huav_pid.PositionOutput[1] = -1.5f;
        // if(huav_pid.PositionOutput[2] >=  1.5f) huav_pid.PositionOutput[2] =  1.5f;
        // if(huav_pid.PositionOutput[2] <= -1.5f) huav_pid.PositionOutput[2] = -1.5f;
    
        huav_pid.SpeedPID[0].SetPoint = huav_pid.PositionOutput[0];    // 期望速度X
        huav_pid.SpeedPID[1].SetPoint = huav_pid.PositionOutput[1];    // 期望速度Y
        huav_pid.SpeedPID[2].SetPoint = huav_pid.PositionOutput[2];    // 期望速度Z
        pos_counter = 0;
    }
    
    // huav_pid.SpeedPID[0].SetPoint = 0.4;
    huav_pid.SpeedPID[2].SetPoint = 0.4;
    // 速度环计算
    float sx_out = positional_pid_ctrl(&huav_pid.SpeedPID[0], huav_pid.SpeedFeedback[0], 200.0f, 0.5f);
    // 输出限幅，防止过大的控制信号
    if(sx_out >=  200.0f) sx_out =  200.0f;
    if(sx_out <= -200.0f) sx_out = -200.0f;

    float sy_out = positional_pid_ctrl(&huav_pid.SpeedPID[1], huav_pid.SpeedFeedback[1], 200.0f, 0.2f);
    // 输出限幅，防止过大的控制信号
    if(sy_out >=  200.0f) sy_out =  200.0f;
    if(sy_out <= -200.0f) sy_out = -200.0f;

    float sz_out = positional_pid_ctrl(&huav_pid.SpeedPID[2], huav_pid.SpeedFeedback[2], 200.0f, 0.5f);
    // 输出限幅，防止过大的控制信号
    if(sz_out >=  200.0f) sz_out =  200.0f;
    if(sz_out <= -200.0f) sz_out = -200.0f;

    float Fx=0,Fy=0;
    float cos_cw = arm_cos_f32(cw);
    float sin_cw = arm_sin_f32(cw);
    Fx = cos_cw * sx_out - sin_cw * sy_out;
    Fy = cos_cw * sy_out + sin_cw * sx_out;

    // // 最终赋值
    // huav_pid.SpeedOutput[0] = Fx;
    // huav_pid.SpeedOutput[1] = Fy;
    // huav_pid.SpeedOutput[2] = sz_out;

    // 最终赋值
    huav_pid.SpeedOutput[0] = Fx;
    huav_pid.SpeedOutput[1] = Fy;
    huav_pid.SpeedOutput[2] = huav_pid.PositionOutput[2];
    if(huav_pid.SpeedOutput[2] > 0)
    {
        if (huav_pid.SpeedOutput[2] < 60.0f)
        {
            huav_pid.SpeedOutput[2] = 60.0f;
        }
    }
    else if(huav_pid.SpeedOutput[2] < 0)
    {
        if (huav_pid.SpeedOutput[2] > -60.0f)
        {
            huav_pid.SpeedOutput[2] = -60.0f;
        }
    }

    float YAW = mavlink_struct.set_yaw_rad;
    // 简单控制Yaw
    double error = -YAW - mavlink_struct.odom_yaw_rad;
    // 归一化误差到[-pi, pi]
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    // 添加2.5、5、10度（分别为约0.0436、0.0873、0.1745弧度）的判断
    const double threshold_2p5 = 1.5 * M_PI / 180.0;
    const double threshold_5 = 5.0 * M_PI / 180.0;
    const double threshold_10 = 20.0 * M_PI / 180.0;
    int16_t temp = 0;

    if (error >= -threshold_2p5 && error <= threshold_2p5)
    {
        temp = 40;
    }
    else if (error > threshold_2p5 && error <= threshold_5)
    {
        temp = 60;
    }
    else if (error < -threshold_2p5 && error >= -threshold_5)
    {
        temp = -60;
    }
    else if (error > threshold_5 && error <= threshold_10)
    {
        temp = 120;
    }
    else if (error < -threshold_5 && error >= -threshold_10)
    {
        temp = -120;
    }
    else if (error > threshold_10)
    {
        temp = 445;
    }
    else // error < -threshold_10
    {
        temp = -445;
    }

    huav_pid.YawOutput = temp;
}

