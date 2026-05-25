#ifndef _MAVLINKAPP_H
#define _MAVLINKAPP_H

#include "main.h"
#include "mavlink.h"

// #define MAVLINK_TASK_DEBUG 1

typedef struct
{
    float odom_position_x;  // 里程计位置x
    float odom_position_y;  // 里程计位置y
    float odom_position_z;  // 里程计位置z

    float odom_velocity_x;  // 里程计速度x
    float odom_velocity_y;  // 里程计速度y
    float odom_velocity_z;  // 里程计速度z

    float odom_yaw_rad;     // 里程计航向角（弧度）
    float odom_yaw_deg;     // 里程计航向角（角度）

    float set_position_x;  // 控制位置x
    float set_position_y;  // 控制位置y
    float set_position_z;  // 控制位置z
    float set_yaw_rad;     // 控制Yaw
} MAVLINK_Structure;
extern MAVLINK_Structure mavlink_struct;

void MavLink_Init();

void send_heartbeat(void);
void send_rc_channels(void);

void MavLink_FSM(uint32_t tick);
void MavLinkAPP_Parse(void);
void Handle_MAVLink_Message(mavlink_message_t *msg);




#endif
