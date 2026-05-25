#include "MavLinkAPP.h"
#include "mavlink.h"
#include "mavlink_msg_rc_channels_raw.h"
#include "usart.h"
#include "sbus_task.h"
#include "ring_buffer.h"
#include "system.h"
#include "usbd_cdc_if.h"
#include "usart_it.h"
#include "LowPassFilter.h"


MAVLINK_Structure mavlink_struct = 
{
    0
};

uint8_t mavlink_buf[MAVLINK_MAX_PACKET_LEN];

#define FLIGHT_CONTROLLER_SYS_ID    1   // 飞控系统ID
#define FLIGHT_CONTROLLER_COMP_ID   1   // 飞控组件ID

// ==================== 参数表 ====================
typedef struct {
    const char* name;
    const float value;
} param_entry_t;

const param_entry_t params[] = {
    {"SYS_AUTOSTART", 4001.0f},
    {"CAL_GYRO0_ID", 0.0f},
    {"COM_RC_IN_MODE", 0.0f},
    {"RC_MAP_ROLL", 0.0f},
    {"SYS_AUTOCONFIG", 0.0f},
    {"MAV_SYS_ID", 1.0f},
    {"CAL_ACC0_ID", 0.0f},
    {"CAL_MAG2_ID", 0.0f},
    {"CAL_MAG1_ID", 0.0f},
    {"CAL_MAG0_ID", 0.0f},
    {"RC_MAP_AUX2", 7.0f},
    {"RC_MAP_AUX1", 6.0f},
    {"RC_MAP_FLAPS", 5.0f},
    {"RC_MAP_THROTTLE", 3.0f},
    {"RC_MAP_YAW", 2.0f},
    {"RC_MAP_PITCH", 1.0f},
    {"COM_FLTMODE1", 0.0f},
    {"COM_FLTMODE2", 1.0f},
    {"COM_FLTMODE3", 2.0f},
    {"COM_FLTMODE4", 3.0f},
    {"COM_FLTMODE5", 4.0f},
    {"COM_FLTMODE6", 5.0f},
    {"BAT1_N_CELLS", 4.0f},
    {"BAT1_V_EMPTY", 13.2f},
    {"BAT1_V_CHARGED", 16.8f},
    {"RTL_LAND_DELAY", -1.0f},
    {"RTL_DESCEND_ALT", 15.0f},
    {"RTL_RETURN_ALT", 30.0f},
    {"NAV_DLL_ACT", 1.0f},
    {"COM_RC_LOSS_T", 5.0f},
    {"NAV_RCL_ACT", 1.0f},
    {"COM_LOW_BAT_ACT", 1.0f}
};

uint16_t param_count = sizeof(params) / sizeof(params[0]);

// ==================== 函数声明 ====================
void send_heartbeat(void);
void send_command_ack(uint16_t command, uint8_t result, uint8_t target_sysid, uint8_t target_compid);
void send_rc_channels(void);
void send_home_position(void);
void send_autopilot_version(uint8_t target_system, uint8_t target_component);
void send_mission_count(uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type);
void send_param_value(const char* param_name, float param_value, uint16_t param_index);


// 创建低通滤波器实例
LowPassFilterStructure odom_pos_x_filter;
LowPassFilterStructure odom_pos_y_filter;
LowPassFilterStructure odom_pos_z_filter;

LowPassFilterStructure odom_vel_x_filter;
LowPassFilterStructure odom_vel_y_filter;
LowPassFilterStructure odom_vel_z_filter;

void MavLink_Init()
{
    // 滤波对象实例化
    odom_pos_x_filter = LowPassFilter_Create(ODOM_POSITION_FILTER, 0.0f);
    odom_pos_y_filter = LowPassFilter_Create(ODOM_POSITION_FILTER, 0.0f);
    odom_pos_z_filter = LowPassFilter_Create(ODOM_POSITION_FILTER, 0.0f);

    odom_vel_x_filter = LowPassFilter_Create(ODOM_VELOCITY_FILTER, 0.0f);
    odom_vel_y_filter = LowPassFilter_Create(ODOM_VELOCITY_FILTER, 0.0f);
    odom_vel_z_filter = LowPassFilter_Create(ODOM_VELOCITY_FILTER, 0.0f);
}

// mavlink发送函数
void mavlink_send(uint8_t *buf, uint16_t len)
{
    // 使用USB发送
    CDC_Transmit_FS(buf, len);
}

// 发送心跳消息
void send_heartbeat(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_PX4,
        29,
        50593792,
        MAV_STATE_STANDBY
    );

    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送命令应答函数
void send_command_ack(uint16_t command, uint8_t result, uint8_t target_sysid, uint8_t target_compid)
{
    mavlink_message_t msg;
    
    mavlink_msg_command_ack_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        command,
        result,
        0,
        0,
        target_sysid,
        target_compid
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送RC通道消息
void send_rc_channels(void)
{
    mavlink_message_t msg;
    
    mavlink_msg_rc_channels_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        HAL_GetTick(),
        18,
        sbus_encode_frame_t.channels[0],
        sbus_encode_frame_t.channels[1],
        sbus_encode_frame_t.channels[2],
        sbus_encode_frame_t.channels[3],
        sbus_encode_frame_t.channels[4],
        sbus_encode_frame_t.channels[5],
        sbus_encode_frame_t.channels[6],
        sbus_encode_frame_t.channels[7],
        sbus_encode_frame_t.channels[8],
        sbus_encode_frame_t.channels[9],
        sbus_encode_frame_t.channels[10],
        sbus_encode_frame_t.channels[11],
        sbus_encode_frame_t.channels[12],
        sbus_encode_frame_t.channels[13],
        sbus_encode_frame_t.channels[14],
        sbus_encode_frame_t.channels[15],
        sbus_encode_frame_t.channels[16],
        sbus_encode_frame_t.channels[17],
        66
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送Home位置信息
void send_home_position(void)
{
    mavlink_message_t msg;
    
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    
    mavlink_msg_home_position_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        0.0,
        0.0,
        0.0,
        0.0f,
        0.0f,
        0.0f,
        q,
        0.0f,
        0.0f,
        0.0f,
        HAL_GetTick() * 1000ULL
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送飞控能力信息
void send_autopilot_version(uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t msg;
    
    uint64_t capabilities = 
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
        MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
        MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
        MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
        MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
        MAV_PROTOCOL_CAPABILITY_TERRAIN;
    
    uint32_t flight_sw_version = 0x010D0000;
    uint32_t middleware_sw_version = 0x010D0000;
    uint32_t os_sw_version = 0x0A030000;
    uint32_t board_version = 0x01010000;
    
    uint8_t flight_custom_version[8] = {0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90};
    uint8_t middleware_custom_version[8] = {0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90};
    uint8_t os_custom_version[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    
    uint16_t vendor_id = 0x2DAE;
    uint16_t product_id = 0x1011;
    uint64_t uid = 0x3031000030303FULL;
    uint8_t uid2[18] = {0};
    
    mavlink_msg_autopilot_version_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        capabilities,
        flight_sw_version,
        middleware_sw_version,
        os_sw_version,
        board_version,
        flight_custom_version,
        middleware_custom_version,
        os_custom_version,
        vendor_id,
        product_id,
        uid,
        uid2
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送航点数量
void send_mission_count(uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
{
    mavlink_message_t msg;
    
    mavlink_msg_mission_count_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        target_system,
        target_component,
        count,
        mission_type,
        0
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// 发送单个参数值
void send_param_value(const char* param_name, float param_value, uint16_t param_index)
{
    mavlink_message_t msg;
    
    mavlink_msg_param_value_pack(
        FLIGHT_CONTROLLER_SYS_ID,
        FLIGHT_CONTROLLER_COMP_ID,
        &msg,
        param_name,
        param_value,
        MAV_PARAM_TYPE_REAL32,
        param_index,    // 当前参数索引
        param_count     // 总参数数量
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &msg);
    mavlink_send(mavlink_buf, len);
}

// MAVLink状态机
void MavLink_FSM(uint32_t tick)
{
    static uint32_t send_heartbeat_tick = 0;
    static uint32_t send_rc_channels_tick = 0;

    // 1000ms心跳包
    if(get_tick_diff(tick, send_heartbeat_tick) >= 1000)
    {
        send_heartbeat_tick = tick;
        
        send_heartbeat();
    }

    // 100ms遥控通道数据
    if(get_tick_diff(tick, send_rc_channels_tick) >= 100)
    {
        send_rc_channels_tick = tick;
        
        send_rc_channels();
    }

    MavLinkAPP_Parse(); // 消息解析
}
// ==================== MAVLink解析函数 ====================

void MavLinkAPP_Parse(void)
{
    uint8_t byte;
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (RingBuff_ReadByte(&USBRxRingBufferStruct, &byte))
    {
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status);

        if (result == MAVLINK_FRAMING_OK)
        {
            Handle_MAVLink_Message(&msg);
        }
    }
}

// ==================== 消息处理函数 ====================

void Handle_MAVLink_Message(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {     
        case MAVLINK_MSG_ID_HEARTBEAT:  // 心跳消息
        {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
        }
        break;

        case MAVLINK_MSG_ID_SYSTEM_TIME:    // 系统时间消息
        {
            mavlink_system_time_t system_time;
            mavlink_msg_system_time_decode(msg, &system_time);
        }
        break;

        case MAVLINK_MSG_ID_TIMESYNC:   // 时间同步消息
        {
            mavlink_timesync_t timesync;
            mavlink_msg_timesync_decode(msg, &timesync);
            
            if (timesync.tc1 == 0)
            {
                // 收到时间同步请求，需要回复
                mavlink_message_t resp_msg;
                uint64_t current_time_us = HAL_GetTick() * 1000ULL;
                
                mavlink_msg_timesync_pack(
                    FLIGHT_CONTROLLER_SYS_ID,
                    FLIGHT_CONTROLLER_COMP_ID,
                    &resp_msg,
                    current_time_us,
                    timesync.ts1,
                    0, 0
                );
                
                uint16_t len = mavlink_msg_to_send_buffer(mavlink_buf, &resp_msg);
                mavlink_send(mavlink_buf, len);
            }
        }
        break;

        case MAVLINK_MSG_ID_COMMAND_LONG:   // 命令消息
        {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);
            
            // 只处理发给飞控系统的命令（target_system为0或飞控系统ID）
            if (cmd.target_system != 0 && cmd.target_system != FLIGHT_CONTROLLER_SYS_ID)
            {
                break;
            }
            
            switch(cmd.command)
            {
                case MAV_CMD_GET_HOME_POSITION: // 请求Home点位置
                {
                    send_home_position();   // 发送Home位置信息
                    send_command_ack(cmd.command, MAV_RESULT_ACCEPTED, 
                                    cmd.target_system, cmd.target_component);
                }
                break;

                case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:    // 请求飞控能力信息
                {
                    // 先发送版本信息，再发送ACK
                    send_autopilot_version(cmd.target_system, cmd.target_component);
                    send_command_ack(cmd.command, MAV_RESULT_ACCEPTED, 
                                    cmd.target_system, cmd.target_component);
                }
                break;
                
                default:    // 未知命令，应答即可
                {
                    send_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,
                                   cmd.target_system, cmd.target_component);
                }
                break;
            }
        }
        break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: // 参数列表请求
        {
            mavlink_param_request_list_t req;
            mavlink_msg_param_request_list_decode(msg, &req);
            
            if (req.target_system != 0 && req.target_system != FLIGHT_CONTROLLER_SYS_ID)
            {
                break;
            }
            
            // 逐个发送所有参数
            for (uint16_t i = 0; i < param_count; i++)
            {
                send_param_value(params[i].name, params[i].value, i);
            }
        }
        break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:   // 航点列表请求
        {
            mavlink_mission_request_list_t req;
            mavlink_msg_mission_request_list_decode(msg, &req);
            
            if (req.target_system != 0 && req.target_system != FLIGHT_CONTROLLER_SYS_ID)
            {
                break;
            }
            
            // 根据 mission_type 分别响应
            send_mission_count(req.target_system, req.target_component, 0, req.mission_type);
        }
        break;

        case MAVLINK_MSG_ID_MISSION_ACK:    // 航点ACK
        {
            mavlink_mission_ack_t ack;
            mavlink_msg_mission_ack_decode(msg, &ack);
        }
        break;

        case(MAVLINK_MSG_ID_ODOMETRY):  // 里程计消息
        {
            mavlink_odometry_t odom;
            mavlink_msg_odometry_decode(msg, &odom);

            // 将四元数转换为欧拉角（用于调试）
            float roll, pitch, yaw;
            
            // 四元数转欧拉角 (ZYX顺序)
            float q0 = odom.q[0];  // w
            float q1 = odom.q[1];  // x
            float q2 = -odom.q[2];  // y
            float q3 = -odom.q[3];  // z
            
            // Roll (x轴旋转)
            roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
            // Pitch (y轴旋转)
            pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
            // Yaw (z轴旋转)
            yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
            
            double yaw_deg = yaw * 57.2957795;

            
            yaw_deg = -yaw_deg;
            yaw_deg = fmod(yaw_deg + 360.0, 360.0);

            // 将里程计数据保存到全局结构体，供其他模块使用
            float temp_x = odom.y;
            float temp_y = odom.x;
            float temp_z = -odom.z;
            mavlink_struct.odom_position_x = odom_pos_x_filter.update(&odom_pos_x_filter, temp_x);
            mavlink_struct.odom_position_y = odom_pos_y_filter.update(&odom_pos_y_filter, temp_y);
            mavlink_struct.odom_position_z = odom_pos_z_filter.update(&odom_pos_z_filter, temp_z);

            float temp_vx = odom.vy;
            float temp_vy = odom.vx;
            float temp_vz = -odom.vz;
            mavlink_struct.odom_velocity_x = odom_vel_x_filter.update(&odom_vel_x_filter, temp_vx);
            mavlink_struct.odom_velocity_y = odom_vel_y_filter.update(&odom_vel_y_filter, temp_vy);
            mavlink_struct.odom_velocity_z = odom_vel_z_filter.update(&odom_vel_z_filter, temp_vz);

            mavlink_struct.odom_yaw_rad = yaw;
            mavlink_struct.odom_yaw_deg = yaw_deg;
        }
        break;

        case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:    // 设置执行器控制目标
        {
            mavlink_set_actuator_control_target_t ctrl;
            mavlink_msg_set_actuator_control_target_decode(msg, &ctrl);
        }
        break;

        case(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED):    // 设置位置目标
        {
            mavlink_set_position_target_local_ned_t pos_target;
            mavlink_msg_set_position_target_local_ned_decode(msg, &pos_target);
            
            mavlink_struct.set_position_x = pos_target.x; // 期望位置X
            mavlink_struct.set_position_y = pos_target.y; // 期望位置Y
            mavlink_struct.set_position_z = pos_target.z; // 期望位置Z
            
            mavlink_struct.set_yaw_rad    = (pos_target.yaw - (M_PI / 2.0));
        }
        break;

        default:    // 未处理的消息ID
        {
            
        }
        break;
    }
}