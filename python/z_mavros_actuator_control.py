#!/usr/bin/env python
import rospy
import math
from mavros_msgs.msg import PositionTarget

rospy.init_node('position_control')
pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
rate = rospy.Rate(200)  # 50Hz

msg = PositionTarget()
msg.header.frame_id = "map"
msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

# 修改掩码：启用所有控制（位置、速度、加速度、偏航）
# 设置 type_mask = 0 表示所有字段都有效
msg.type_mask = 0  # 或者 PositionTarget.IGNORE_NOTHING

start_time = rospy.Time.now().to_sec()

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    t = rospy.Time.now().to_sec() - start_time
    
    # ========== 位置控制 ==========
    msg.position.x = 2.0 * math.sin(t * 0.3)      # X轴 正弦运动 ±2米
    msg.position.y = 2.0 * math.cos(t * 0.3)      # Y轴 余弦运动 ±2米
    msg.position.z = 2.0 + 0.5 * math.sin(t * 0.5) # 高度 1.5-2.5米
    
    # ========== 速度控制（位置的时间导数）==========
    # vx = dx/dt = 2.0 * 0.3 * cos(t * 0.3) = 0.6 * cos(t * 0.3)
    msg.velocity.x = 0.6 * math.cos(t * 0.3)
    
    # vy = dy/dt = -2.0 * 0.3 * sin(t * 0.3) = -0.6 * sin(t * 0.3)
    msg.velocity.y = -0.6 * math.sin(t * 0.3)
    
    # vz = dz/dt = 0.5 * 0.5 * cos(t * 0.5) = 0.25 * cos(t * 0.5)
    msg.velocity.z = 0.25 * math.cos(t * 0.5)
    
    # ========== 加速度控制（速度的时间导数）==========
    # ax = dvx/dt = -0.6 * 0.3 * sin(t * 0.3) = -0.18 * sin(t * 0.3)
    msg.acceleration_or_force.x = -0.18 * math.sin(t * 0.3)
    
    # ay = dvy/dt = -0.6 * 0.3 * cos(t * 0.3) = -0.18 * cos(t * 0.3)
    msg.acceleration_or_force.y = -0.18 * math.cos(t * 0.3)
    
    # az = dvz/dt = -0.25 * 0.5 * sin(t * 0.5) = -0.125 * sin(t * 0.5)
    msg.acceleration_or_force.z = -0.125 * math.sin(t * 0.5)
    
    # ========== 偏航和偏航速度 ==========
    msg.yaw = 0.5 * math.sin(t * 0.2)                    # 偏航角 ±0.5 rad
    msg.yaw_rate = 0.5 * 0.2 * math.cos(t * 0.2) # 偏航角速度
    
    pub.publish(msg)
    
    rate.sleep()