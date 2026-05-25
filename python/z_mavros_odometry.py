#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
import tf.transformations as tf

rospy.init_node('odometry_publisher')
pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
rate = rospy.Rate(50)  # 50Hz

msg = Odometry()
msg.header.frame_id = "odom"
msg.child_frame_id = "base_link"

start_time = rospy.Time.now().to_sec()

# 协方差矩阵需要 36 个元素（完整的 6x6 矩阵）
# 顺序: x, y, z, roll, pitch, yaw
pose_covariance = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # x
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # y
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,   # z
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,   # roll
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,   # pitch
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01    # yaw
]

# 速度协方差也需要 36 个元素
# 顺序: vx, vy, vz, roll_rate, pitch_rate, yaw_rate
twist_covariance = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
]

# 用于速度计算的变量
prev_x, prev_y, prev_z = 0.0, 0.0, 0.0
prev_yaw = 0.0
prev_t = 0.0

while not rospy.is_shutdown():
    current_t = rospy.Time.now().to_sec()
    t = current_t - start_time
    dt = current_t - prev_t if prev_t > 0 else 0.02
    
    # 计算当前位置和姿态
    x = 2.0 * math.sin(t * 0.3)
    y = 2.0 * math.cos(t * 0.3)
    z = 2.0 + 0.5 * math.sin(t * 0.5)
    yaw = 0.5 * math.sin(t * 0.2)
    
    # 计算速度（一阶差分）
    if dt > 0.001:
        vx = (x - prev_x) / dt
        vy = (y - prev_y) / dt
        vz = (z - prev_z) / dt
        yaw_rate = (yaw - prev_yaw) / dt
    else:
        vx = vy = vz = yaw_rate = 0.0
    
    # 将偏航角转换为四元数
    quaternion = tf.quaternion_from_euler(0, 0, yaw)
    
    # 填充位姿
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]
    msg.pose.covariance = pose_covariance
    
    # 填充速度
    msg.twist.twist.linear.x = vx
    msg.twist.twist.linear.y = vy
    msg.twist.twist.linear.z = vz
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = yaw_rate
    msg.twist.covariance = twist_covariance
    
    # 更新时间戳
    msg.header.stamp = rospy.Time.now()
    
    pub.publish(msg)
    
    # 保存当前值用于下一帧的速度计算
    prev_x, prev_y, prev_z = x, y, z
    prev_yaw = yaw
    prev_t = current_t
    
    rate.sleep()