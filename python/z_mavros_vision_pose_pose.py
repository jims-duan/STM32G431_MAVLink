#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('vision_publisher')
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
rate = rospy.Rate(200)  # 50Hz

msg = PoseStamped()
msg.header.frame_id = 'vision'
msg.pose.position.x = 2.0
msg.pose.position.y = 1.0
msg.pose.position.z = 1.5
msg.pose.orientation.w = 1.0

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()  # 每次都更新时间戳
    pub.publish(msg)
    rate.sleep()