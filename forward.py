#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def publish_message(origin, destination):
    rospy.init_node(origin,  anonymous=True)
    pub = rospy.Publisher(destination, AckermannDriveStamped, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	msg = AckermannDriveStamped()
	msg.drive.speed = 1
	msg.drive.steering_angle = 0
	pub.publish(msg)
	rate.sleep()

publish_message('forward_node', '/vesc/high_level/ackermann_cmd_mux/input/nav_0')
