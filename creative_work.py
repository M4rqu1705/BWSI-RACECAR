#!/usr/bin/env python

import rospy
from time import sleep
from ackermann_msgs.msg import AckermannDriveStamped

def move_racecar(_speed, _angle, times):
    rospy.init_node('creative_project_node',  anonymous=True)
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
    rate = rospy.Rate(10)
    for i in range(0,times):
	msg = AckermannDriveStamped()
	msg.drive.speed = _speed
	msg.drive.steering_angle = _angle
	pub.publish(msg)
	rate.sleep()

for counter in range(0,10):
    move_racecar(1,0,10)
    sleep(1)
    move_racecar(1,1,10)
    sleep(1)
    move_racecar(1,0,10)
    sleep(1)
    move_racecar(1,-1,10)
    sleep(1)
