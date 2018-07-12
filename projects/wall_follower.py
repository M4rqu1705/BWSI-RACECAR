#!/usr/bin/env python

import rospy 
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from pid.py import pid_controller

def move_racecar(speed, steering_angle):
    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    drive_pub.publish(msg)


rospy.init_node('wall_follower_node', anonymous=True)
drive_pub = rospy.Publisher("/ackermann_cmd_mux", AckermannDriveStamped, queue_size=10)

my_pid = pid_controller(1,0,0.3)
move_racecar(1, my_pid.calculate(10, LaserScan.ranges[int(len(LaserScan.ranges)/2)])
