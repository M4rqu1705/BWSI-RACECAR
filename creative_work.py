#!/usr/bin/env python
# Written by Marcos

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

# Define a function that will input desired RACECAR speed, steering angle and 100 millisecond iterations that the RACECAR will run for and move accordingly
def move_racecar(_speed, _angle, _iterations):
    rospy.init_node('creative_project_node',  anonymous=True)
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
    rate = rospy.Rate(10)
    for i in range(0,_iterations):
	msg = AckermannDriveStamped()
	msg.drive.speed = _speed
	msg.drive.steering_angle = _angle
	pub.publish(msg)
	rate.sleep()


move_racecar(1,0,10)    # Drive forward
move_racecar(1,1,10)    # Drive towards the left
move_racecar(1,0,10)    # Drive forward again
move_racecar(1,-1,10)   # Drive towards the right
