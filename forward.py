#!/usr/bin/env python
# Written by Marcos

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node('forward_node', anonymous=True) # Initiated a node called 'forward_node'
pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)   # Created a Publisher object in order to help us publish in later steps
rate = rospy.Rate(10)   # Create a rate object with which we can later adjust the program execution to go at 10 hertz

while not rospy.is_shutdown():  # Run this code while it isn't shut down by the Control-C interrupt
    msg = AckermannDriveStamped()   # Create the AckermannDriveStamped object, which will later be sent to the RACECAR
    msg.drive.speed = 1     # Define that the speed sent to the robot will be 1
    msg.drive.steering_angle = 0    # Define that the steering_angle will be 0, or it will face forward
    pub.publish(msg)    # Publish the AckermannDriveStamped object to the '/vesc/high_level/ackermann_cmd_mux/input/nav_0' topic
    rate.sleep()    # Wait enough time for the code to run 10 times every second
