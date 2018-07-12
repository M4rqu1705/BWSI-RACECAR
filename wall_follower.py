#!/usr/bin/env python

import rospy 
from ackermann_msgs.msg import AckermannDriveStamped
from senso

class pd_controller:
    error = 0
    last_error = 0

    def __init__(self, kP, kD):
        self.kP = kP
        self.kD = kD
        self.error = 0
        self.last_error = 0

    def calculate(self, setpoint, process_variable):
        self.error = setpoint - process_variable
        derivative = self.error - self.last_error
        return self.kP*self.error + self.kD*derivative


def move_racecar(speed, steering_angle):
    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    drive_pub.publish(msg)


rospy.init_node('wall_follower_node', anonymous=True)
drive_pub = rospy.Publisher("/ackermann_cmd_mux", AckermannDriveStamped, queue_size=10)

my_pd = pd_controller(1,0)
