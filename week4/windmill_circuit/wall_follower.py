#!/usr/bin/env python

from time import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Pid_Controller:
    def __init__(self, KP, KI, KD):
        ''' Constructor: KP, KI, KD '''
        self.KP = KP    # Initialize KP to a value specified by constructor parameters
        self.KI = KI    # Initialize KI to a value specified by constructor parameters
        self.KD = KD    # Initialize KD to a value specified by constructor parameters

        # Initialize error, last error and integral to 0 by default
        self.error = self.last_error = self.integral = 0

        # Store the current time as the last_count variable
        self.last_count = time()

    def calculate(self, setpoint, process_value):
        ''' Calculate desired output: setpoint, process_value '''

        # P - Calculate error
        self.error = setpoint - process_value

        # ID - Change in time later used to improve integral and derivative calculations
        change_in_time = (time() - self.last_count)     # Time seconds

        # I - Accumulate error
        self.integral += self.error * change_in_time
        # I - Reset Integral if got to destination. Prevents overshoot
        if self.error == 0:
            self.integral = 0

        # D - Calculate 'break force'
        derivative = (self.error - self.last_error) / change_in_time

        # PID - Multiply gains with respective products
        output = self.KP * self.error + self.KI * self.integral + self.KD * derivative

        # D - Update last error for next cycle
        self.last_error = self.error
        # ID - Update last_count for next cycle
        self.last_count = time()

        return output
    

laser_ranges = np.array([x for x in range(0, 1081)])


def scanCallback(scan):
    global laser_ranges

    laser_ranges = np.array(scan.ranges)
    print laser_ranges[900]
if __name__ == "__main__":

    # Initialize ROS node called wall_follower
    rospy.init_node('wall_follower')

# ========= ========= ========= ========= ========= ========= Object instantiation ==========  ==========  ==========  ==========  ==========  ========== #

pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)

sub = rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size = 10)

steering_angle_turn_pid = Pid_Controller(1/30.0, 0, 0)
steering_angle_straight_pid = Pid_Controller(0.5, 0, 0.1)

rate = rospy.Rate(30)

# ========= ========= ========= ========= ========= ========= Object instantiation ==========  ==========  ==========  ==========  ==========  ========== #

while not rospy.is_shutdown():
    msg = AckermannDriveStamped()

    '''

    The reason why we use the law of cosines is to make sharper turns. 
    By using this tool we can make the wall follower more stable and 
    although we might manually tune the desired distance, it helps us
    detect turns even before the RACECAR gets to them

    c = a^2 + b^2 - 2*a*b*cos(theta)

    '''

    diagonal_front = np.average(laser_ranges[720-10:720+10])
    diagonal_back = np.average(laser_ranges[1080-20:1080])
    

    turn_visible = bool(diagonal_front > diagonal_back*3)

    if turn_visible:
        a, b = diagonal_front, diagonal_back
        C = np.radians(89.916743)
        # Law of cosines to calculate the length of leg c
        c = np.power(np.power(a,2) + np.power(b,2) - 2*a*b*np.cos(C), 0.5)

        A = np.arcsin((a*np.sin(C)/c))
        B = np.arcsin((b*np.sin(C)/c))

        # Change to left wall following
        msg.drive.speed = 3
        msg.drive.steering_angle = -steering_angle_turn_pid.calculate(45, np.degrees(A))
        #  print msg.drive.steering_angle
        #  print 'Turning'

    else:
        c = diagonal_front

        msg.drive.speed = 3
        msg.drive.steering_angle = -steering_angle_straight_pid.calculate(1, c)
        #  print msg.drive.steering_angle
        #  print 'Forward'

    pub.publish(msg)

    rate.sleep()
