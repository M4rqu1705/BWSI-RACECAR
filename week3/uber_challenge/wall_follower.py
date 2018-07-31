#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from time import time

class Pid_Controller:

    def __init__(self, KP, KI, KD):
        ''' Constructor: KP, KI, KD'''
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
        return self.kP*self.error + self.kD*derivative

desired_distance = 0.5
desired_front_distance = 1.5
turn_speed = 0.75
front_distance = 10
left_distance = 10

def scanCallback(scan):
    global front_distance
    global left_distance

    scan  = np.array(scan.ranges)
    front_distance = scan[540]
    left_distance = scan[900]

if __name__ == "__main__":

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)
    sub = rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size = 10)
    rate = rospy.Rate(30)
    steering_angle_pid = Pid_Controller(1, 0, 0.3)

    while not rospy.is_shutdown():

        msg = AckermannDriveStamped()
        if left_distance > 1:
            print 'Turning left'
            msg.drive.speed = turn_speed - 0.25
            msg.drive.steering_angle = 1
        elif front_distance < desired_front_distance:
            print 'Turning right'
            msg.drive.speed = turn_speed
            msg.drive.steering_angle = -1
        elif front_distance > desired_front_distance:
            print 'Wall following'
            msg.drive.speed = 3
            msg.drive.steering_angle = -steering_angle_pid.calculate(desired_distance, left_distance)

        pub.publish(msg)

        rate.sleep()
