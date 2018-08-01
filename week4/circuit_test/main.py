#!/usr/bin/env python

from time import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import ar_pose_retriever as ar

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
    


class Potential_Fields_Controller:
    ''' Class to command the robot to avoid wall obstacles and follow curves '''

    def __init__(self, car_charges, wall_charges, push_charge, speed_coefficient, steering_angle_coefficient):
        ''' 
            Constructor that takes car charge, wall charge and RACECAR pushing
            charge as parameters to later use them for computations. These 
            three variables will determine the RACECAR's behavior while
            navigating between two walls

            Coulomb's law:
                
                f = (q1*q1)/r^2

            Additionally, the speed coefficient and steering angle prevents
            too much sensitivity or too little sensitivity
        '''
        self.car_charge = car_charges       # Q1 variable in Coulomb's law
        self.wall_charges = wall_charges    # Q2 variable in Coulomb's law
        self.push_charge = push_charge      # Helps the RACECAR keep moving forward
        self.speed_coefficient = speed_coefficient
        self.steering_angle_coefficient = steering_angle_coefficient

        # r will be the individual laser readings

    def calculate(self, laser_readings):

        # Calcuate the forces acting on the robot using Coulomb's law
        forces = np.true_divide((self.car_charge * self.wall_charges), np.power(laser_readings, 2))

        # List the angle of each reading corresponding forces. Take into account the 45 degree offset
        angles = np.array([x for x in np.linspace(45,315, laser_readings.size, endpoint=True, dtype=np.float_)])

        # Convert angles from degrees to radians
        radians = np.radians(angles)

        # Look for the cosine and sine of specific angles so not every force equally contributes to the robot's movements
        steering_angle = np.sum(np.multiply(np.clip(np.sin(radians), -0.625, 0.625), forces))
        speed = np.sum(np.multiply(np.clip(np.cos(radians), -0.625, 0.625), forces))

        # Without the pushing charge, the robot would move backwards forever
        speed += self.push_charge

        # Return modified steering
        speed = self.speed_coefficient*speed
        steering_angle = self.steering_angle_coefficient*steering_angle

        #  print "Speed = %s, Steering Angle = %s" % (speed, steering_angle)

        # Returns values after calculation 
        return speed, steering_angle

front_distance, left_distance, right_distance = 0, 0, 0
laser_ranges = np.array([x for x in range(0, 1081)])
last_ar_marker = 22
ar_tag_list = []


def scanCallback(scan):
    global laser_ranges
    global front_distance
    global left_distance
    global right_distance

    scan  = np.array(scan.ranges)
    laser_ranges = scan
    front_distance = scan[540]
    left_distance = scan[900]
    right_distance = scan[180]


if __name__ == "__main__":

    rospy.init_node('wall_follower')

    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)

    sub = rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size = 10)

    ar_sub = ar.ar_pose_retriever()

    steering_angle_pid = Pid_Controller(1, 0, 0.3)
    
    potential_fields = Potential_Fields_Controller(0.05, 0.05, 1.75, 0.5, 0.5)

    rate = rospy.Rate(30)


    while not rospy.is_shutdown():
        ar_tag_list = ar_sub.get_tag_list()

        print last_ar_marker

        if len(ar_tag_list) > 0 and ar_tag_list[0]["z"] < 0.4:
            last_ar_marker = ar_tag_list[0]["id"]
            #  print ar_tag_list[0]["z"]

        msg = AckermannDriveStamped()

        if last_ar_marker == 18 or last_ar_marker == 22:
            # Change to left line following
            msg.drive.speed = 1
            msg.drive.steering_angle = -steering_angle_pid.calculate(0.75, left_distance)
        elif last_ar_marker == 19:
            # Change to right line following
            msg.drive.speed = 1
            msg.drive.steering_angle = -steering_angle_pid.calculate(0.5, right_distance)
        elif last_ar_marker == 23:
            # Change to potential fields controlling
            msg.drive.speed, msg.drive.steering_angle = potential_fields.calculate(laser_ranges)


        pub.publish(msg)

        rate.sleep()
