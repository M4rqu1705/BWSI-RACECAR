#!/usr/bin/env python

import numpy as np
from time import time
import cv2

class Pid_Controller:

    def __init__(self, KP, KI, KD):
        ''' Constructor: KP, KI, KD'''
        self.set_gains(KP, KI, KD)
        self.reset()

    def set_gains(self, KP, KI, KD):
        self.KP = KP    # Initialize KP to a value specified by constructor parameters
        self.KI = KI    # Initialize KI to a value specified by constructor parameters
        self.KD = KD    # Initialize KD to a value specified by constructor parameters

    def reset(self):
        # Reset error, last error and integral to 0 
        self.error = self.last_error = self.integral = 0

        # Store the current time as the last_count variable
        self.last_count = time()


    def calculate(self, setpoint, process_value):
        ''' Calculate desired output: setpoint, process_value '''

        # P - Calculate error
        self.error = setpoint - process_value

        # ID - Change in time later used to improve integral and derivative calculations
        change_in_time = (time() - self.last_count) 
        if change_in_time == 0:
            change_in_time = 0.001

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

    def set_charges(car_charges, wall_charges, push_charge):
        self.car_charge = car_charges
        self.wall_charges = wall_charges
        self.push_charge = push_charge

    def set_coefficients(speed_coefficient, steering_angle_coefficient):
        self.speed_coefficient = speed_coefficient
        self.steering_angle_coefficient = steering_angle_coefficient

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

        print "Speed = %s, Steering Angle = %s" % (speed, steering_angle)

        # Returns values after calculation 
        return speed, steering_angle

class Wall_Follower:
    
    def __init__(self, straight_pid_gains, turn_pid_gains):
        self.straight_pid = Pid_Controller(straight_pid_gains[0], straight_pid_gains[1], straight_pid_gains[2])
        self.turn_pid = Pid_Controller(turn_pid_gains[0], turn_pid_gains[1], turn_pid_gains[2])

    def calculate(self, direction, distance, laser_ranges):
        '''

        The reason why we use the law of cosines is to make sharper turns. 
        By using this tool we can make the wall follower more stable and 
        although we might manually tune the desired distance, it helps us
        detect turns even before the RACECAR gets to them

        c = a^2 + b^2 - 2*a*b*cos(theta)

        '''

        diagonal_front, diagonal_back = 0,0
        steering_coefficient = 0

        if str(direction).lower() in ['right', 'r', 'True', '1']:
            diagonal_front = np.average(laser_ranges[360-10:360+10])
            diagonal_back = np.average(laser_ranges[0:0+20])
            steering_coefficient = 1
        elif str(direction).lower() in ['left', 'l', 'False', '0']:
            diagonal_front = np.average(laser_ranges[720-10:720+10])
            diagonal_back = np.average(laser_ranges[1080-20:1080])
            steering_coefficient = -1
        elif str(direction).lower() in ['mesh_right', 'mesh right', 'mr']:
            sample = laser_ranges[180:360]
            min_indices = np.argsort(sample, kind='mergesort')[5:15]
            min_readings = np.take(sample, min_indices)
            diagonal_front = np.average(min_readings)
            sample = laser_ranges[0:180]
            min_indices = np.argsort(sample, kind='mergesort')[5:15]
            min_readings = np.take(sample, min_indices)
            diagonal_back = np.average(min_readings)
            steering_coefficient = 1
        elif str(direction).lower() in ['mesh_left', 'mesh left', 'ml']:
            sample = laser_ranges[720:900]
            min_indices = np.argsort(sample, kind='mergesort')[5:15]
            min_readings = np.take(sample, min_indices)
            diagonal_front = np.average(min_readings)
            sample = laser_ranges[900:1080]
            min_indices = np.argsort(sample, kind='mergesort')[5:15]
            min_readings = np.take(sample, min_indices)
            diagonal_back = np.average(min_readings)
            steering_coefficient = -1
            
        turn_visible = bool(diagonal_front > diagonal_back*3)

        if turn_visible:
            a, b = diagonal_front, diagonal_back
            C = np.radians(89.916743)
            # Law of cosines to calculate the length of leg c
            c = np.power(np.power(a,2) + np.power(b,2) - 2*a*b*np.cos(C), 0.5)

            A = np.arcsin((a*np.sin(C)/c))
            B = np.arcsin((b*np.sin(C)/c))

            # Change to left wall following
            speed = 3
            steering_angle = self.turn_pid.calculate(45, np.degrees(A)) * steering_coefficient
            #  print msg.drive.steering_angle
            #  print 'Turning'
            return speed, steering_angle

        else:
            speed = 3
            steering_angle = self.straight_pid.calculate(distance, diagonal_front) * steering_coefficient

            return speed, steering_angle

class Start_Light:

    def __init__(self, min_hls, max_hls):
        self.going = False
        self.has_seen_light = False
        self.min_hls = np.array(min_hls)
        self.max_hls = np.array(max_hls)

    def check_if_start(self, image):
        blurred_image = cv2.medianBlur(image, 9)
        hls_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hsvImg, self.min_hls , self.max_hls)
        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        max_contour_area_index = np.argmax(cv2.contourArea(contours))[0]
        
        if not self.has_seen_light and contours[max_contour_area_index] > 200:
            self.has_seen_light = True
        
        if self.has_seen_light and contour[max_contour_area_index] < 200:
            self.going = True

        return self.going





        



    



if __name__ == "__main__":
    wall_follow = Wall_Follower([1,0,0], [1/30.0, 0, 0])

    for i in range(3):
        print wall_follow.calculate('l', 1, np.array([i for x in range(1081)]))
