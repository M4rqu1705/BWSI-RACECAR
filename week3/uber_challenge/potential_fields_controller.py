#!/usr/bin/env python

import numpy as np

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

        print "Speed = %s, Steering Angle = %s" % (speed, steering_angle)

        # Returns values after calculation 
        return speed, steering_angle
