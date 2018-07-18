#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety_controller:

    laser_readings = np.array([x for x in range(0,897)])    # Prepare initial thrash values for laser readings
    readings_degrees = np.linspace(0, 360, num=897)         # Prepare mappings from laser readings to degrees
    counter = 0                                             # Preset the counter to 0
    important_indices = [0, 0, (0, 0), 0, 0]                # Preset the importnt_indices list to 0

    def __init__(self, amount_readings, amount_degrees):              
        global laser_readings
        global important_indices

        # Create subscriber from which to read LIDAR readings
        self.laser_sub = rospy.Subscriber("/scan_processed", LaserScan, self.laser_callback, queue_size=1)  

        # Create subscriber from which to read requests to move from other programs
        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size = 1)                

        # Create publisher with which to update RACECAR speed and steering angle
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)    

        
        self.state = "none"                     # String with which to indicate what to do in response of laser readings
        self.distance_thresholds = [1, 0.5]     # Indicate max and min threshold with which to respond
        self.amount_readings = amount_readings  # Update the amount of readings the LIDAR does per cycle based on constructor parameters
        self.amount_degrees = amount_degrees    # Update the range of degrees the LIDAR reads from per cycle based on constructor parameters

        laser_readings = np.array([x for x in range(0, self.amount_readings)])            # Prepare initial thrash values for laser readings

        # Make list containing the following indices: left, diagonal left, front range,  diagonal right, right
        important_indices = [int((laser_readings.size/2.0)-(90.0/self.amount_degrees*laser_readings.size)),
                int((laser_readings.size/2.0)-(45.0/self.amount_degrees*laser_readings.size)),
                [int(laser_readings.size/2)+15, int(laser_readings.size/2)-15 ],
                int((laser_readings.size/2.0)-(-45.0/self.amount_degrees*laser_readings.size)),
                int((laser_readings.size/2.0)-(-90.0/self.amount_degrees*amount_readings))]
        

    def laser_callback(self, scan_data):  
        global laser_readings
        global counter
        global important_indices
        
        # Update the global laser_range's value with the new scans
        laser_readings = np.array(scan_data.ranges)
        
        laser_readings[laser_readings==200] = 0.3       # Values of 200, or capped values, are interpreted as 0.3
        
        # Use x equally distanced readings from the front part to decide if to stop
        mid_data_range = np.array(laser_readings[important_indices[2][1]:important_indices[2][0]])

        # --------- Median ------
        
        temp = np.sort(mid_data_range, kind='mergesort')
        mid_data_point = temp[int(temp.size/2)]

        # --------- Median ------


        # If front distance is within the max limit and min limit
        if self.distance_thresholds[0] > mid_data_point and  mid_data_point > self.distance_thresholds[1]:
            self.state = "slow down"
        # If the side distances are within the max limit
        elif laser_readings[important_indices[0]] < self.distance_thresholds[0] or laser_readings[important_indices[4]] < self.distance_thresholds[0]:
            self.state = "slow down"
        # if the front distance is within the min limit
        elif mid_data_point <= self.distance_thresholds[1]:  
            self.state = "reverse"
        # If everything is alright
        else: 
            self.state = "none"

	rospy.loginfo(mid_data_range)

    def drive_callback(self, drive_msg): 

        self.counter+=1

        if "reverse" in self.state:
            if laser_readings[important_indices[1]] <= laser_readings[important_indices[3]] and laser_readings[important_indices[0]] <= laser_readings[important_indices[4]]:
                drive_msg.drive.steering_angle += 0.05*self.counter
            else:
                drive_msg.drive.steering_angle -= 0.05*self.counter

            drive_msg.drive.speed -=0.5*self.counter

        elif "slow down" in self.state:
            if laser_readings[important_indices[1]] <= laser_readings[important_indices[3]]:
                drive_msg.drive.steering_angle -= 0.1*self.counter
                drive_msg.drive.speed = laser_readings[important_indices[1]]/1.25
            else:
                drive_msg.drive.steering_angle += 0.1*self.counter
                drive_msg.drive.speed = laser_readings[important_indices[3]]/1.25

        else:
           self.counter = 0 
	self.drive_pub.publish(drive_msg)         

if __name__ == "__main__":
    rospy.init_node("safety_controller") 
    node = safety_controller(897, 360)                  
    rospy.spin()  
