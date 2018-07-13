#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety_controller:

    laser_ranges = np.array([x for x in range(0,897)])
    past_readings = np.array([0 for x in range(0,3)])

    def __init__(self):              
        self.laser_sub = rospy.Subscriber("/scan_processed", LaserScan, self.laser_callback, queue_size=1)  
        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size = 1)                
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)    
        self.stop = False 
    	self.distance_threshold = 1

    def laser_callback(self, scan_data):  
        global laser_ranges
        
        # Update the global laser_range's value with the new scans
        laser_ranges = np.array(scan_data.ranges)
        
        # Assign the reading directly forward to the variable mid_data_range
	mid_data_range = laser_ranges[int(laser_ranges.size/2)] 

        # ------- Median filter --------

        np.delete(past_readings, 0) # Remove oldest/ left-most value
        np.append(past_readings, mid_data_range)    # Add new value

        # Sort the past readings and look for the median. Assign the median to mid_data_range
        mid_data_range = (np.sort(past_readings, kind='mergesort'))[int(past_readings.size/2)]

        # ------- Median filter --------

        # Send the stop signal only if the measured obstacle is within the distance threshold or is equal to 200
        if mid_data_range < self.distance_threshold or mid_data_range == 200 :   
            self.stop = True  
        else: 
            self.stop = False      
	rospy.loginfo(mid_data_range)


    def drive_callback(self, drive_msg): 

        # If self.stop is true, make the robot follow an algorithm
        if self.stop:
            if laser_ranges[int(len(laser_ranges) * 3.0/8.0)] < laser_ranges[int(len(laser_ranges) * 5.0/8.0)]:
                drive_msg.drive.steering_angle = -1
            else:
                drive_msg.drive.steering_angle = 1

            drive_msg.drive.speed = -1
            
	self.drive_pub.publish(drive_msg)         


if __name__ == "__main__":
    rospy.init_node("safety_controller") 
    node = safety_controller()                  
    rospy.spin()  
