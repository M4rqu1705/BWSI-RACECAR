#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety_controller:

    laser_ranges = np.array([x for x in range(0,897)])
    counter = 0

    def __init__(self):              
        self.laser_sub = rospy.Subscriber("/scan_processed", LaserScan, self.laser_callback, queue_size=1)  
        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size = 1)                
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)    
        self.stop = False 
        self.slow_down = False
        self.distance_thresh_max = 1
    	self.distance_threshold_min = 0.75

    def laser_callback(self, scan_data):  
        global laser_ranges
        global counter
        
        # Update the global laser_range's value with the new scans
        laser_ranges = np.array(scan_data.ranges)
        laser_ranges[laser_ranges==200] = 0.4
        
        # Use 20 equally distanced readings from the front part to decide if to stop
	mid_data_range = np.array(laser_ranges[int(laser_ranges.size/2)-15 : int(laser_ranges.size/2)+15])

        # --------- Median ------
        
        temp = np.sort(mid_data_range, kind='mergesort')
        mid_data_point = temp[int(temp.size/2)]

        # --------- Median ------

        # Send the stop signal only if the measured obstacle is within the distance threshold or is equal to 200
        if mid_data_point < self.distance_thresh_max and mid_data_point > self.distance_threshold_min:
            self.slow_down = True
            self.stop = False
        elif mid_data_point < self.distance_threshold_min:  
            self.stop = True  
            self.slow_down = False
        else: 
            self.stop = False      
            self.slow_down = False
	rospy.loginfo(mid_data_range)

    def drive_callback(self, drive_msg): 

        self.counter+=1

        # If self.stop is true, make the robot follow an algorithm
        if self.stop:
            if laser_ranges[int(len(laser_ranges) * 3.0/8.0)] < laser_ranges[int(len(laser_ranges) * 5.0/8.0)]:
                drive_msg.drive.steering_angle -= 0.05*self.counter
                #  drive_msg.drive.steering_angle = -1
            else:
                drive_msg.drive.steering_angle += 0.05*self.counter
                #  drive_msg.drive.steering_angle = 1

            drive_msg.drive.speed -=0.5*self.counter
            #  drive_msg.drive.speed = -1

        elif self.slow_down:
            if laser_ranges[int(len(laser_ranges) * 3.0/8.0)] < laser_ranges[int(len(laser_ranges) * 5.0/8.0)]:
                drive_msg.drive.steering_angle += 0.1*self.counter
                drive_msg.drive.speed = 0.25*laser_ranges[int(len(laser_ranges) / 2.0)]
            else:
                drive_msg.drive.steering_angle -= 0.1*self.counter
                drive_msg.drive.speed = 0.25*laser_ranges[int(len(laser_ranges) / 2.0)]

            #  drive_msg.drive.speed = -1

        else:
           self.counter = 0 
	self.drive_pub.publish(drive_msg)         

if __name__ == "__main__":
    rospy.init_node("safety_controller") 
    node = safety_controller()                  
    rospy.spin()  
