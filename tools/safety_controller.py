#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety_controller:

    laser_ranges = [x for x in range(0,897)]

    def __init__(self):              
        self.laser_sub = rospy.Subscriber("/scan_processed", LaserScan, self.laser_callback, queue_size=1)  
        self.drive_sub = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.drive_callback, queue_size = 1)                
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)    
        self.stop = False 
    	self.distance_threshold = 1

    def laser_callback(self, scan_data):  
        global laser_ranges
        laser_ranges = scan_data.ranges
	mid_data_range = scan_data.ranges[int(len(scan_data.ranges)/2)] 
        if mid_data_range < self.distance_threshold or mid_data_range == 200 :   
            self.stop = True  
        else: 
            self.stop = False      
	rospy.loginfo(mid_data_range)


    def drive_callback(self, drive_msg): 
        if self.stop:
            if laser_ranges[int(len(laser_ranges)/4)] < laser_ranges[int(len(laser_ranges)*3/4)]:
                drive_msg.drive.steering_angle = -1
            else:
                drive_msg.drive.steering_angle = 1

            drive_msg.drive.speed = -1
            
	self.drive_pub.publish(drive_msg)         


if __name__ == "__main__":
    rospy.init_node("safety_controller") 
    node = safety_controller()                  
    rospy.spin()  
