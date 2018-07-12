#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class safety_controller:
    self.distance_threshold = 1
    def __init__(self):              
        self.laser_sub = rospy.Subscriber("/scan_processed", LaserScan, self.laser_callback, queue_size=1)  
        self.drive_sub = rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input", AckermannDriveStamped, self.drive_callback, queue_size = 1)                    
        self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux", AckermannDriveStamped, queue_size=10)    
        self.stop = False 

    def drive_callback(self, drive_msg): 
        if self.stop and drive_msg.speed > 0:                                               
            drive_msg.drive.speed = 0      
            self.drive_pub.publish(drive_msg)         
    
    def laser_callback(self, scan_data):  
        if scan_data.ranges[len(scan_data.ranges)/2] < self.distance_threshold:   
            self.stop = True  
        else: 
            self.stop = False      

if __name__ == "__main__":
    rospy.init_node("safety_controller_node", anonymous = True) 
    node = safety_controller()                  
    rospy.spin()  
