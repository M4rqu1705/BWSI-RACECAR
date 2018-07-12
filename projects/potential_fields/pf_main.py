#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from PFController import pfController

laser_readings = [x for x in range(1081)]

def move_racecar(speed, steering_angle):
    ''' Function for moving racecar

    Change the speed and steering angle that will be published '''

    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(speed), str(steering_angle)))
    drive_pub.publish(msg)

# Initiate potential_fields_racecar 'driver' node
rospy.init_node('potential_fields_racecar', anonymous=True)

# Create Ackermann Drive Publisher
drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)


def laser_callback(data):
    global laser_readings
    laser_readings = data.ranges

scanner_sub = rospy.Subscriber("/scan_processed", LaserScan, laser_callback, queue_size = 10)


# Instantiate the pfController class
pf_controller = pfController(0.01, 0.01, 1)

# Instantiate rate object at 10 hertz
rate = rospy.Rate(10)

# Eternally run loop
while not rospy.is_shutdown():

    # Retrieve the potential fields controller output based on laser readings
    output = pf_controller.compute(laser_readings)

    # Move the racecar according to the desired steering angle and speed
    move_racecar(output[1], output[0])
    
    # Stop the program for 100ms
    rate.sleep()

rospy.spin
