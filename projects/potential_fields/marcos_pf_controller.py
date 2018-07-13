#!/usr/bin/env python
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

laser_readings = [x for x in range(1081)]

def laser_callback(data):
    global laser_readings
    laser_readings = np.array(data.ranges)      # Convert received value list to numpy array
    laser_readings[laser_readings >= 200] = 0.4 # Replace every 'faulty' sensor reading for 0.4

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def method_1(car_charge, wall_charges, pushing_charge):
    global laser_readings

    # Calculate the forces acting on the robot using Coulomb's law
    forces = true_divide(car_charge * wall_charges, np.square(laser_readings))

    # List the angle of each reading corresponding forces
    angles = np.array([x for x in np.arange(0, 360, 360/laser_readings.size)])
    
    # Discard 1/4 of the readings because the racecar interferes with them
    forces = forces[1.0/8.0 * forces.size, 7.0/8.0 * forces.size]
    radians = radians[1.0/8.0 * radians.size, 7.0/8.0 * radians.size]

    # Convert angles from degrees to radians
    radians = np.radians(angles)

    # Look for the cos and sin of specific angles so not every force equally contributes to the robot's movement
    steering_angle = np.sum(np.multiply(np.cos(radians), forces))
    speed = np.sum(np.multiply(np.sin(radians), forces))

    # Without the pushing charge, the robot would move forward forever
    speed += pushing_charge

    return speed, steering_angle

def method_2():
    global laser_readings

    # Find the index of the largest reading
    max_index = np.where(laser_readings == laser_readings.max())
    if type(max_index) is list or type(max_index) is tuple:
        max_index = max_index[0]

    #(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, where in_min = 0, in_max = 897, out_min = 0, out_max = 360
    map(max_index, 0, 897, 0, 360

    # Find the error between the maximum distance reading degree with the front part of the robot
    steering_angle = max_degrees - 180

    # Calculate the speed to run based on the forward reading
    speed = 1.25 - (0.5/laser_readings[448])    

    return speed, steering_angle

def move_racecar(speed, steering_angle):
    # Change the speed and steering angle that will be published
    msg = AckermannDriveStamped()   # Create instance of the message object

# Change speed to be published
    if abs(speed) <= 1:
        msg.drive.speed = speed
    elif speed > 1:
        msg.drive.speed = 1
    elif speed < -1:
        msg.drive.speed = -1

 # Change steering_angle to be published
    if abs(steering_angle) <= 1:
        msg.drive.steering_angle = steering_angle
    elif steering_angle > 1:
        msg.drive.steering_angle = 1
    elif steering_angle < -1:
        msg.drive.steering_angle = -1

    rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(speed), str(steering_angle)))   # Log and display the speed and steering angle for debugging purposes

    drive_pub.publish(msg)


if __name__ == "__main__": 

    # Initiate potential_fields_racecar 'driver' node
    rospy.init_node('potential_fields_racecar', anonymous=True)

    # Create Ackermann Drive Publisher
    drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)

    # Create /scan_processed subscriber
    scanner_sub = rospy.Subscriber("/scan_processed", LaserScan, laser_callback, queue_size = 10)

    # Instantiate rate object at 10 hertz
    rate = rospy.Rate(10)


    # Eternally run loop
    while not rospy.is_shutdown():

        # Retrieve the potential fields controller output based on laser readings
        speed, steering_angle = method_1(5, 5, 10)

        # Move the racecar according to the desired steering angle and speed
        move_racecar(speed, steering_angle) 
        
        # Stop the program for 100ms
        rate.sleep()
