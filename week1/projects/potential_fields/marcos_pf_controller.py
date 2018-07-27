#!/usr/bin/env python
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

laser_readings = np.array([x for x in range(1081)])

def laser_callback(data):
    global laser_readings
    laser_readings = np.array(data.ranges)      # Convert received value list to numpy array
    #  print laser_readings[540:542]
    #  laser_readings[laser_readings >= 200] = 0.4 # Replace every 'faulty' sensor reading for 0.4

def method_1(car_charge, wall_charges, pushing_charge):
    global laser_readings

    # Calculate the forces acting on the robot using Coulomb's law
    forces = np.true_divide(car_charge * wall_charges, np.square(laser_readings))

    # List the angle of each reading corresponding forces
    angles = np.array([x for x in np.linspace(0, 270, laser_readings.size)])
    np.add(angles, 45)
    
    # Discard 1/4 of the readings because the racecar interferes with them
    #  forces = forces[int(1.0/8.0 * forces.size): int(7.0/8.0 * forces.size)]
    #  angles = angles[int(1.0/8.0 * angles.size): int(7.0/8.0 * angles.size)]

    # Convert angles from degrees to radians
    radians = np.radians(angles)

    # Look for the cos and sin of specific angles so not every force equally contributes to the robot's movement
    steering_angle = np.sum(np.multiply(np.sin(radians), forces))
    speed = np.sum(np.multiply(np.cos(radians), forces))

    # Without the pushing charge, the robot would move forward forever
    speed += pushing_charge

    
    return speed*0.1, steering_angle*0.1


def move_racecar(speed, steering_angle):
    # Change the speed and steering angle that will be published
    msg = AckermannDriveStamped()   # Create instance of the message object
    rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(speed), str(steering_angle)))   # Log and display the speed and steering angle for debugging purposes

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


    drive_pub.publish(msg)


if __name__ == "__main__": 

    # Initiate potential_fields_racecar 'driver' node
    rospy.init_node('potential_fields_racecar', anonymous=True)

    # Create Ackermann Drive Publisher
    drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)

    # Create /scan_processed subscriber
    scanner_sub = rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 10)

    # Instantiate rate object at 10 hertz
    rate = rospy.Rate(10)


    # Eternally run loop
    while not rospy.is_shutdown():

        # Retrieve the potential fields controller output based on laser readings
        speed, steering_angle = method_1(0.1, 0.1, 10)

        # Move the racecar according to the desired steering angle and speed
        move_racecar(speed, steering_angle) 
        
        # Stop the program for 100ms
        rate.sleep()
