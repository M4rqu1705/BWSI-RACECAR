#!/usr/bin/env python

import numpy as np
import cv2
import rospy

# Message with which to update Ackermann Drive values such as speed and steering angle
from ackermann_msgs.msg import AckermannDriveStamped

# Module with which to interpret images from the ZED encoded in a ROS format of sorts
from cv_bridge import CvBridge, CvBridgeError

# Message with which to receive Images from the ZED
from sensor_msgs.msg import Image

from sensor_msgs.msg import LaserScan

#  from ar_track_alvar_msgs.msg import AlvarMarkers

import PID
import potential_fields_controller as pf
import face_detector as fd
import AR_marker_detector as armd
import odometry_navigation as on


laser_readings = np.array([x for x in range(1081)])


def laser_callback(data):
    # Make sure to manipulate the global laser_readings
    global laser_readings
    # Convert the retrieved data list to a numpy array
    laser_readings = np.array(data.ranges)


def image_callback(img):
    # Make sure to manipulate the globals face_x_position and maker_x_positions
    global face_x_position
    global marker_x_positions

    # Convert the image to a usable format
    retrieved_image = bridge.imgmsg_to_cv2(img, 'bgr8')

    # Resize image to a fourth of its size so less processing power is consumed later
    retrieved_image = cv2.resize(retrieved_image, (0,0), fx=0.25, fy=0.25)

    # Blur the image to reduce noise
    retrieved_image = cv2.medianBlur(retrieved_image, 5)

    # Image proccessing for face detection and recognition
    face_found, face_id, face_x_position = face_detector.process(retrieved_image)
    if current_ta == face_id:
        phase = [False, False, True, False, False, False, False, False]
                
    #  #Image processing for AR marker detector and recognition
    #  maker_ids, marker_x_positions = AR_marker_detector.process(retrieved_image)
    #  #What marker's x position??    ********************************************************************************************
    #  if current_ta in marker_ids:
        #  phase = [False, False, False, False, False, True, False, False]


def move_racecar(_speed, _steering_angle):
    # Change speed and steering angle that will be published
    msg = AckermannDriveStamped()

    # Cap values so they don't surpass the RACECAR's limits
    if abs(_speed) <= 2:
        msg.drive.speed = _speed
    elif _speed > 2:
        msg.drive.speed = 2
    elif _speed < -2:
        msg.drive.speed = -2

    if abs(_steering_angle) <= 1:
        msg.drive.steering_angle = _steering_angle
    elif _steering_angle > 1:
        msg.drive.steering_angle = 1
    elif _steering_angle < -1:
        msg.drive.steering_angle = -1

    # Publish desired speed and steering angle
    drive_pub.publish(msg)


steering_angle = 0     
speed = 0             
current_ta = 0
previously_changed_ta = False
phase = [True, False, False, False, False, False, False, False]

# Instantiate potential fields controller
potential_fields_controller = pf.Potential_Fields_Controller(0.05, 0.05, 1.75, 0.5, 0.5)

# Instantiate face detector 
face_detector = fd.Face_Detector()

AR_marker_detector = armd.AR_Marker_Detector()

odometry_navigator = on.odometry_navigation()

# Instantiate steering angle PID controller
steering_angle_PID = PID.Pid_Controller(1, 0, 0)

# Instantiate speed PID controller
speed_PID = PID.Pid_Controller(1, 0, 0)

bridge = CvBridge()


# Initiate uber challenge ROS node
rospy.init_node('uber_challenge')

# Create Ackermann Drive Publisher to move the racecar
drive_pub = rospy.Publisher(
        "/vesc/high_level/ackermann_cmd_mux/input/nav_0", # Topic to publish to
        AckermannDriveStamped,  # Data type. Do not change
        queue_size = 10)        # Amount of requests that might wait for send

# Create LIDAR subscriber to receive distance readings all around
scanner_sub = rospy.Subscriber(
        "/scan",                # Topic from which data is retrieved
        LaserScan,              # Data type. Do not change.
        laser_callback,    # Callback function that will handle data
        queue_size = 10)        # Limit amount of received data that wait in queue

# Subscribe to ZED camera topic to frequently receive video 
sub = rospy.Subscriber('/zed/left/image_rect_color',
        Image,                  # Data type. Do not change
        image_callback,    # Callback function. No need to change
        queue_size = 10)        # Not so necessary, and can be any number

# Instantiate rate object at 30 hertz
rate = rospy.Rate(30)


while not rospy.is_shutdown():
    print 'Running'

    if phase[0]:
        phase = [True, False, False, False, False, False, False, False]
        previously_changed_ta = False
        # Move towards position 1

    elif phase[1]:
        phase = [False, True, False, False, False, False, False, False]
        # Scan the area
        steering_angle = 1
        speed = -1
        speed = -1
        move_racecar(speed, steering_angle)

    elif phase[2]:
        phase = [False, False, True, False, False, False, False, False]
        # Home for the TA
        steering_angle = steering_angle_PID(retrieved_image.shape[1]/2, face_x_position)
        speed =  speed_PID(0.25, laser_readings[np.argmin(laser_readings)])
        move_racecar(speed, steering_angle)

    elif phase[3]:
        phase = [False, False, False, True, False, False, False, False]
        # Move towards reverse position 1

    elif phase[4]:
        phase = [False, False, False, False, True, False, False, False]
        # Move towards position 2

    elif phase[5]:
        phase = [False, False, False, False, False, True, False, False]
        # Scan the area
        steering_angle = -1
        speed = -1
        move_racecar(speed, steering_angle)

    elif phase[6]:
        phase = [False, False, False, False, False, False, True, False]
        # Home for the AR tag
        steering_angle = steering_angle_PID(retrieved_image.shape[1]/2, marker_x_position)
        speed =  speed_PID(0.25, laser_readings[np.argmin(laser_readings)])
        move_racecar(speed, steering_angle)
        
    elif phase[7]:
        phase = [False, False, False, False, False, False, False, True]
        if not previously_changed_ta:
            current_ta += 1
        else:
            previously_changed_ta = True

        # Move towards reversed position 2



    # Stop the program for 33.33ms
    rate.sleep()
