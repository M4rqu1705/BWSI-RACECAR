#!/usr/bin/env python
# Leo Belyi BWSI 2018

from __future__ import division
import random
import math
import csv
import colorsys
import rospy
import cv2
import numpy as np
import tf
from std_msgs.msg import Bool
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

AR_TOPIC = "/ar_pose_marker"
ENABLE_AR_TOPIC = "/ar_track_alvar/enable_detection"
IMAGE_TOPIC = "/zed/left/image_rect_color"

# How big the AR tag is (in meters)
BOX_DIMENSION = 0.105


class ArTagDetectionDemo:
    # Class that detects the AR tags, the position and orientation.
    # Demonstrates it using OpenCV to track it on the screen.

    def __init__(self):
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=100)
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback, queue_size=10)
        self.image_pub = rospy.Publisher("AR_Finder", Image, queue_size=1)
	
	enable_pub = rospy.Publisher(ENABLE_AR_TOPIC, Bool, queue_size=1)
	enable_pub.publish(True)

        self.bridge = CvBridge()

        self.markers = []

        self.pix_w = 1
        self.pix_h = 1 

        self.max_theta = 0.1
        self.max_rho = 0.1

        self.ar_colors = {}
        self.ar_corner_colors = {}

        self.max_theta, self.max_rho = 1.08, 0.8

    def ar_callback(self, markers):
        # Store the markers
        if len(markers.markers) > 0:
            self.markers = markers.markers
        else:
            self.markers = []

    def rotated_vector(self, center, v, q):
        # Use Euler rotational matricies to calculate the rotation

        # Make sure you are rotating from the center and not the camera
        v = np.array((v[0] - center[0], v[1] - center[1], v[2] - center[2]))

        # tf built in quaternion to euler
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

        # The individual rotational matricies
        r_x = np.array([[1, 0, 0],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll), math.cos(roll)]])
        r_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                        [0, 1, 0],
                        [-math.sin(pitch), 0, math.cos(pitch)]])
        r_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                        [math.sin(yaw), math.cos(yaw), 0],
                        [0, 0, 1]])

        # Apply the rotations to the vector
        v = np.dot(r_x, v)
        v = np.dot(r_y, v)
        v = np.dot(r_z, v)

        # Add in the center once again
        return (v[0] + center[0], v[1] + center[1], v[2] + center[2])

    def image_point_from_tf(self, point):
        # Gets a point on the picture from the tf coordinates
        x = point[0]
        y = point[1]
        z = point[2]

        # Calculate the current angles from the center
        theta = math.atan2(x, z)
        rho = math.atan2(y, z)

        # Find the height and width of the lens from the distance
        w = z * math.tan(self.max_theta)
        h = z * math.tan(self.max_rho)

        # Use ratio algebra to calculate where the image point is
        pix_x = int((x / w + 0.5) * self.pix_w)
        pix_y = int((y / h + 0.5) * self.pix_h)

        return (pix_x, pix_y)

    def corner_color(self, color):
        # Just make it less value
        hsv = colorsys.rgb_to_hsv(color[2]/255, color[1]/255, color[0]/255)
        rgb = colorsys.hsv_to_rgb(hsv[0], hsv[1], hsv[2] / 1.25)
        return (255 * rgb[2], 255 * rgb[1], 255 * rgb[0])

    def random_color(self):
        # B G R
        return (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256))

    def image_callback(self, image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        self.pix_h, self.pix_w, _ = image_cv.shape
        for marker in self.markers:
            ar_id = marker.id
            if ar_id == 0 or ar_id == 255:
                continue
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            z = marker.pose.pose.position.z
            center = (x, y, z)

            point = self.image_point_from_tf(center)

            # This is in quaternion units
            q = marker.pose.pose.orientation
            roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

            print "id:", ar_id
            print
            print "x:", x, "meters"
            print "y:", y, "meters"
            print "z:", z, "meters"
            print
            print "roll:", 180 * roll / 2 / np.pi, "degrees"
            print "pitch:", 180 * pitch / 2 / np.pi, "degrees"
            print "yaw:", 180 * yaw / 2 / np.pi, "degrees"
            print "="*25

            corners = []

            d = BOX_DIMENSION / 2

            # This creates a cube of dots around the AR tag
            """
            box_points = []
            for i in range(-1, 2):
                for j in range(-1, 2):
                    for k in range(-1, 2):
                        c_x = d * i + x
                        c_y = d * k + y
                        c_z = d * j + z
                        v = (c_x, c_y, c_z)
                        rot_v = self.rotated_vector(center, v, q)
                        box_points.append(self.image_point_from_tf(rot_v))
            """

            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i != 0 and j != 0:
                        c_x = i * d + x
                        c_y = j * d + y
                        c_z = z
                        v = (c_x, c_y, c_z)
                        rot_v = self.rotated_vector(center, v, q) 
                        corners.append(self.image_point_from_tf(rot_v))

            # Assign a color to each AR id number
            if not ar_id in self.ar_colors:
                self.ar_colors[ar_id] = self.random_color()

            if not ar_id in self.ar_corner_colors:
                self.ar_corner_colors[ar_id] = self.corner_color(self.ar_colors[ar_id])

            color = self.ar_colors[ar_id]
            corner_color = self.ar_corner_colors[ar_id]

            cv2.circle(image_cv, point, 8, color, 7)
            
            # Make the axis lines that show the pose better
            roll_color = (255, 0, 0)
            pitch_color = (0, 255, 0)
            yaw_color = (0, 0, 255)

            axis_d = d / 1.75

            roll_p = (center[0], center[1], center[2] + axis_d)
            pitch_p = (center[0], center[1] + axis_d, center[2])
            yaw_p = (center[0] + axis_d, center[1], center[2])

            roll_p = self.rotated_vector(center, roll_p, q)
            pitch_p = self.rotated_vector(center, pitch_p, q)
            yaw_p = self.rotated_vector(center, yaw_p, q)

            # draw axis
            cv2.line(image_cv, point, self.image_point_from_tf(roll_p), roll_color, 3)
            cv2.line(image_cv, point, self.image_point_from_tf(pitch_p), pitch_color, 3)
            cv2.line(image_cv, point, self.image_point_from_tf(yaw_p), yaw_color, 3)

            # Draw the sides of the box
            cv2.line(image_cv, corners[0], corners[1], corner_color, 5)
            cv2.line(image_cv, corners[1], corners[3], corner_color, 5)
            cv2.line(image_cv, corners[3], corners[2], corner_color, 5)
            cv2.line(image_cv, corners[2], corners[0], corner_color, 5)

            # for p in box_points:
                # cv2.circle(image_cv, p, 5, color, 3)

        if not self.markers:
            # just display image
            self.image_pub.publish(image_msg)
        else:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))


if __name__ == "__main__":
    try:
        rospy.init_node("ar_detection_node")
        node = ArTagDetectionDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()

