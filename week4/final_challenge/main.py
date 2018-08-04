#!/usr/bin/env python

from time import time
import numpy as np
import rospy
import cv2

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

from cv_bridge import CvBridge, CvBridgeError

import ar_pose_retriever as ar
import tools as tool

laser_ranges = np.array([x for x in range(0, 1081)])
image = None
last_ar_marker = 22
ar_tag_list = []


def scan_callback(scan):
    global laser_ranges
    laser_ranges = np.array(scan.ranges)


def image_callback(img):
    global image
    img = cv_bridge.imgmsg_to_cv2(img, 'bgr8')
    img = img.medianBlur(img, 5)
    image = img


if __name__ == "__main__":
  
    # Initialize ROS node called wall_follower
    rospy.init_node('fina_challenge')

    # ========= ========= ========= ========= ========= ========= Object instantiation ==========  ==========  ==========  ==========  ==========  ========== #

    drive_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)

    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 10)

    img_sub = rospy.Subscriber('/zed/left/image_rect_color', Image, image_callback, queue_size=10)

    ar_sub = ar.ar_pose_retriever()

    steering_angle_pid = tool.Pid_Controller(1, 0, 0.3)

    potential_fields = tool.Potential_Fields_Controller(0.05, 0.05, 1.75, 0.5, 0.5)

    wall_follower = tool.Wall_Follower([1, 0, 0], [1/30.0, 0, 0])

    starter = tool.Start_Light()

    rate = rospy.Rate(30)

    # ========= ========= ========= ========= ========= ========= Object instantiation ==========  ==========  ==========  ==========  ==========  ========== #

    while not rospy.is_shutdown():
        ar_tag_list = ar_sub.get_tag_list()

        if len(ar_tag_list) > 0 and ar_tag_list[0]["z"] < 0.4:
        last_ar_marker = ar_tag_list[0]["id"]

        msg = AckermannDriveStamped()

        if last_ar_marker == 1:

        elif last_ar_marker == 2:

        elif last_ar_marker == 3:

        elif last_ar_marker == 4:

        elif last_ar_marker == 5:

        elif last_ar_marker == 6:

        elif last_ar_marker == 7:

        elif last_ar_marker == 8:

        elif last_ar_marker == 9:

        elif last_ar_marker == 10:

        elif last_ar_marker == 11:

        elif last_ar_marker == 12:

        elif last_ar_marker == 13:

        elif last_ar_marker == 14:

        elif last_ar_marker == 15:

        elif last_ar_marker == 16:

        elif last_ar_marker == 17:

        elif last_ar_marker == 18:

        elif last_ar_marker == 19:

        elif last_ar_marker == 20:



        pub.publish(msg)

      rate.sleep()
