#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

# Module with which to interpret images from the ZED encoded in a ROS format of sorts
from cv_bridge import CvBridge, CvBridgeError

class AR_Marker_Detector:

    def __init__(self):

        self.retrieved_ar_markers = None

        self.ar_sub = rospy.Subscriber(
                "/ar_pose_marker",
                AlvarMarkers,
                self.ar_callback,
                queue_size = 10)

        # Make publisher to be able to see video frame with detected face surrounded by square
        self.image_pub = rospy.Publisher(
                'ar_tag_detector',    # Topic to publish to
                Image,              # Datatype of message. DO NOT CHANGE
                queue_size=10)      # Amount of sends to make until waits for next send

        self.bridge = CvBridge()

    def ar_callback(self, ar_markers):
        self.retrieved_ar_markers = ar_markers.markers

    def process(self, processed_image):

        marker_ids = []

        for marker in self.retrieved_ar_markers:
            cv2.rectangle(
                    processed_image,
                    (marker.pose.pose.position.x, marker.pose.pose.position.y),
                    (marker.pose.pose.position.x + marker.width, marker.pose.pose.position.y + marker.height),
                    (255, 255, 255),
                    2)
            cv2.putText(
                    processed_image,
                    str(marker.id),
                    (marker.pose.pose.position.x + 5, marker.pose.pose.position.y + marker.height),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    3,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA)
            
            marker_ids.append(marker.id)

        output = marker_ids, marker.pose.pose.position.x

        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(
            processed_image,        # Processed image object
            encoding='bgr8'))                 # Image data type

        return output[0], output[1]



