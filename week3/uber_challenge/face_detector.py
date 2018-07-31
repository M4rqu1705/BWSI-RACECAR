#!/usr/bin/env python

import numpy as np

# Module with which to recognize and categorize faces
import face_recognition as fr
import cv2
import rospy
from sensor_msgs.msg import Image
# Module with which to interpret images from the ZED encoded in a ROS format of sorts
from cv_bridge import CvBridge, CvBridgeError

# parameter = image
# return [face_mid_x, ar_destination, 

class Face_Detector:

    def __init__(self):

        # Make publisher to be able to see video frame with detected face surrounded by square
        self.image_pub = rospy.Publisher(
                'face_detector',    # Topic to publish to
                Image,              # Datatype of message. DO NOT CHANGE
                queue_size=10)      # Amount of sends to make until waits for next send

        # Make sure OpenCV is running optimized code. Increases efficiency
        cv2.setUseOptimized(True)

        # Load required XML classifier
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontface_default.xml')

        self.bridge = CvBridge()
        
        
        # Load the png files to numpy arrays
        self.ahmad_image = fr.load_image_file('ahmad.png')
        self.belyi_image = fr.load_image_file('belyi.png')
        self.cruz_image = fr.load_image_file('cruz.png')
        self.fishberg_image = fr.load_image_file('fishberg.png')
        self.hassan_image = fr.load_image_file('hassan.png')
        self.mali_image = fr.load_image_file('mali.png')
        self.nguyen_image = fr.load_image_file('nguyen.png')
        self.plancher_image = fr.load_image_file('plancher.png')

        # Get encodings for first face found
        self.ahmad_encoding = fr.face_encodings(self.ahmad_image)[0]
        self.belyi_encoding = fr.face_encodings(self.belyi_image)[0]
        self.cruz_encoding = fr.face_encodings(self.cruz_image)[0]
        self.fishberg_encoding = fr.face_encodings(self.fishberg_image)[0]
        self.hassan_encoding = fr.face_encodings(self.hassan_image)[0]
        self.mali_encoding = fr.face_encodings(self.mali_image)[0]
        self.nguyen_encoding = fr.face_encodings(self.nguyen_image)[0]
        self.plancher_encoding = fr.face_encodings(self.plancher_image)[0]

#  ==========================Edit==================================================
        # List of TA faces. Order is important, since it will later be used in a int to string mapping
        self.known_faces = [self.ahmad_encoding,
                self.belyi_encoding,
                self.cruz_encoding,
                self.fishberg_encoding,
                self.hassan_encoding,
                self.mali_encoding,
                self.nguyen_encoding,
                self.plancher_encoding]

        # Dictionary of number keys to TA last names strings. Indicates which TA corresponds to which index
        self.ta_dict = {0:"Ahmad", 
                1:"Belyi", 
                2:"Cruz", 
                3:"Fisberg", 
                4:"Hassan", 
                5:"Mali", 
                6:"Nguyen", 
                7:"Plancher"}
#  ==========================Edit==================================================


    def process(self, retrieved_image):


        # Convert image to improve processing speed
        gray_image = cv2.cvtColor(
                retrieved_image,        # Image to convert to grayscale
                cv2.COLOR_BGR2GRAY)     # Specifies that the image will be converted to grayscale

        # Detect faces on image and store them on faces variable
        faces = self.face_cascade.detectMultiScale(
                gray_image,     # Image to perform detection in
                1.25,            # Scale Factor. ~ Adjusts sensitivity. 1.3 is pretty good (I found number through trial and error)
                5)              # Min Neighbors. IDK what it does, but documentation and lab recommend it


        output = []     # List in which to store the return values

        # Before recognizing faces, we must make sure there is a face on the image
        if len(faces) > 0:
            # Extract x and y coordinates from face and its width and height
            x, y, width, height = faces[0]

            # Select the detected face as a region of interest
            roi = retrieved_image[y:y+height, x:x+width]
            
            # Find the detected face's encoding. Time-consuming process, but effective
            unknown_encoding = fr.face_encodings(roi)[0]

            # Find differences/distances from sample known faces' encodings
            distances = fr.face_distance(self.known_faces, unknown_encoding)

            # Look for the index of the face with the least distance/most similarities
            minimum_distance_index = np.argmin(distances)
            
            # Prepare output: bool(face_detected), int(face_middle_x), int(ta_number)
            output +=  True, minimum_distance_index, int(x + width/2)

            # Surround detected face with a rectangle
            cv2.rectangle(retrieved_image, (x,y), (x+width, y+width), (255, 0, 255), 2)

            # Put the name of the detected TA at the bottom left part of the rectangle
            cv2.putText(
                    retrieved_image,                        # Image to put text in
                    self.ta_dict[minimum_distance_index],   # Name of TA. Dictionary translates from number to a string
                    (x+5, y+height),                          # Bottom left coordinate for text
                    cv2.FONT_HERSHEY_SIMPLEX,               # Font type
                    3,                                      # Font size
                    (255, 255, 255),                        # Font color
                    2,                                      # Font weight
                    cv2.LINE_AA)                            # Line type. DO NOT CHANGE
        else:
            self.detected_face = "No face detected"
            output +=  False, 0, -1

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(retrieved_image, encoding="passthrough"))

        return output[0], output[1], output[2]
