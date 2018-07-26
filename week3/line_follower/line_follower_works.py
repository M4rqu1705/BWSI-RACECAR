import numpy as np      # Used for vectorized numerical operations
import cv2              # Used for image analysis
import rospy            # Used to interact with ROS

# Message with which to update Ackermann Drive values such as speed and steering angle
from ackermann_msgs.msg import AckermannDriveStamped

# Module with which to interpret images from the ZED encoded in a ROS format of sorts
from cv_bridge import CvBridge, CvBridgeError

# Message with which to receive Images from the ZED
from sensor_msgs.msg import Image

from ar_track_alvar_msgs.msg import AlvarMarkers

class pid_controller:

    def __init__(self, KP, KI, KD):
        ''' Constructor: KP, KI, KD'''
        self.KP = KP    # Initialize KP to a value specified by constructor parameters
        self.KI = KI    # Initialize KI to a value specified by constructor parameters
        self.KD = KD    # Initialize KD to a value specified by constructor parameters

        # Initialize error, last error and integral to 0 by default
        self.error = self.last_error = self.integral = 0

        # Store the current time as the last_count variable
        self.last_count = cv2.getTickCount()

    def calculate(self, setpoint, process_value):
        ''' Calculate desired output: setpoint, process_value '''

        # P - Calculate error
        self.error = setpoint - process_value

        # ID - Change in time later used to improve integral and derivative calculations
        change_in_time = (cv2.getTickCount() - self.last_count) / 1000000000.0 # Time in microseconds, so it has to be converted to seconds

        # I - Accumulate error
        self.integral += self.error * change_in_time
        # I - Reset Integral if got to destination. Prevents overshoot
        if self.error == 0:
            self.integral = 0

        # D - Calculate 'break force'
        derivative = (self.error - self.last_error) / change_in_time

        # PID - Multiply gains with respective products
        output = self.KP * self.error + self.KI * self.integral + self.KD * derivative

        # D - Update last error for next cycle
        self.last_error = self.error
        # ID - Update last_count for next cycle
        self.last_count = cv2.getTickCount()

        return output



class line_follower():

    def __init__(self):

        
        self.image = 0       # Initialize variable to hold image
        self.image_width = 0    # Initialize variable to hold image width
        self.image_height = 0   # Initialize variable to hold image height
        self.speed = 0          # Initialize variable to hold RACECAR speed
        self.steering_angle = 0 # Initialize variable to hold RACECAR steering angle

        # Subscribe to ZED camera topic to frequently receive video 
        self.sub = rospy.Subscriber('/zed/left/image_rect_color',
                Image,                  # Data type. Do not change
                self.image_callback,    # Callback function. No need to change
                queue_size = 10)        # Not so necessary, and can be any number
        # Create publisher to Ackermann Drive topic
        self.drive_pub = rospy.Publisher(                            
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",   # Topic
                AckermannDriveStamped,                              # Datatype
                queue_size=10)
        # Make publisher to be able to see mask or video frame
        self.image_pub = rospy.Publisher('line_follower_image', Image, queue_size=10)
        # Subscribe to the ar_pose_marker topic to detect and receive ar markers
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size=10)

        self.rate = rospy.Rate(20)      # Instantiate Rate object at 20 hertz

        # Used to recuperate images from rostopic
        self.bridge = CvBridge()        # Instantiate CvBridge object

        # Used to recalculate steering angle based on position of line in screen
        self.PID = pid_controller(0.009, 0, 0)

        self.ar_detected = "none"



    def move_racecar(self):
        msg = AckermannDriveStamped()   # Instantiate AckermannDriveStamped object
        msg.drive.speed = self.speed    # Update speed to be sent
        # Update steering angle to be sent
        msg.drive.steering_angle = self.steering_angle

        # Log information so it can be read by user
        rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(self.speed), str(self.steering_angle)))

        # Publish formatted message
        self.drive_pub.publish(msg)


    def ar_callback(self, ar_markers):
        #  print ar_markers.markers[0].pose.pose.position.z

        if len(ar_markers.markers) > 0 and ar_markers.markers[0].pose.pose.position.z > 0.75 and ar_markers.markers[0].pose.pose.position.z < 1.5:
            self.ar_detected = "forward"
        elif len(ar_markers.markers) > 0 and ar_markers.markers[0].pose.pose.position.z < 0.75:
            self.ar_detected = "stop" 
        else:
            self.ar_detected = "none"


    def image_callback(self, image):

        # Convert recuperated image to a usable bgr format
        self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")

        # Resize image to a fourth of its size so less processing power is consumed later
        self.image = cv2.resize(self.image, (0,0), fx=0.25, fy=0.25)

        # Update the height and width with final values
        self.image_height, self.image_width, _ = self.image.shape

        # Blur the image to reduce noise
        self.image = cv2.GaussianBlur(self.image, (7, 7), 0)



    def follow_line(self):
        if isinstance(self.image, int):
            return


        # Crop top half of image to be used
        cropped_image = self.image[self.image_height*1/2:self.image_height, :, :]

        # Convert image to HSV
        cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        # Determine HSV threshold values for yellow, green, blue, and orange
        yellow_lower_hsv = np.array([25,150,150])
        yellow_top_hsv = np.array([35, 255, 255])
        

        green_lower_hsv = np.array([70,150, 150])
        green_top_hsv = np.array([90, 255, 255])

        blue_lower_hsv = np.array([95,100, 150])
        blue_top_hsv = np.array([115, 250, 250])

        orange_lower_hsv = np.array([15, 175, 150])
        orange_top_hsv = np.array([25, 250, 250])

        # Make masks taking into account desired threshold values for all colors
        yellow_mask = cv2.inRange(cropped_image, yellow_lower_hsv, yellow_top_hsv)
        green_mask = cv2.inRange(cropped_image, green_lower_hsv, green_top_hsv)
        blue_mask = cv2.inRange(cropped_image, blue_lower_hsv, blue_top_hsv)
        orange_mask = cv2.inRange(cropped_image, orange_lower_hsv, orange_top_hsv)

        # Quick make mask copies to prepare for contour detection
        yellow_mask_copy = np.array(yellow_mask)
        green_mask_copy = np.array(green_mask)
        blue_mask_copy = np.array(blue_mask)
        orange_mask_copy = np.array(orange_mask)

        # Find the contours of lines in each mask
        _, yellow_contours, hierarchy = cv2.findContours(yellow_mask_copy, 1, cv2.CHAIN_APPROX_NONE)
        _, green_contours, hierarchy = cv2.findContours(green_mask_copy, 1, cv2.CHAIN_APPROX_NONE)
        _, blue_contours, hierarchy = cv2.findContours(blue_mask_copy, 1, cv2.CHAIN_APPROX_NONE)
        _, orange_contours, hierarchy = cv2.findContours(orange_mask_copy, 1, cv2.CHAIN_APPROX_NONE)

        # Prepare variables to store each color's biggest contour
        biggest_yellow_contour, biggest_green_contour, biggest_blue_contour, biggest_orange_contour = 0, 0, 0, 0

        # Make a boolean list to list what contours were found
        colors_found = [False, False, False, False]

        if len(yellow_contours) > 0:
            # Find the biggest contour, if the contour list is bigger than 0
            biggest_yellow_contour = max(yellow_contours, key=cv2.contourArea)
            # Update the yellow's entry in the colors_found array
            if cv2.contourArea(biggest_yellow_contour) > 30:
                colors_found[0] = True
        if len(green_contours) > 0:
            # Find the biggest contour, if the contour list is bigger than 0
            biggest_green_contour = max(green_contours, key=cv2.contourArea)
            # Update the green's entry in the colors_found array
            if cv2.contourArea(biggest_green_contour) > 30:
                colors_found[1] = True
        if len(blue_contours) > 0:
            # Find the biggest contour, if the contour list is bigger than 0
            biggest_blue_contour = max(blue_contours, key=cv2.contourArea)
            # Update the blue's entry in the colors_found array
            if cv2.contourArea(biggest_blue_contour) > 30:
                colors_found[2] = True
        if len(orange_contours) > 0:
            # Find the biggest contour, if the contour list is bigger than 0
            biggest_orange_contour = max(orange_contours, key=cv2.contourArea)
            # Update the orange's entry in the colors_found array
            if cv2.contourArea(biggest_orange_contour) > 30:
                colors_found[3] = True

        #  contour_list = [cv2.contourArea(biggest_yellow_contour), cv2.contourArea(biggest_green_contour), cv2.contourArea(biggest_blue_contour), cv2.contourArea(biggest_orange_contour)]
#  
        #  if cv2.contourArea

        moment = None

# _____________________________________ State Machine _____________________________________ #
        ''' Since the center_x and center_y values depend on the moments of the contour, 
            change the moment based on the level of priority we established on previous
            parts of program and calculate the center as we did previously '''

        if colors_found[0]:
            # Store the yellow contour's moments into a variable
            moment = cv2.moments(biggest_yellow_contour)
            print 'Following Yellow Line'
        elif colors_found[1]:
            # Store the green contour's moments into a variable
            moment = cv2.moments(biggest_green_contour)
            print 'Following green Line'
        elif colors_found[2]:
            # Store the blue contour's moments into a variable
            moment = cv2.moments(biggest_blue_contour)
            print 'Following blue Line'
        elif colors_found[3]:
            # Store the orange contour's moments into a variable
            moment = cv2.moments(biggest_orange_contour)
            print 'Following orange Line'
        else:
            # Two different images to publish to the line_follower topic
            #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(cropped_image, cv2.COLOR_HSV2BGR), encoding="bgr8"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR), encoding="bgr8"))

            print "Line not found. Steering angle unchanged" 

            # Slowly drive back and try to rectify according to the last steering angle to prevent RACECAR from not touching the line
            self.speed, self.steering_angle = -0.25, -self.steering_angle

            # Don't forget to move RACECAR according to the new values
            self.move_racecar()

            # Exit the function early, since no line was found
            return
# _____________________________________ State Machine _____________________________________ #

     
        # Prepare variables to store the center of the contour
        center_x, center_y = 0, 0

        try:
            center_x = int(moment['m10']/moment['m00'])
        except ZeroDivisionError:
            center_x = 0
            
        try:
            center_y = int(moment['m01']/moment['m00'])
        except ZeroDivisionError:
            center_y = 0


        print "Error = %s" % (str(self.image_width/4-center_x/2))


        self.steering_angle = self.PID.calculate( self.image_width/4, center_x/2)
        self.speed = 1

        # Two different images to publish to the line_follower topic
        #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(cropped_image, cv2.COLOR_HSV2BGR), encoding="bgr8"))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR), encoding="bgr8"))

        # Don't forget to move RACECAR according to the new values
        self.move_racecar()




if __name__ == "__main__":
    rospy.init_node('line_follower')
    node = line_follower()

    while not rospy.is_shutdown():
        if  node.ar_detected == "none":
            node.follow_line()
        elif node.ar_detected == "forward":
            node.speed = 0.5
            node.steering_angle = -0.2
            node.move_racecar()
        elif node.ar_detected == "stop":
            node.speed = 0
            node.steering_angle = 0
            node.move_racecar()
        node.rate.sleep()
