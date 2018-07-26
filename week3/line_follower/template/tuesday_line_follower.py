import numpy as np      # Used for vectorized numerical operations
import cv2              # Used for image analysis
import rospy            # Used to interact with ROS

# Message with which to update Ackermann Drive values such as speed and steering angle
from ackermann_msgs.msg import AckermannDriveStamped

# Module with which to interpret images from the ZED encoded in a ROS format of sorts
from cv_bridge import CvBridge, CvBridgeError

# Message with which to receive Images from the ZED
from sensor_msgs.msg import Image

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

        
        self.image = None       # Initialize variable to hold image
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
        # Create publisher to publish line follower mask images. Important when tuning HSV threshold values
        self.image_pub = rospy.Publisher('line_follower_image', Image, queue_size=10)
        self.rate = rospy.Rate(30)      # Instantiate Rate object at 30 hertz

        # Used to recuperate images from rostopic
        self.bridge = CvBridge()        # Instantiate CvBridge object

        # Used to recalculate steering angle based on position of line in screen
        self.PID = pid_controller(0.001, 0, 0.001)



    def move_racecar(self):
        msg = AckermannDriveStamped()   # Instantiate AckermannDriveStamped object
        msg.drive.speed = self.speed    # Update speed to be sent

        # Update steering angle to be sent
        msg.drive.steering_angle = self.steering_angle

        # Log information so it can be read by user
        rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(self.speed), str(self.steering_angle)))

        # Publish formatted message
        self.drive_pub.publish(msg)



    def image_callback(self, image):

        # Convert recuperated image to a usable bgr format
        self.img = self.bridge.imgmsg_to_cv2(image, "bgr8")

        # Resize image to a fourth of its size so less processing power is consumed later
        self.img = cv2.resize(self.img, (0,0), fx=0.25, fy=0.25)

        # Update the height and width with final values
        self.image_height, self.image_width, _ = self.img.shape

        # Blur the image to reduce noise
        self.img = cv2.GaussianBlur(self.img, (5, 5), 0)


    def follow_line(self):

        # Crop top half of image to be used
        cropped_image = self.img[self.image_height*1/2:self.image_height, :, :]

        # Convert image to HSV
        cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        # Determine HSV threshold values
        lower_hsv = np.array([0, 150, 175])
        top_hsv = np.array([25, 255, 250])

        # Make mask taking into account desired threshold values
        mask = cv2.inRange(cropped_image, lower_hsv, top_hsv)

        # Find the contours of lines in the mask
        _, contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:

            # Find the biggest contour, if the contour list is bigger than 0
            biggest_contour = max(contours, key=cv2.contourArea)
            # Store the biggest contour's moments into a variable
            moment = cv2.moments(biggest_contour)
     
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


            cv2.circle(self.img, (center_x, center_y), 5, (255, 255, 255), -1)
     
            print "Error = %s" % (str(self.image_width/2-center_x))


            self.steering_angle = self.PID.calculate( self.image_width/2, center_x+10)

        else:
            print "Line not found. Steering angle unchanged" 
            #  print "Line not found"
            #  self.steering_angle = 0

        # Default speed for initial phases of program
        self.speed = 1               

        #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(cropped_image, cv2.COLOR_HSV2BGR), encoding="bgr8"))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), encoding="bgr8"))

        self.move_racecar()



if __name__ == "__main__":
    rospy.init_node('line_follower')
    node = line_follower()

    while not rospy.shut_downjkkk
