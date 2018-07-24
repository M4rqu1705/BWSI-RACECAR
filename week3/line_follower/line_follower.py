import numpy as np
import cv2
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class pid_controller:

    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.error = self.last_error = self.integral = 0
        self.last_count = cv2.getTickCount()

    def calculate(setpoint, process_value):

        # P - Calculate error
        self.error = setpoint - process_value

        # ID - Change in time later used to improve calculations
        change_in_time = (cv2.getTickCount() - self.last_count) * 0.001

        # I - Accumulate error
        self.integral += self.error * change_in_time
        # I - Reset Integral if got to destination. Prevents overshoot
        if self.error == 0:
            self.integral = 0

        # D - Calculate 'break force'
        derivative = (self.error - self.last_error) / change_in_time

        # PID - Multiply gains with respective products
        output = self.KP * self.error + self.KI * integral + self.KD * derivative

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
        self.sub = rospy.Subscriber('/zed/right/image_raw_color',
                Image,                  # Data type. Do not change
                self.image_callback,    # Callback function. No need to change
                queue_size = 10)        # Not so necessary, and can be any number
        # Create publisher to Ackermann Drive topic
        self.drive_pub = rospy.Publisher(                            
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",   # Topic
                AckermannDriveStamped,                              # Datatype
                queue_size=10)
        self.rate = rospy.Rate(30)      # Instantiate Rate object at 30 hertz

        # Used to recuperate images from rostopic
        self.bridge = CvBridge()        # Instantiate CvBridge object

        # Used to recalculate steering angle based on position of line in screen
        self.PID = pid_controller(1, 0, 0)



    def move_racecar(self):
        msg = AckermannDriveStamped()   # Instantiate AckermannDriveStamped object
        msg.drive.speed = self.speed    # Update speed to be sent
        # Update steering angle to be sent
        msg.dirve.steering_angle = self.steering_angle

        # Log information so it can be read by user
        rospy.loginfo("Speed = %s,\tSteering Angle = %s" % (str(self.speed), str(self.steering_angle)))

        # Publish formatted message
        self.drive_pub.publish(msg)



    def image_callback(self, image):

        # Convert recuperated image to a usable bgr format
        self.img = self.bridge.imgmsg_to_cv2(image, "bgr8")

        # Update the height and width variables temporarily with the current size
        self.img_height, self.img_width, _ = self.img.shape

        # Resize image to a fourth of its size so less processing power is consumed later
        self.img = cv2.resize(self.img, (0,0), fx=0.25, fy=0.25)

        # Update the height and width with final values
        self.img = self.bridge.imgmsg_to_cv2(image, "bgr8")

        # Blur the image to reduce noise
        self.img = cv2.GaussianBlur(self.img, (5, 5), 0)

        self.follow_line()      # Call follow line function



    def follow_line(self):

        # Crop top third of image to be used
        image = self.img[int(self.image_height*(2.0/3.0)):self.image_height, :]

        # Convert image to HSV
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Determine HSV threshold values
        lower_hsv = np.array([0, 0, 0])
        top_hsv = np.array([359, 255, 255])

        # Make mask taking into account desired threshold values
        mask = cv2.inRange(image, lower_hsv, top_hsv)

        # Find the contours of lines in the mask
        _, contours, hierarchy = cv2.findContours(mask[:], 1, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:

            # Find the biggest contour, if the contour list is bigger than 0
            biggest_contour = max(contours, key=cv2.contourArea)
            # Store the biggest contour's moments into a variable
            moment = cv2.moments(biggest_contour)
     
            # Prepare variables to store the center of the contour
            center_x = 0 

            try:
                center_x = int(moment['m10']/moment['m00'])
            except ZeroDivisionError:
                center_x = 0


            cv2.circle(self.img, (center_x, center_y), 5, (255, 255, 255), -1)
     
            print "Error = %s" % (str(self.img_width/2-center_x))

            self.steering_angle = self.PID.calculate(self.img_width/2, center_x)

        else:
            print "Line not found. Steering angle unchanged" 

        # Default speed for initial phases of program
        self.speed = 0.5

        self.move_racecar()


if __name__ == "__main__":
    rospy.init_node('line_follower')
    node = line_follower()

    rospy.spin()
