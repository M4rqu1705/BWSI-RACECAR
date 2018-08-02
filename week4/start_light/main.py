import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

def image_callback(img):
    global image
    image = bridge.imgmsg_to_cv2(img, 'bgr8')

    image = cv2.medianBlur(image, 5)
     
    check_green()

def check_green():
    global image

    processed_image = image
    #  processed_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    processed_image = processed_image[0:processed_image.shape[0]/2]

    green_lower_hsv = np.array([70, 150, 100])
    green_higher_hsv = np.array([90, 255, 255])

    #  green_mask = cv2.inRange(processed_image, green_lower_hsv, green_higher_hsv)
    green_mask = cv2.inRange(processed_image, np.array([200, 200, 200]), np.array([255, 255, 255]))
    
    if np.sum(green_mask) > 4000000:
        print np.sum(green_mask)
        print 'Do not begin'
    else:
        print np.sum(green_mask)
        print '******Begin'
    

    #  green_mask = cv2.cvtColor(green_mask, cv2.COLOR_HSV2BGR)

    image_pub.publish(bridge.cv2_to_imgmsg(green_mask))


     


image = 0

bridge = CvBridge()

if __name__ == '__main__':
    
    rospy.init_node('bulb_detector')

    image_sub = rospy.Subscriber('/zed/left/image_rect_color', Image, image_callback, queue_size=10)
    image_pub = rospy.Publisher('/green_bulb_mask', Image, queue_size=10)

    rospy.spin()


