import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def forward(myMsg):
    myMsg.drive.steering_angle = 1
    myMsg.drive.speed = 1


if __name__ == "__main__":
    rospy.init_node('forwardNode')
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)
    msg = AckermannDriveStamped()
    forward(msg)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)

