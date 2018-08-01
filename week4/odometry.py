import rospy
import tf
from nav_msgs.msg import Odometry


def odometry_callback(data):
    q = data.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    print "Roll = %s\tPitch = %s\tYaw = %s" % (str(roll), str(pitch), str(yaw))

rospy.init_node('odometry_test')
rospy.Subscriber('/odom_corrected', Odometry, odometry_callback, queue_size = 10)

rospy.spin()
