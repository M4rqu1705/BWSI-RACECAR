import rospy
from sensor_msgs.msg import LaserScan


def scanCallback(scan, LaserScan):
    i = 0
    for range in scan.ranges:
        print(i + ',' + range)

if __name__ == '__main__':
    rospy.init_node('LidarReader')
    sub = rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size = 10)
    scan = LaserScan()

    
    
