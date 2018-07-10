#!/usr/bin/env python

import rospy
from std_msgs import String

def publish_message(origin, destination, out_string):
    pub = rospy.Publisher(destination, String, queue_size=10)
    rospy.init_node(origin,  anonymous=True)
    rospy.loginfo(out_string)
    pub.publish(out_string)

if __name__ == '__main__':
    try:
        publish_message('forward.py', '/vesc/ackermann_cmd_mux/input/navigation/ackerman_msgs/AckermannDriveStamped', '{header: auto, drive: {steering_angle:0.2, speed:1.2}}')
    except rospy.ROSInterruptException:
        pass
