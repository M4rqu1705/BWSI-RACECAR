#!/usr/bin/env python
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers

class ar_pose_retriever:

    def __init__(self):
        self.pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.AlvarMarkers_callback, queue_size=1)

        self.ar_tag_list

    def AlvarMarkers_callback(self,data):
        artag_list = []
        for marker in data.markers:
            artag_list.append(
                    {"id":data.marker.id,
                        "x":data.marker.pose.pose.position.x, 
                        "y":data.marker.pose.pose.position.y,
                        "z":data.marker.pose.pose.position.z})
        
        sorted_list = sorted(artag_list, key=lambda k: k["z"])

        self.ar_tag_list = sorted_list


if __name__ == "__main__":
        try:
            rospy.init_node("ar_pose_retriever")
            node = artag_test()
            print node.ar_tag_list
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
