#!/usr/bin/env python
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers

class ar_pose_retriever:

    def __init__(self):
        self.pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.AlvarMarkers_callback, queue_size=1)

        self.ar_tag_list = 0

    def AlvarMarkers_callback(self,data):
        artag_list = []
        for marker in data.markers:
            #  print marker.id
            artag_list.append(
                    {"id":marker.id,
                        "x":marker.pose.pose.position.x, 
                        "y":marker.pose.pose.position.y,
                        "z":marker.pose.pose.position.z})
        
        sorted_list = sorted(artag_list, key=lambda k: k["z"])

        self.ar_tag_list = sorted_list

    def get_tag_list(self):
        if not isinstance(self.ar_tag_list, int) and len(self.ar_tag_list) > 0:
            return self.ar_tag_list
        else:
            return []


if __name__ == "__main__":
        try:
            rospy.init_node("ar_pose_retriever")
            node = ar_pose_retriever()
            print node.get_tag_list
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
