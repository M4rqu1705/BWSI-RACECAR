#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class tag_state_machine:
    def __init__(self):
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 10)
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback, queue_size = 10)
        self.speed = 0
        self.steering_angle = 0

    def ar_callback(self, ardata):
        print(ardata)
        ids = [m.id for m in ardata.markers]
        if len(ids) == 0: print("no ids")
        self.respond(ids)

    def move_racecar(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        print(self.speed, self.steering_angle)
        self.drive_pub.publish(msg)

    def respond(self, tags):
        if 1 in tags:
            self.speed = 1

        if 2 in tags:
            self.speed = -1

        if 3 in tags:
            self.steering_angle = -1

        if 4 in tags:
            self.steering_angle = 1

        if 5 in tags:
            self.speed = 0
            self.steering_angle = 0
        
        self.move_racecar(self.speed, self.steering_angle)

if __name__ == "__main__":
    rospy.init_node("statemachine")
    sm = tag_state_machine()
    rospy.spin()
