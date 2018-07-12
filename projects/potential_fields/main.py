#!/usr/bin/env python
import rospy


def move_racecar(speed, steering_angle):
    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    drive_pub.publish(msg)
