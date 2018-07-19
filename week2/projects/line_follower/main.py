#/usr/bin/env python
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class pid_controller:
    error = last_error = integral = 0

    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.error = self.last_error = self.integral = 0

    def calculate(setpoint, process_value):
        self.error = setpoint - process_value

        self.integral += self.error
        if self.error == 0:
            self.integral = 0

        derivative = self.error - self.last_error

        output = self.KP * self.error + self.KI * integral + self.KD * derivative

        self.last_error = self.error

        return output

class line_follower:

    def __init__(self, PID_constants):
        self.PID = pid_controller(PID_constants[0], PID_constants[1], PID_constants[2])
        self.speed, self.steering_angle = 0, 0

        rospy.init_node('line_follower_node')

        drive_publisher = rospy.Publisher("/vesc/high_level/ackermann_cmd/mux/input/nav_0", AckermannDriveStamped, queue_size = 10)

        rate = rospy.Rate(30)

    def move_racecar(self):

        # Print out values before capping them
        rospy.loginfo(("Speed = %s,\tSteering_angle = %s" % (str(self.speed)), str(self.steering_angle)))

        # Cap speed
        if self.speed > 1:
            self.speed = 1
        elif self.speed < -1:
            self.speed = -1

        # Cap steering_angle
        if self.steering_angle > 1:
            self.steering_angle = 1
        elif self.steering_angle < -1:
            self.steering_angle = -1

        # Prepare message
        msg = AckermannDriveStamped
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle

        # Publish message
        drive_pub.publish(msg)

    def controller(line_middle, current_middle):
        self.steering_angle = self.PID.calculate(line_middle, current_middle)
        self.speed = 0.5
        self.move_racecar()
        self.rate()


