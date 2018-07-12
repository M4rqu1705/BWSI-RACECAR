#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class pd_controller:
    error = 0
    last_error = 0

    def __init__(self, kP, kD):
        self.kP = kP
        self.kD = kD
        self.error = 0
        self.last_error = 0

    def calculate(self, setpoint, process_variable):
        self.error = setpoint - process_variable
        derivative = self.error - self.last_error
        #rospy.loginfo(self.kD * derivative)
        return self.kP*self.error + self.kD*derivative

pub = None
sub = None
distFromWall = None
totalReadings = 0
desiredDist = 1
obstruction = False


def scanCallback(scan):
    totalReadings = len(scan.ranges)
    global obstruction
    global distFromWall
    minIndex = getMinIndex(scan,totalReadings / 6, totalReadings / 3)
    distFromWall = scan.ranges[minIndex]
    rospy.loginfo(distFromWall)
    if(scan.ranges[totalReadings/2] < .7):
        obstruction = True

def getMinIndex(scan, min, max):
    minDist = 500
    minIndex = 0
    for i in range(min, max):
        dist = scan.ranges[i]
        if dist < minDist:
            minDist = dist
            minIndex = i
    return minIndex

if __name__ == "__main__":
    rospy.init_node('SafetyNode')
    pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size = 10)
    sub = rospy.Subscriber('/scan_processed', LaserScan, scanCallback, queue_size = 10)
    rate = rospy.Rate(10)
    myPD = pd_controller(.4,.05)
    while not distFromWall:
        pass
    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()
        if not obstruction:
            msg.drive.speed = 1
            steering_ang = myPD.calculate(desiredDist, distFromWall)
           # rospy.loginfo(steering_ang)
            msg.drive.steering_angle = steering_ang
        else:
            msg.drive.speed = 0
        pub.publish(msg)
        rate.sleep()

    
