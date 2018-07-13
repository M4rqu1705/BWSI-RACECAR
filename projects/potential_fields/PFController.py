#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
class pfController:

    steeringScale = None
    speedScale = None
    backChargeSize = None
    totalReadings = None

    def __init__(self, inputSteeringScale, inputSpeedScale,  inputBackChargeSize):
        self.steeringScale = inputSteeringScale
        self.speedScale = inputSpeedScale
        self.backChargeSize = inputBackChargeSize

    def compute(self, scan):
        self.totalReadings = len(scan)
        myScan = self.partialScan(self.totalReadings/8, (7*self.totalReadings)/8, scan)
        partialReadings = len(myScan)
        angles = np.arange(self.totalReadings/8, (7*self.totalReadings)/8, dtype=float)
        angles = np.multiply(angles, (2*math.pi)/self.totalReadings)

        xArray = np.empty(partialReadings, dtype = float)
        yArray = np.empty(partialReadings, dtype = float)

        xArray = np.true_divide(np.cos(angles), np.power(myScan, .25))
        yArray = np.true_divide(np.sin(angles), np.power(myScan, .25))


        finalX = (np.sum(xArray)+ self.backChargeSize) * self.speedScale
        finalY = np.sum(yArray)*self.steeringScale

        return(finalY, finalX)

    def partialScan(self, start, end, scan):
        return (scan[start:end])
