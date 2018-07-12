#!/usr/bin/env python

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
        return self.kP*self.error + self.kD*derivative

