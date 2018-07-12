#!/usr/bin/env python

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



