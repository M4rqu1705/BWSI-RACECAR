#/usr/bin/env python
import rospy

class pid_controller:
    error, last_error, integral = 0, 0, 0

    def __init__(self, kP, kI, kD, dT):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.dT = dT / 1000
        self.error = 0
        self.last_error = 0
        self.integral = 0

    def reset(self, kP, kI, kD, dT):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.dT = dT / 1000
        self.error = 0
        self.last_error = 0
        self.integral = 0

    def calculate(self, setpoint, process_variable):

        # Update the error based on parameters
        self.error = setpoint - process_variable

        # Re-calculate integral -- Accumulate error
        self.integral += self.error*self.dT
        # Reset integral if error == 0, so it doesn't overshoot
        if self.error == 0:
            self.integral  = 0

        # Calculate derivative so "brakes" work
        derivative = (self.error - self.last_error) / self.dT

        # Calculate the output based on gains and calculations
        output = self.kP * self.error + self.kI * self.integral + self.kD * derivative

        # Update the last_error
        self.last_error = self.error

        return output
