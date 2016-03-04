"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from .pwm import Pwm
import time

class Servo:
    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self._pwm = Pwm(number)
        self._pwm.enable()
        self._pwm.set_frequency(1000/20.0)
        
    def go_to(self, angle):
        #convert angle to pwm
        width = 0.75 + ((2.25-0.75)/180 * (angle + 90))
        print(width)
        self._pwm.set_duty_cycle(100*width/20.0)
