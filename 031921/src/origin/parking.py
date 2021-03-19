#!/usr/bin/env python

import cv2, math
import numpy as np
from tf.transformations import euler_from_quaternion

class PARK:
    angle = 0
    speed = 0
    
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = motor

    def set_data(self, data):
        pass

    def get_motor(self):
        return self.angle, self.motor
