#!/usr/bin/env python

class BACKWARD:
    angle = 0
    speed = 0
    
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_data(self, data):
        pass

    def get_motor(self):
        return self.angle, self.speed