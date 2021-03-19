#!/usr/bin/env python

class YOLO:
    angle = 0
    speed = 0
    
    def __init__(self, follow):
        self.follow = follow

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = motor

    def set_data(self, data):
        pass

    def get_motor(self):
        return self.angle, self.motor
