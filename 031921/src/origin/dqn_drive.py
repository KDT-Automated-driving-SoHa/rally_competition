#!/usr/bin/env python

from model import *

class DQN:
    angle = 0
    speed = 0
    
    def __init__(self, hidden_size, pth_path):
        study_init(5, hidden_size, "DQN")
        self.load_pth(pth_path)

    def load_pth(path):
        study_model_load(path)

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = motor

    def set_data(self, data):
        pass

    def get_motor(self):
        return self.angle, self.motor

