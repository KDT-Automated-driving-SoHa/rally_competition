#!/usr/bin/env python
# -*- coding: utf-8 -*-
from handler import AbstractHandler
from motor_info import MotorInfo
from module.stop_line.preprocessing import preprocessing_stopline
from module.stop_line.stop_line import is_detect_crossline

class StopLineHandler(AbstractHandler):

    def __init__(self, speed=50):
        self.speed = speed


    def handle(self, frame):
        preprcessed_frame = preprocessing_stopline(frame, thres_L=180)
        ret, where = is_detect_crossline(preprcessed_frame, self.speed)

        if ret:
            motor_info_list = []

            if where == "up":
                motor_info_list.append(MotorInfo(angle=0, speed=self.speed, iterations=15))
            motor_info_list.append(MotorInfo(angle=0, speed=0, iterations=30, delay_sec=0.1))
            motor_info_list.append(MotorInfo(angle=0, speed=self.speed*2//3, iterations=20, delay_sec=0.1))
            
            return motor_info_list
        else:
            return self._next_handler.handle(frame)
