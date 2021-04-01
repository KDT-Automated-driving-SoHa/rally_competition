#!/usr/bin/env python
# -*- coding: utf-8 -*-
from handler import AbstractHandler
from module.infos.motor_info import MotorInfo
from module.stop_line.preprocessing import preprocessing_stopline
from module.stop_line.stop_line import is_detect_crossline

class StopLineHandler(AbstractHandler):

    def __init__(self, speed=50):
        self.speed = speed


    def handle(self, handler_info):
        frame = handler_info.image
        preprcessed_frame = preprocessing_stopline(frame, thres_L=200)
        ret, where = is_detect_crossline(preprcessed_frame, self.speed)

        if ret:
            motor_info_list = []

            cycle = (600-where)//20
            motor_info_list.append(MotorInfo(angle=0, speed=self.speed*2//3, iterations=cycle, delay_sec=0.05))
            motor_info_list.append(MotorInfo(angle=0, speed=0, iterations=60, delay_sec=0.1)) # stop for 6 sec
            motor_info_list.append(MotorInfo(angle=0, speed=self.speed*2//3, iterations=20, delay_sec=0.1))
            handler_info.laps_count+=1
            print("laps:",handler_info.laps_count)
            
            return motor_info_list, handler_info
        else:
            return self._next_handler.handle(handler_info)
