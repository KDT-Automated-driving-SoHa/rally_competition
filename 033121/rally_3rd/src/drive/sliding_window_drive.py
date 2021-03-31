#!/usr/bin/env python
#-*- coding: utf-8 -*-

from motor_info import MotorInfo
from utils.linear_functions import *
from utils.discreate_filter import DiscreateFilter
from module.image_processing import preprocessing_sliding_window
from module.sliding_window import sliding_window, get_steering_angle_from_linear_function


class SlidingWindowDrive():

    def __init__(self):
        self.filter_deg = DiscreateFilter(f_cut=4000, freq=10000)
        

    def _sliding_window_drive(self, frame):
        choosen_left, choosen_right = sliding_window(frame)

        ret_left, linear_func_left = get_sliding_window_function(choosen_left)
        ret_right, linear_func_right = get_sliding_window_function(choosen_right)

        if ret_left and ret_right:
            steering_angle = get_steering_angle_from_linear_function((linear_func_left + linear_func_right)/2, frame)
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_left:
            steering_angle = get_steering_angle_from_linear_function(linear_func_left, frame, rel_x_ratio=1.0)
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_right:
            steering_angle = get_steering_angle_from_linear_function(linear_func_right, frame, rel_x_ratio=1.0)
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        else:
            steering_angle = self.filter_deg.prev_lpf
        steering_angle = self.filter_deg.get_lpf(steering_angle)[0]

        """
        Explain Matrix
        """
        # vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        # explain_merge = np.hstack((explain2, vertical_line, explain1))
        # return steering_angle, explain_merge
        return steering_angle


    def get_motor_info(self, frame):
        motor_angle = self._sliding_window_drive(frame)
        abs_motor_angle = max(0, get_linear_steering_angle2(abs(motor_angle)))

        angle = np.clip(np.sign(motor_angle) * abs_motor_angle, -50, 50)
        speed = get_linear_speed2(angle, 50)

        return MotorInfo(angle, speed)
