#!/usr/bin/env python
#-*- coding: utf-8 -*-
from handler import AbstractHandler
from module.infos.motor_info import MotorInfo
from module.utils.linear_functions import *
from module.utils.discreate_filter import DiscreateFilter
from module.sliding_window.preprocessing import preprocessing_sliding_window
from module.sliding_window.sliding_window import get_points_with_sliding_window,            \
                                                get_steering_angle_from_linear_function,    \
                                                get_steering_angle_from_linear_function2


class SlidingWindowHandler(AbstractHandler):

    def __init__(self):
        self.filter_deg = DiscreateFilter(f_cut=4000, freq=10000)
        

    def handle(self, handler_info):
        frame = handler_info.image
        
        # 영상 전처리
        preprcessed_frame = preprocessing_sliding_window(frame)

        # 슬라이딩 윈도우로 motor_angle(steering_angle) 계산
        motor_angle = self._get_steering_angle2(preprcessed_frame)

        # 설정된 2차함수에 따라 angle 값을 새로 계산
        abs_motor_angle = max(0, get_linear_steering_angle2(abs(motor_angle)))

        # angle 값 범위 조정 (-50 ~ 50)
        angle = np.clip(np.sign(motor_angle) * abs_motor_angle, -50, 50)

        # 설정된 2차함수에 따라, 해당 angle에 대한 speed 값을 새로 계산
        speed = get_linear_speed2(angle, 50)

        return MotorInfo(angle, speed)


    def _get_steering_angle(self, frame):
        # 슬라이딩 윈도우를 통해 왼쪽/오른쪽 영역에서 검출된 차선의 점 추출
        choosen_left, choosen_right = get_points_with_sliding_window(frame)

        # 검출된 차선의 점에 맞는 2차함수 계산
        ret_left, linear_func_left = get_sliding_window_function(choosen_left)
        ret_right, linear_func_right = get_sliding_window_function(choosen_right)

        # 각 2차함수를 통해 steering angle 계산
        if ret_left and ret_right:
            steering_angle = get_steering_angle_from_linear_function((linear_func_left + linear_func_right)/2, frame)
        elif ret_left:
            steering_angle = get_steering_angle_from_linear_function(linear_func_left, frame, rel_x_ratio=1.2)
        elif ret_right:
            steering_angle = get_steering_angle_from_linear_function(linear_func_right, frame, rel_x_ratio=0.8)
        else:
            steering_angle = self.filter_deg.prev_lpf

        # Low-pass filter를 적용한 steering angle 계산
        steering_angle = self.filter_deg.get_lpf(steering_angle)[0]

        """
        Explain Matrix
        """
        # vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        # explain_merge = np.hstack((explain2, vertical_line, explain1))
        # return steering_angle, explain_merge
        return steering_angle


    def _get_steering_angle2(self, frame):
        # 슬라이딩 윈도우를 통해 왼쪽/오른쪽 영역에서 검출된 차선의 점 추출
        choosen_left, choosen_right = get_points_with_sliding_window(frame)

        # 검출된 차선의 점에 맞는 2차함수 계산
        ret_left, linear_func_left = get_sliding_window_function(choosen_left)
        ret_right, linear_func_right = get_sliding_window_function(choosen_right)

        # 각 2차함수를 통해 steering angle 계산
        height, width = frame.shape[:2]
        scan_height = height // 4 * 3
        if ret_left or ret_right:
            steering_angle = get_steering_angle_from_linear_function2(linear_func_left, linear_func_right, frame, scan_height)
        else:
            steering_angle = self.filter_deg.prev_lpf

        # Low-pass filter를 적용한 steering angle 계산
        steering_angle = self.filter_deg.get_lpf(steering_angle)[0]

        """
        Explain Matrix
        """
        # vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        # explain_merge = np.hstack((explain2, vertical_line, explain1))
        # return steering_angle, explain_merge
        return steering_angle
