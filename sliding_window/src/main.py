#!/usr/bin/env python
#-*- coding: utf-8 -*-

## version upgrade v1.1
## incremnet lidar data by chy
## original from sm

import os
import rospy
import socket

import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

from collections import deque
from datetime import datetime

from ros_manager import RosManager
from image_processing import processing
from sliding_window import sliding_window, get_steering_angle_from_linear_function

from utils.linear_functions import *
from utils.low_pass_filter import LowPassFilter
from utils.moving_average_filter import MovingAverageFilter
from utils.discreate_filter import DiscreateFilter
from utils.speed import decrease_function, decrease_function2
from utils.lidarprocessing import *
script_dir = os.path.dirname(__file__)

# filter_deg = MovingAverageFilter(5)
# filter_deg2 = LowPassFilter(0.8)
filter_deg2 = DiscreateFilter(f_cut=4000, freq=10000)

motor_angle = 0
max_angle = 0

manager = RosManager()

fps_dq = deque(maxlen=100)

while not rospy.is_shutdown():
    ret, image = manager.get_image()
    
    if ret is False:
        continue
    ret, cp_param = manager.get_lidar_param()

    frame = image
    cv.imshow('original', frame)
    def routine(frame):
        global rel_x_ratio
        processed_frame, explain1 = processing(frame)

        choosen_left, choosen_right, explain2 = sliding_window(processed_frame)

        ret_left, linear_func_left = get_sliding_window_function(choosen_left)
        ret_right, linear_func_right = get_sliding_window_function(choosen_right)

        if ret_left and ret_right:
            steering_angle, explain2 = get_steering_angle_from_linear_function((linear_func_left + linear_func_right)/2, explain2)
            # left = linear_func_left(480)
            # right = linear_func_right(480)
            # print(left, right, right - left)
            
            ## distance ~= 435.662 PIXEL or 0.845m 
            ## 515.58 PIXEL / m
            ## Offset from lidar to ROI : 0.400 m
            
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_left:
            steering_angle, explain2 = get_steering_angle_from_linear_function(linear_func_left, explain2, rel_x_ratio=1.2)
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_right:
            steering_angle, explain2 = get_steering_angle_from_linear_function(linear_func_right, explain2, rel_x_ratio=0.8)
            # steering_angle = filter_deg.get_moving_average(steering_angle)
        else:
            steering_angle = filter_deg2.prev_lpf
            # steering_angle = filter_deg.prev_avg
        steering_angle = filter_deg2.get_lpf(steering_angle)[0]

        # Merge Explain
        vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        explain_merge = np.hstack((explain2, vertical_line, explain1))

        return steering_angle, explain_merge

    motor_angle, explain_frame = routine(frame)

    abs_motor_angle = max(0, get_linear_steering_angle2(abs(motor_angle)))

    angle = np.clip(np.sign(motor_angle) * abs_motor_angle, -50, 50)
    speed = get_linear_speed2(angle, 50)

    manager.publish_motor(angle, -speed)

    fps_dq.append(datetime.now())

    if (fps_dq[-1] - fps_dq[0]).seconds != 0:
    #     print("angle: {: >7.3f} | speed: {: >6.3f} | fps: {: >2d} | Ratio: {: >3.3f}".format(
    #         angle, speed, len(fps_dq) / (fps_dq[-1] - fps_dq[0]).seconds, rel_x_ratio))
        print("angle: {: >7.3f} | motor angle: {:7.3f} | speed: {: >6.3f} | fps: {: >2d}".format(
            angle, motor_angle, speed, len(fps_dq) / (fps_dq[-1] - fps_dq[0]).seconds))

    # fps_dq.append(datetime.now())
    # if (fps_dq[-1] - fps_dq[0]).seconds != 0:
    #     print(len(fps_dq) / (fps_dq[-1] - fps_dq[0]).seconds)

    """
    """
    # Rendering
    if socket.gethostname() == 'hcy-VirtualBox':
        cv.imshow("explain", explain_frame)
        key = cv.waitKeyEx(1)
        if key == ord('q'):
            break
    """
    """