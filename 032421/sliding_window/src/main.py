#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import rospy

import numpy as np
import cv2 as cv

from collections import deque
from datetime import datetime

from ros_manager import RosManager
from image_processing import processing
from sliding_window import sliding_window, get_steering_angle_from_linear_function

from utils.calculate import get_linear_function
from utils.low_pass_filter import LowPassFilter
from utils.moving_average_filter import MovingAverageFilter
from utils.filter import Filter
from utils.speed import decrease_function, decrease_function2

script_dir = os.path.dirname(__file__)

# filter_left = Filter(f_cut=3000, freq=10000, num_data=3)
# filter_right = Filter(f_cut=3000, freq=10000, num_data=3)
filter_deg = MovingAverageFilter(5)
filter_deg2 = LowPassFilter(0.5)

motor_angle = 0
prev_speed = 50

manager = RosManager()

fps_dq = deque(maxlen=100)

while not rospy.is_shutdown():
    ret, image = manager.get_image()
    
    if ret is False:
        continue

    frame = image

    def routine(frame, POS=None):
        global rel_x_ratio
        processed_frame, explain1 = processing(frame)

        choosen_left, choosen_right, explain2 = sliding_window(processed_frame)

        ret_left, linear_func_left = get_linear_function(choosen_left)
        ret_right, linear_func_right = get_linear_function(choosen_right)

        log_x_ratio = prev_speed / 50 * np.e
        rel_x_ratio = np.log(min(log_x_ratio*1.05, np.e))

        if ret_left and ret_right:
            steering_angle, explain2 = get_steering_angle_from_linear_function((linear_func_left + linear_func_right)/2, explain2)
            steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_left:
            steering_angle, explain2 = get_steering_angle_from_linear_function(linear_func_left, explain2, rel_x_ratio=rel_x_ratio)
            steering_angle = filter_deg.get_moving_average(steering_angle)
        elif ret_right:
            steering_angle, explain2 = get_steering_angle_from_linear_function(linear_func_right, explain2, rel_x_ratio=rel_x_ratio)
            steering_angle = filter_deg.get_moving_average(steering_angle)
        else:
            steering_angle = filter_deg.prev_avg
        steering_angle = filter_deg2.get_lpf(steering_angle)

        # Merge Explain
        vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        explain_merge = np.hstack((explain2, vertical_line, explain1))

        return steering_angle, explain_merge

    motor_angle, explain_frame = routine(frame)

    angle = np.clip(motor_angle * 1.3, -50, 50)
    speed = decrease_function2(angle, 40)

    manager.publish_motor(angle, speed)

    prev_speed = speed
    
    fps_dq.append(datetime.now())

    if (fps_dq[-1] - fps_dq[0]).seconds != 0:
        print("angle: {: >7.3f} | speed: {: >6.3f} | fps: {: >2d} | Ratio: {: >3.3f}".format(
            angle, speed, len(fps_dq) / (fps_dq[-1] - fps_dq[0]).seconds, rel_x_ratio))

    # fps_dq.append(datetime.now())
    # if (fps_dq[-1] - fps_dq[0]).seconds != 0:
    #     print(len(fps_dq) / (fps_dq[-1] - fps_dq[0]).seconds)

    """
    """
    # Rendering
    cv.imshow("explain", explain_frame)
    key = cv.waitKeyEx(1)
    if key == ord('q'):
        break
    """
    """