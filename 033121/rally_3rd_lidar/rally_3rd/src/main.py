#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import rospy
import socket

import numpy as np
import cv2 as cv

from collections import deque
from datetime import datetime

from ros_manager import RosManager
from drive.stopline_drive import StoplineDrive
from drive.sliding_window_drive import SlidingWindowDrive
from utils.discreate_filter import DiscreateFilter

from module.image_processing import *
from utils.lidarprocessing import TrnasformCP

script_dir = os.path.dirname(__file__)

fps_dq = deque(maxlen=1000)

manager = RosManager()

sliding_window_dirve = SlidingWindowDrive()
stopline_drive = StoplineDrive()

## implemnet lidar
ret, cp_param = manager.get_lidar_param()
while not ret:
    ret, cp_param = manager.get_lidar_param()
    continue
# 1.3
tfcp = TrnasformCP(cp_param, K=180 / 3.14 * 1.0)
tfcp.set_azimuth()

filter_obs = DiscreateFilter(f_cut=1000, freq=10000)
filter_obs_vel = DiscreateFilter(f_cut=1000, freq=10000)


while not rospy.is_shutdown():
    ret, frame = manager.get_image()

    if ret is False:
        continue

    ## lidar implement
    _, cp_law = manager.get_lidar()
    cloud_r, cloud_al = tfcp.rm_spherical(cp_law, 180)
    cartesian = tfcp.get_cartesian(cloud_r, cloud_al)
    camera_mks = tfcp.transform_to_ROI(cartesian)
    # print(camera_mks) --
    num_data, camera_mks_rm = tfcp.rm_cartesian(camera_mks)

    fps_dq.append(datetime.now())
    fps = len(fps_dq) / ((fps_dq[-1]-fps_dq[0]).seconds + 1e-10)

    # 슬라이딩 윈도우 처리
    processed_sliding_window = preprocessing_sliding_window(frame)
    sliding_window_motor, linear_func_left, linear_func_right = sliding_window_dirve.get_motor_info(processed_sliding_window)

    speed = 35
    angle_obs, explain2, speed = tfcp.get_segment(processed_sliding_window, camera_mks_rm, linear_func_left, linear_func_right, speed)
    # angle_obs = filter_obs.get_lpf(angle_obs)[0]
    sliding_window_motor.speed = filter_obs_vel.get_lpf(speed)[0]

    sliding_window_motor.angle = sliding_window_motor.angle + int(angle_obs) + 8

    main_motor = sliding_window_motor

    # if type(stopline_motor) is list:
    #     # stopline_motor.angle = sliding_window_motor.angle
    #     main_motor = stopline_motor
    #     print "{: >15s} | fps: {:>1.0f} | {:s}".format("stopline_motor", fps, stopline_motor)
    # else:
    #     print "{: >15s} | fps: {:>1.0f} | {:s}".format("sliding_window", fps, sliding_window_motor)
    
    manager.publish_motor(main_motor)
    print(main_motor)

    """
    """
    # Rendering
    if socket.gethostname() == 'hcy-VirtualBox':
        
        cv.imshow("original", explain2)
        key = cv.waitKeyEx(1)
        if key == ord('q'):
            break
    """
    """