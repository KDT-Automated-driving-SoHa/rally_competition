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
from drive.keyboard_drive import KeyboardDrive

from module.image_processing import *

script_dir = os.path.dirname(__file__)

fps_dq = deque(maxlen=1000)

manager = RosManager()

sliding_window_dirve = SlidingWindowDrive()
stopline_drive = StoplineDrive()

while not rospy.is_shutdown():
    ret, frame = manager.get_image()

    if ret is False:
        continue

    fps_dq.append(datetime.now())
    fps = len(fps_dq) / ((fps_dq[-1]-fps_dq[0]).seconds + 1e-10)

    # 슬라이딩 윈도우 처리
    processed_sliding_window = preprocessing_sliding_window(frame)
    sliding_window_motor = sliding_window_dirve.get_motor_info(processed_sliding_window)

    # 정지선 멈춤 처리
    preprocessed_stopline = preprocessing_stopline(frame)
    stopline_motor = stopline_drive.get_motor_info(preprocessed_stopline, speed=50)

    main_motor = sliding_window_motor

    if type(stopline_motor) is list:
        # stopline_motor.angle = sliding_window_motor.angle
        main_motor = stopline_motor
        print "{: >15s} | fps: {:>1.0f} | {:s}".format("stopline_motor", fps, stopline_motor)
    else:
        print "{: >15s} | fps: {:>1.0f} | {:s}".format("sliding_window", fps, sliding_window_motor)
    
    manager.publish_motor(main_motor)

    """
    """
    # Rendering
    if socket.gethostname() == 'smbyeon-ubuntu':
        cv.imshow("original", frame)
        key = cv.waitKeyEx(1)
        if key == ord('q'):
            break
    """
    """