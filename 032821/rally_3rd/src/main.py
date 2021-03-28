#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import socket

import cv2 as cv

from collections import deque
from datetime import datetime

from module.ros_manager import RosManager
from handler.stop_line_handler import StopLineHandler
from handler.sliding_window_handler import SlidingWindowHandler

fps_dq = deque(maxlen=100)

manager = RosManager()

stop_line_handler = StopLineHandler()           # 정지선 멈춤 처리
sliding_window_handler = SlidingWindowHandler() # 슬라이딩 윈도우 처리
stop_line_handler.set_next(sliding_window_handler)

handler = stop_line_handler

while not rospy.is_shutdown():
    ret, frame = manager.get_image()

    if ret is False:
        continue

    fps_dq.append(datetime.now())
    fps = len(fps_dq) / ((fps_dq[-1]-fps_dq[0]).seconds + 1e-4)

    motor_info = handler.handle(frame)
    manager.publish_motor(motor_info)

    print "fps: {:>1.0f} | {:s}".format(fps, motor_info)

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