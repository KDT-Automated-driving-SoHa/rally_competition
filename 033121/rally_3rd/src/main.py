#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import socket

import cv2 as cv

from collections import deque
from datetime import datetime

from module.ros_manager import RosManager
from module.infos.handler_info import HandlerInfo
from handler.stop_line_handler import StopLineHandler
from handler.sliding_window_handler import SlidingWindowHandler
from handler.hough_line_handler import HoughLineHandler
from handler.ar_parking_handler import ARParkingHandler

fps_dq = deque(maxlen=100)

manager = RosManager()

stop_line_handler = StopLineHandler()           # 정지선 멈춤 처리
sliding_window_handler = SlidingWindowHandler() # 슬라이딩 윈도우 처리
hough_line_handler = HoughLineHandler()         # Hough Line 처리
ar_parking_handler = ARParkingHandler()         # AR 주차

# 책임 연쇄 초기화
# ar_parking_handler.set_next(sliding_window_handler)

ar_parking_handler.set_next(stop_line_handler)      # AR 주차 ---> 정지선 멈춤
stop_line_handler.set_next(sliding_window_handler)  # 정지선 멈춤 ---> 슬라이딩 윈도우 주행
# stop_line_handler.set_next(hough_line_handler)

handler = ar_parking_handler

while not rospy.is_shutdown():
    ret, frame = manager.get_image()

    if ret is False:
        continue

    # 프레임 수 검사
    fps_dq.append(datetime.now())
    fps = len(fps_dq) / ((fps_dq[-1]-fps_dq[0]).seconds + 1e-4)

    # Handler 정보 저장
    lRet, lidar_info = manager.get_lidar()
    uRet, ultrasonic_info = manager.get_ultrasonic()
    (aRet1, ar_info1), (aRet2, ar_info2) = manager.get_ar()

    handler_info = HandlerInfo()
    handler_info.image              = frame
    handler_info.lidar_info         = lidar_info if lRet else None
    handler_info.ultrasonic_info    = ultrasonic_info if uRet else None
    handler_info.ar_info1           = ar_info1 if aRet1 else None
    handler_info.ar_info2           = ar_info2 if aRet2 else None
    # handler_info.laps_count         = 3

    # 모터 정보 저장
    motor_info = handler.handle(handler_info)

    # motor_info.speed = 0
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