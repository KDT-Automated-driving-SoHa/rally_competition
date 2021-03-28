#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

from std_msgs.msg import Int32MultiArray


class UltrasonicInfo():
    def __init__(self, data):
        # Xycar 방향에 따른 초음파 거리값
        self.left           = data[0]
        self.front_left     = data[1]
        self.front_middle   = data[2]
        self.front_right    = data[3]
        self.right          = data[4]
        self.back_right     = data[5]
        self.back_middle    = data[6]
        self.back_left      = data[7]


class UltrasonicSubscriber():
    def __init__(self, name="/xycar_ultrasonic"):
        # 원본 초음파 데이터
        self.raw_data = None

        # rospy subscriber
        rospy.Subscriber(name, Int32MultiArray, self._callback, queue_size=1)


    def get(self):
        if self.raw_data is None:
            return False, None
        return True, self.raw_data
            

    def _callback(self, msg):
        self.raw_data(msg.data)
