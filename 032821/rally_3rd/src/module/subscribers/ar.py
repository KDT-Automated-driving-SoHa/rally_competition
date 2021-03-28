#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers


class ARInfo():
    def __init__(self, position, orientation):
        # AR의 위치
        self.dx = position.x * 100
        self.dy = position.y * 100
        self.dz = position.z * 100

        # AR의 방향(quaternion)
        self.ax = orientation.x
        self.ay = orientation.y
        self.az = orientation.z
        self.aw = orientation.w

        # AR의 방향(rpy)
        self.roll,
        self.pitch,
        self.yaw = euler_from_quaternion((self.ax, self.ay, self.ax, self.aw))

        # AR의 방향(rpy, degree)
        self.roll_deg = np.degrees(self.roll)
        self.pitch_deg = np.degrees(self.pitch)
        self.yaw_deg = np.degrees(self.yaw)


class ARSubscriber():
    def __init__(self, name="/ar_pose_marker"):
        # 원본 라이다 데이터
        self.raw_sign = None        # 주차 신호 정보
        self.raw_parking = None     # 주차 위치 정보

        # rospy subscriber
        rospy.Subscriber(name, AlvarMarkers, self._callback, queue_size=1)


    def get(self):
        return False, None
            

    def _callback(self, msg):
        self.raw_sign = None
        self.raw_parking = None

        for marker in markers:
            position = marker.pose.pose.position
            orientation = marker.pose.pose.orientation

            if marker.id == 1:
                self.raw_sign = ARInfo(position, orientation)
            elif marker.id = 2:
                self.raw_parking = ARInfo(position, orientation)


    def get_parking_sign(self):
        if self.raw_sign is None:
            return False, None
        return True, self.raw_sign


    def get_parking_line(self):
        if self.raw_parking is None:
            return False, None
        return True, self.raw_parking
