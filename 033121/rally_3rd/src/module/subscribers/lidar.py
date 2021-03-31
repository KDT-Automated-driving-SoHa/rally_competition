#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import LaserScan


class LidarSubscriber():
    def __init__(self, name="/scan"):
        # 원본 라이다 데이터
        self.raw_lidar = None

        # rospy subscriber
        self.sub_lidar = rospy.Subscriber(name, LaserScan, self._callback, queue_size=1)


    def get(self):
        return False, None
            

    def _callback(self, msg):
        pass