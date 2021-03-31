#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

from sensor_msgs.msg import LaserScan

# rospy.init_node("lidar_sub")
class LidarSubscriber():
    def __init__(self, name="/scan"):
        # data
        self.ranges = []
        self.number = 0
        self.del_alpha = None
        
        # rospy subscriber
        
        self.sub_lidar = rospy.Subscriber(name, LaserScan, self._callback, queue_size=1)

    def get_param(self):
        result = False
        if self.number != 0:
            result = True
        return result, [self.del_alpha, self.number]
        
    def get(self):
        return True, self.ranges

    def _callback(self, msg):
        self.ranges = msg.ranges
        self.number = len(self.ranges)
        self.del_alpha = msg.angle_increment