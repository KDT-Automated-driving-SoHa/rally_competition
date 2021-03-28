#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy

import numpy as np

from ros_manager import RosManager

angle = 0
speed = 0

manager = RosManager()

while not rospy.is_shutdown():
    key_angle = 0
    key_speed = 0

    ret, key = manager.keyboard_input()

    if ret:
        if key == "w":
            key_speed += 5
        elif key == "s":
            key_speed -= 5
        elif key == "a":
            key_angle -= 5
        elif key == "d":
            key_angle += 5
        elif key == "j":
            angle = 0
        elif key == "k":
            speed = 0

    angle += key_angle
    speed += key_speed

    speed = np.clip(speed, -50, 50)
    angle = np.clip(angle, -50, 50)

    manager.publish_motor(angle, speed)
    print("angle: {: >5d}, speed: {:>5d}".format(angle, speed))
