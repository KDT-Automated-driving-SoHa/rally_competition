#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time
import rospy

from xycar_motor.msg import xycar_motor
from module.subscribers.camera import CameraSubscriber
from module.subscribers.lidar import LidarSubscriber


class RosManager():
    def __init__(self, name="driver", rate=10):
        self.motor_msg = xycar_motor()

        self.motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        self.camera_sub = CameraSubscriber()
        self.lidar_sub = LidarSubscriber()

        rospy.init_node("driver")
        self.rate = rospy.Rate(10)

    
    def get_image(self):
        return self.camera_sub.get()


    def publish_motor(self, motor_infos):
        if type(motor_infos) == list:
            for info in motor_infos:
                for _ in range(info.iterations):
                    self.motor_msg.angle = info.angle
                    self.motor_msg.speed = info.speed
                    # self.motor_msg.speed = 0
                    self.motor_pub.publish(self.motor_msg)
                    self.rate.sleep()
                    time.sleep(info.delay_sec)
        else:
            info = motor_infos
            for _ in range(info.iterations):
                self.motor_msg.angle = info.angle
                self.motor_msg.speed = info.speed
                # self.motor_msg.speed = 0
                self.motor_pub.publish(self.motor_msg)
                self.rate.sleep()
                time.sleep(info.delay_sec)

            

            
            
if __name__ == "__main__":
    controller = RosManager()
    rospy.spin()