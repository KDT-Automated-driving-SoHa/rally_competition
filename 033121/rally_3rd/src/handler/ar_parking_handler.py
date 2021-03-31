#!/usr/bin/env python
# -*- coding: utf-8 -*-
from handler import AbstractHandler
from module.infos.motor_info import MotorInfo
from module.ar_parking.ar_parking import ParkingBehavior

class ARParkingHandler(AbstractHandler):

    def __init__(self):
        self.behavior = ParkingBehavior()


    def handle(self, handler_info):
        laps_count = handler_info.laps_count
        ultrasonic_info = handler_info.ultrasonic_info
        ar_info1 = handler_info.ar_info1
        ar_info2 = handler_info.ar_info2

        if laps_count >= 3 and (ar_info1 or ar_info2):
            return self.behavior.get_motor_info(ultrasonic_info, ar_info1, ar_info2)
        else:
            return self._next_handler.handle(handler_info)
