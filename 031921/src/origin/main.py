#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
from dqn_drive import *
from parking import *
from ultrasonic_drive import *
from yolo_drive import *
from go_backward import *

rm = ROS("team_name")

angle = 0
speed = 20

obj = {
    "DQNQR":DQN([256, 256], ""),
    "PARKQR":PARK(),
    "ULTRASONICQR":ULTRASONIC(),
    "YOLOQR":YOLO(),
    "BACKWARDQR":BACKWARD()
}

while rm.get_shutdown():
    camera_image = rm.get_camera_image_data()
    if not camera_image.size == (640*480*3):
        continue

    qrcodes = pyzbar.decode(camera_image)
    for qtcode in qrcodes:
        qr_string = qtcode.data
        if qr_string in obj.keys():
            mode = qr_string

    if mode == "":
        rm.set_motor(0, 0)
        continue

    obj[mode].set_motor(angle, speed)

    if mode == "DQNQR":
        obj[mode].set_data([])
    elif mode == "PARKQR":
        obj[mode].set_data([])
    elif mode == "YOLOQR":
        obj[mode].set_data([])
    elif mode == "ULTRASONICQR":
        obj[mode].set_data([])
    elif mode == "BACKWARDQR":
        obj[mode].set_data([])
    else:
        rm.set_motor(0, 0)
        continue

    angle, speed = obj[mode].get_motor() 
    rm.set_motor(angle, speed)
    
    
    
