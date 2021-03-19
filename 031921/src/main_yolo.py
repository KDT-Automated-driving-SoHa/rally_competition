#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
#from dqn_drive import *
# from parking import *
#from ultrasonic_drive import *
from yolo_drive import *
#from go_backward import *

rm = ROS("team_name")

angle = 0
speed = 20 
mode = ""

obj = {
    #"DQNQR":DQN([256, 256], ""),
    #"PARKQR":PARK(),
    #"ULTRASONICQR":ULTRASONIC(),
    "YOLOQR":YOLO(),
}

mode = "YOLOQR"
rm.set_motor(angle, speed)

while rm.get_shutdown():
    camera_image = rm.get_camera_image_data()
    
    if not camera_image.size == (640*480*3):
        continue

    label = "bottle"
    obj["YOLOQR"].set_data([label, rm.bounding_boxes, camera_image])

    angle, speed = obj[mode].get_motor() 
    rm.set_motor(angle, speed)
    time.sleep(0.05)
    
