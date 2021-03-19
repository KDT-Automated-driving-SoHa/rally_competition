#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
#from dqn_drive import *
from parking2 import *
#from ultrasonic_drive import *
#from yolo_drive import *
from go_backward import *

rm = ROS("team_name")

angle = 0
speed = 20
mode = ""

cnt=0
total_count=0

obj = {
    #"DQNQR":DQN([256, 256], ""),
    "PARKQR":PARK(),
    #"ULTRASONICQR":ULTRASONIC(),
    #"YOLOQR":YOLO(),
    "BACKWARDQR":BACKWARD()
}


while rm.get_shutdown():
    camera_image = rm.get_camera_image_data()
    if not camera_image.size == (640*480*3):
        continue
    
    #cv2.imshow('frame', camera_image)   
    qrcodes = pyzbar.decode(camera_image)
    for qtcode in qrcodes:
        qr_string = qtcode.data
        mode = qr_string
	print("mode : "+str(mode))

    #QR code string
    
    if mode == "":
        rm.set_motor(0, 0)
        continue

    #elif mode=="algorithm drive_avoid_obstacle":
	#mode="ULTRASONICQR"
    
    if mode=="2 algorithm drive_turn_back":
	mode="BACKWARDQR"
    """
    elif mode=="dqn dqn_drive_start":
	mode="DQNQR"
    elif mode=="dqn dqn_drive_end":
	mode="YOLOQR"
    """
    if mode=="7 algorithm drive_to_parking_lot_1":
	mode="PARKQR"
    
    #obj[mode].set_motor(angle, speed)
    
    '''
    if mode == "DQNQR":
        obj[mode].set_data([])
    
    elif mode == "YOLOQR":
        obj[mode].set_data([])
    elif mode == "ULTRASONICQR":
        obj[mode].set_data([])
    '''
    if mode == "BACKWARDQR":
        continue
        
    elif mode == "PARKQR":
        obj_ar = obj[mode]
        ar_data = rm.get_ar_tags_datas()
        obj_ar.set_data(ar_data)
        if obj_ar.is_back:
            for _ in range(2):
                rm.set_motor(0,0)
                time.sleep(0.1)
            for _ in range(21):
                rm.set_motor(obj_ar.get_motor())
                time.sleep(0.1)
            for _ in range(7):
                rm.set_motor(-obj_ar.angle,obj_ar.speed)
                time.sleep(0.1)
        else:
            rm.set_motor(obj_ar.get_motor())
            time.sleep(0.1)
        
    #angle, speed = obj[mode].get_motor() 
    #rm.set_motor(angle, speed)
    
    
    
