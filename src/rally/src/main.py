#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
#from dqn_drive import *
from parking import *
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
    
    cv2.imshow('frame', camera_image) 
    if not camera_image.size == (640*480*3):
        continue
       
    qrcodes = pyzbar.decode(camera_image)
    print(qrcodes)   
    for qtcode in qrcodes:
        qr_string = qtcode.data
        if qr_string in obj.keys():
            mode = qr_string
	    print("mode : "+str(mode))

    #QR code string
    print(mode)
    
    if mode == "":
        rm.set_motor(0, 0)
        continue

    #elif mode=="algorithm drive_avoid_obstacle":
	#mode="ULTRASONICQR"
    
    elif mode=="algorithm drive_turn_back":
	mode="BACKWARDQR"
    '''
    elif mode=="dqn dqn_drive_start":
	mode="DQNQR"
    elif mode=="dqn dqn_drive_end":
	mode="YOLOQR"
    '''
    elif mode=="algorithm drive_to_parking_lot_1":
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
        #obj[mode].set_data([10])
        if total_count<=6:
            if cnt<=2:
                if cnt==2:
                   obj[mode].set_data(False)
            elif cnt<=3:
                obj[mode].set_data(True)
            cnt+=1
            else:
                break
            total_count+=1
        else:
            rm.set_motor(0, 0)
            continue
        
    elif mode == "PARKQR":
	obj[]
	angle, speed = obj[mode].get_motor() 
        rm.set_motor(angle, speed)
        #obj[mode].set_data([10])
        if total_count<=6:
             if cnt<=2:
            if cnt==2:
               obj[mode].set_data(False)
             elif cnt<=3:
            obj[mode].set_data(True)
             cnt+=1
        else:
             break
        total_count+=1
        else:
            rm.set_motor(0, 0)
            continue
        
    #angle, speed = obj[mode].get_motor() 
    #rm.set_motor(angle, speed)
    
    
    
