#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
from stopline_int import *
from ar_int import *

rm = ROS("team_name")
da = 7
angle = 0
speed = 50
mode = ""
count =3
thres_50 = 90

if speed == 30:
    #thres_y = 200
    thres_y = thres_50 + 200
elif speed == 40:
    #thres_y = 100
    thres_y = thres_50 + 80
elif speed == 50:
    thres_y = thres_50
stop = Stopline('174', speed=speed, thres_L=200)
ar = Parking('int')

def drive(Angle, Speed, pub):
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

while rm.get_shutdown():
    
    #print(rm.get_ultrasonic_data())
    #camera_image = rm.get_camera_image_data()
    
    #cv2.imshow('frame', camera_image) 
    if not stop.cam_image.size == (640*480*3):
        print("not yet")
        continue

    if count>=3:
        ar_data = rm.get_ar_tags_datas()
        ar.callback(ar_data)
        ult_data = rm.get_ultrasonic_data()
        ar.ultrasonic_callback(ult_data)
        ar.park()
        if ar.stage == 1:
            for _ in range(30):
                rm.set_motor(0, 20)
                time.sleep(0.1)
            for _ in range(20):
                rm.set_motor(-50, 20)
                time.sleep(0.1)
            ar.stage = 2
            print("===Stage2===")

        else:
            print("count", count)
            rm.set_motor(ar.angle,ar.speed)
            time.sleep(0.1)

    else:
        stop.camera_callback(rm.get_camera_image_data())
        is_crossline, where = stop.is_crossline(y_th=thres_y, num_th=15)
        if is_crossline:
            if where =='up':
                for _ in range(15):
                    rm.set_motor(da,20)
                    time.sleep(0.1)
            #print("stopline")
            for _ in range(30): # 60
                #print()
                rm.set_motor(da, 0)
                time.sleep(0.1)
            for _ in range(20):
                #drive(da,0,pub)
                rm.set_motor(da, speed*2//3)  # 30
                time.sleep(0.1)
            count+=1
            #speed = 0

        rm.set_motor(da, speed)
        time.sleep(0.1)



		
    
    
    
