#!/usr/bin/env python

import cv2, math
import numpy as np
from tf.transformations import euler_from_quaternion

class PARK:
    angle = 0
    speed = 0
    roll, pitch, yaw = 0, 0, 0
    is_data = False
    
    arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}

    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = motor
    
    def drive(Angle,Speed):
	set_motor(Angle, Speed)

    def back(ang,cy):
        for i in range(10):
            drive(0,0)
            time.sleep(0.1)

        for i in range(cy):
            drive(ang,-20)
            time.sleep(0.1)

        for _ in range(cy//3):
            drive(-ang,-20)
            time.sleep(0.1)

    def callback(Data):
   	global arData, is_data

    	if len(Data) == 0:
            is_data = False
            arData["DX"] = 0.0
            arData["DY"] = 0.0
            arData["DZ"] = 0.0

            arData["AX"] = 0.0
            arData["AY"] = 0.0
            arData["AZ"] = 0.0
            arData["AW"] = 0.0
        else:

            is_data = True
            
            arData["DX"] = Data["pos_x"] * 100
            arData["DY"] = Data["pos_y"] * 100
            arData["DZ"] = Data["pos_z"] * 100

            arData["AX"] = Data["ori_x"]
            arData["AY"] = Data["ori_y"]
            arData["AZ"] = Data["ori_z"]
            arData["AW"] = Data["ori_w"]


    def set_data(self, data):
	#check len of data
	callback(data)

        (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

	roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

	theta = math.atan(arData["DX"]/arData["DZ"])*180/math.pi if arData["DZ"] !=0 else 0
        w_t = 1
        w_p = 0.5

        angle = int(theta * w_t)

	if is_data:
            if int(arData["DZ"]) == 0:
    	        speed,angle = 0,0
            elif int(arData["DZ"]) > 0 and int(arData["DZ"]) <= 50:
            	if (pitch) >= 10:
                    back(-20,30)

            	elif pitch <=-10:
                    back(20,30)

            	else:
                    speed = 0

            elif int(arData["DZ"]) > 50 and int(arData["DZ"]) <=90:
            	speed = 20
            else:
            	speed = 20
        else:
            speed,angle =0,0

    def get_motor(self):
        return self.angle, self.motor
