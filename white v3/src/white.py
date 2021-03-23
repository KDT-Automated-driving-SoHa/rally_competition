#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image

pub = None

bridge = CvBridge()
cam_image = np.empty(shape=[0])

calibration = True
is_stop = False
Width = 640
Height = 480

#calibration
#mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
mtx = np.array([[369.721374, 0.0, 326.581101], [0.0, 371.087667, 236.510947], [0.0, 0.0, 1.0]])

#dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
dist = np.array([-0.346953, 0.113405, -0.002992, -0.000073, 0.0])
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 1, (640, 480))

angle, speed = 0, 0

speed_def = 40
params = {30:[1000,250,325,10],40:[800,200,297,15]}#white threshold , x_offset , y_start, y offset (x_start = (width-x_off)/2)
def rand_color():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

def calibrate_image(image):

    tf_image = cv2.undistort(image, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y + h, x:x + w]

    return cv2.resize(tf_image, (640, 480))

def camera_callback(data):
    global cam_image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    if calibration:
        cam_image = calibrate_image(image)
    else:
        cam_image = image

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

def white(image):
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    H, L, S = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, H2 = cv2.threshold(H, 148, 255, cv2.THRESH_BINARY)
    _, S2 = cv2.threshold(S, 148, 255, cv2.THRESH_BINARY)
    _, L2 = cv2.threshold(L, 148, 255, cv2.THRESH_BINARY)

    #cv2.imshow('H',H2)
    #cv2.imshow('L',L2)
    #cv2.imshow('S',S2)
    #cv2.imshow('L', L)
    return L2

def detect_stopline(cal_image, low_threshold_value):
    global speed, angle
    global is_stop, speed_def, params

    white_th, x_off, y_start, y_off= params[speed_def]

    stopline_roi, x_s, y_s = set_roi(cal_image, x_off, y_start, y_off)
    image = image_processing(stopline_roi, low_threshold_value)
    count = cv2.countNonZero(image)
    #is_stop = False
    if count > white_th:
        print("stopline")
        is_stop = True
        
    print(count, is_stop)

    #draw roi
    image_w = white(cal_image)
    
    color = rand_color()
    image2 = cv2.line(image_w, (x_s, y_s), (x_s, y_s + y_off), color, 2)
    image2 = cv2.line(image2, (x_s, y_s), (x_s+x_off, y_s), color, 2)
    image2 = cv2.line(image2, (x_s, y_s+y_off), (x_s + x_off, y_s+y_off), color, 2)
    image = cv2.line(image2, (x_s + x_off, y_s+y_off), (x_s + x_off, y_s+y_off), color, 2)
    cv2.putText(image,str(count),(320+x_off//2,y_s+5),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255))
    return image

def set_roi(frame, x_len, start_y, offset_y):
    _, width, _ = frame.shape
    start_x = int(width/2 - (x_len/2))
    end_x = int(width - start_x)

    return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

def image_processing(image, low_threshold_value):
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, L = cv2.threshold(L, 150, 255, cv2.THRESH_BINARY)
    
    return L

def start():
    global pub
    global cam_image
    global Width, Height
    global angle, speed
    global is_stop
    rospy.init_node('white')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, camera_callback)
    print("start")
    rospy.sleep(1)

    while not cam_image.size == (640*480*3):
        #print("not yet", cam_image.size)
        time.sleep(0.5)
        continue

    while not rospy.is_shutdown():
        H,L,S = white(cam_image)
        #cv2.imshow('H',H)
        #cv2.imshow('S',S)
        cv2.imshow('L',L)
        '''
        img_white_roi = detect_stopline(cam_image,148)
        #cv2.imshow('binary image with roi',img_white_roi)
        if is_stop:
            print("stopline")
            drive(1,0)
        else:
            drive(1,0)
        '''
if __name__ == '__main__':
    start()
