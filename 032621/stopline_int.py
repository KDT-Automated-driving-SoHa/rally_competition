#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge

import rospy, rospkg
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image

class Stopline:
    def __init__(self, car, speed=40, thres_L=200):
        self.bridge = CvBridge()
        self.cam_image = np.empty(shape=[0])
        self.calibration = True

        self.Width, self.Height = 640, 480

        # self.draw_roi = True

        self.thres_L = thres_L
        self.speed_def = speed
        #self.y_s, self.y_e = 250, 278
        #self.xs = [250, 193, 387, 439]
        self.y_s, self.y_e = 230,310
        self.xs = [265,90, 375, 560]
        self.persp_mtx = {"margin": [0, 0, 30], 
                          "size": [1, 600, 640]} #480

        ## calibration
        if car == '171':
            # 171
            self.mtx = np.array([[369.721374, 0.0, 326.581101], [0.0, 371.087667, 236.510947], [0.0, 0.0, 1.0]])
            self.dist = np.array([-0.346953, 0.113405, -0.002992, -0.000073, 0.0])
        elif car == '174':
            # 174
            self.mtx = np.array([[345.346806, 0.000000, 323.241362], [0.000000, 347.001292, 239.609553],
                                 [0.000000, 0.000000, 1.000000]])
            self.dist = np.array([-0.302321, 0.071616, -0.002477, -0.000052, 0.000000])
        else:
            raise RuntimeError('No car number')
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (640, 480), 1, (640, 480))
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.cal_mtx,
                                                             (self.Width, self.Height), cv2.CV_32FC1)

    def camera_callback(self, data):
        self.cam_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def calibrate_image(self, image):
        if self.calibration:
            tf_image = cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR)
            x, y, w, h = self.cal_roi
            tf_image = tf_image[y:y + h, x:x + w]
            return cv2.resize(tf_image, (self.Width, self.Height))
        else:
            return image

    def processing(self, frame):
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, binary = cv2.threshold(L, self.thres_L, 255, cv2.THRESH_BINARY)

        # Perspective Transform 적용
        frame_height, frame_width = frame.shape[:2]
        margin = self.persp_mtx["margin"][1]

        roi_x1, roi_x2, roi_x3, roi_x4 = self.xs
        roi_y1, roi_y2 = self.y_s, self.y_e
        roi_x, roi_y = max(0, min(roi_x1, roi_x2)), max(0, min(roi_y1, roi_y2))
        roi_width, roi_height = min(max(roi_x3, roi_x4), frame_width) - roi_x, max(0, roi_y2 - roi_y1)

        tf_dst_size = self.persp_mtx["size"][1]
        tf_src_pts = np.array([[roi_x1, roi_y1], [roi_x2, roi_y2], [roi_x3, roi_y1], [roi_x4, roi_y2]],
                              dtype=np.float32)
        tf_dst_pts = np.array([[0, 0], [0, tf_dst_size], [tf_dst_size, 0], [tf_dst_size, tf_dst_size]],
                              dtype=np.float32)

        tf_matrix = cv2.getPerspectiveTransform(tf_src_pts, tf_dst_pts)
        tf_image = cv2.warpPerspective(binary, tf_matrix, (480, tf_dst_size), flags=cv2.INTER_LINEAR)

        # Drawing ROI
        roi = [(roi_x1, roi_y1), (roi_x2, roi_y2), (roi_x4, roi_y2), (roi_x3, roi_y1), (roi_x1, roi_y1)]
        # explain_image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        explain_image = frame
        for pt1, pt2 in zip(roi[:-1], roi[1:]):
            cv2.line(explain_image, pt1, pt2, (0, 0, 255))

        return tf_image, explain_image

    def detect_stopline(self, y_th=240, name=''):
        image2 = self.calibrate_image(self.cam_image)
        image, image3 = self.processing(image2)

        st = []
        num, color = 0, 0
        for x in range(image.shape[1]):
            if x == 0:
                color = image[y_th][x]
                num = 0
            else:
                if image[y_th][x] == color:
                    num += 1
                else:
                    st.append(num)
                    num = 0
                    color = image[y_th][x]
        st_new = [i for i in st if i != 0]
        explain_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        #explain_image = image3
        cv2.line(explain_image,(0,y_th),(explain_image.shape[1],y_th),(0,0,255))
        cv2.imshow(name,explain_image)
        return st_new

    def is_crossline(self, y_th=100, num_th=17):
        lis = self.detect_stopline(y_th,'cross')
        lis2 = self.detect_stopline(y_th+100,'cross2')
        if len(lis)!=0 and len(lis)>num_th:
            print("cross line up", len(lis))
            return True, 'up'
        if len(lis2)!=0 and len(lis2)>num_th:
            print("cross line down", len(lis2))
            return True, 'down'
        return False, 'no'

    def is_stopline(self, y_th=240, len_th=100):
        lis = self.detect_stopline(y_th,'stop')
        if len(lis)!=0 and max(lis)>=len_th:
            print("stop line",max(lis))
            return True
        return False


def drive(Angle, Speed, pub):
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

if __name__ == '__main__':
    
    speed = 50
    da = 7
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
    rospy.init_node('stopline')
    rospy.Subscriber("/usb_cam/image_raw", Image, stop.camera_callback)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        if stop.cam_image.size != (640*480*3):
            print("not yet")
            continue

        #is_stline = stop.is_stopline(y_th=240, len_th=300)
        is_crossline, where  = stop.is_crossline(y_th=thres_y, num_th=15)
        if is_crossline:
            if where =='up':
                for _ in range(15):
                    drive(da,20,pub)
                    time.sleep(0.1)
            #print("stopline")
            for _ in range(30): # 60
                #print()
                drive(da, 0, pub)
                time.sleep(0.1)
            for _ in range(20):
                #drive(da,0,pub)
                drive(da, speed*2//3, pub)  # 30
                time.sleep(0.1)
            #speed = 0
        else:
            drive(da, speed ,pub)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
