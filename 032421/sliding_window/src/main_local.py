#!/usr/bin/env python
#-*- coding: utf-8 -*-

import yaml, os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from image_processing import processing
from sliding_window import sliding_window
from scipy.optimize import least_squares
from utils.calculate import get_linear_function
from utils.filter import Filter
from utils.low_pass_filter import LowPassFilter
from utils.calculate import get_linear_function

f_cut = 100
freq = 100000

filter_left = Filter(f_cut=f_cut, freq=freq, num_data=3)
filter_right = Filter(f_cut=f_cut, freq=freq, num_data=3)
filter_deg = Filter(f_cut=f_cut, freq=freq, num_data=1)
filter_deg2 = LowPassFilter(0.75)

filter_coef_left = None
filter_coef_right = None

heading_deg = 0

def rint(point):
    return np.rint(point).astype(np.int32)


script_dir = os.path.dirname(__file__)
yaml_file_path = os.path.join(script_dir, "params", "calibration", "usb_cam.yaml")

with open(yaml_file_path) as f:
    data = yaml.load(f)

width, height = data["image_width"], data["image_height"]

cameraMatrix = data["camera_matrix"]["data"]
rows, cols = data["camera_matrix"]["rows"], data["camera_matrix"]["cols"]
cameraMatrix = np.array(cameraMatrix).reshape((rows, cols))

distCoeffs = data["distortion_coefficients"]["data"]
rows, cols = data["distortion_coefficients"]["rows"], data["distortion_coefficients"]["cols"]
distCoeffs = np.array(distCoeffs).reshape((rows, cols))

imageSize = (width, height)
newImgSize = (width, height)

newCameraMatrix, calibrated_ROI = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, newImgSize)

def calibrate_image(frame):
    tf_image = cv.undistort(frame, cameraMatrix, distCoeffs, None, newCameraMatrix)
    roi_x, roi_y, roi_width, roi_height = calibrated_ROI
    tf_image = tf_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width].copy()
    return cv.resize(tf_image, (frame.shape[1], frame.shape[0]))


filename1 = "/Volumes/UNTITLED/track_video/version1/rally.avi"
filename2 = "/Volumes/UNTITLED/track_video/version0/avi/track5.avi"

# 파일에서 영상 불러오기
cap1 = cv.VideoCapture(filename1)
cap2 = cv.VideoCapture(filename2)

if not cap1.isOpened():
    raise FileExistsError(filename1)
if not cap2.isOpened():
    raise FileExistsError(filename2)

# 영상 FPS, WIDTH, HEIGHT 불러오기
FPS = cap1.get(cv.CAP_PROP_FPS)
COUNT1 = cap1.get(cv.CAP_PROP_FRAME_COUNT)
COUNT2 = cap2.get(cv.CAP_PROP_FRAME_COUNT)

while True:
    # 프레임 한개 불러오기
    ret1, frame1 = cap1.read()
    # ret2, frame2 = cap2.read()

    if not ret1 or frame1 is None:
        cap1.set(cv.CAP_PROP_POS_FRAMES, 0)
        continue
    # if not ret2 or frame2 is None:
    #     cap2.set(cv.CAP_PROP_POS_FRAMES, 0)
    #     continue

    POS1 = cap1.get(cv.CAP_PROP_POS_FRAMES)
    # POS2 = cap2.get(cv.CAP_PROP_POS_FRAMES)

    def routine(frame, POS):
        global filter_coef_left, filter_coef_right, heading_deg
        frame = calibrate_image(frame)

        processed_frame, explain1 = processing(frame)

        processed_height, processed_width = processed_frame.shape[:2]
        processed_height2 = processed_height//3*2

        choosen_left, choosen_right, explain2 = sliding_window(processed_frame)

        func_coef_left = get_linear_function(choosen_left)
        func_coef_right = get_linear_function(choosen_right)

        if func_coef_left and func_coef_right:
            for coef in [func_coef_left, func_coef_right]:
                func = np.poly1d(coef)
                new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                new_xs = func(new_ys)

                for x, y in zip(new_xs, new_ys):
                    if 0 < y < processed_frame.shape[0] and 0 < x < processed_frame.shape[1]:
                        x, y = int(x), int(y)
                        cv.circle(explain2, (x, y), 3, (0, 255, 255), -1)

            for coef in [(func_coef_left + func_coef_right)/2]:
                func = np.poly1d(coef)
                new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                new_xs = func(new_ys)

                for x, y in zip(new_xs, new_ys):
                    if 0 < y < processed_frame.shape[0] and 0 < x < processed_frame.shape[1]:
                        x, y = int(x), int(y)
                        cv.circle(explain2, (x, y), 3, (0, 0, 255), -1)


            heading_src = (processed_width//2, processed_height)
            heading_dst = (func(processed_height2), processed_height2)
            heading_sub =  (heading_src[0] - heading_dst[0], heading_src[1] - heading_dst[1])

            heading_rad = np.arctan2(heading_sub[1], heading_sub[0])
            heading_deg = np.degrees(heading_rad)

            # heading_deg = filter_deg.LPF(np.array([heading_deg]))[0]
            heading_deg = filter_deg2.get_lpf(heading_deg)
            steering_deg = heading_deg - 90

            heading_rad = np.radians(heading_deg)

            gradient = np.tan(heading_rad)

            heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)


            heading_src = tuple(rint(heading_src))
            heading_dst = tuple(rint(heading_dst))

            cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
            cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
            cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)

        else:
            # heading_rad = np.radians(heading_deg)
            # gradient = np.tan(heading_rad) + 1e-10
            
            # heading_src = (processed_width//2, processed_height)
            # heading_dst = ((processed_height2 - heading_src[1])/gradient + heading_src[0], processed_height2)

            # heading_src = tuple(rint(heading_src))
            # heading_dst = tuple(rint(heading_dst))

            # cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
            # cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
            # cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
        
            if func_coef_left:
                func = np.poly1d(func_coef_left)
                new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                new_xs = func(new_ys)

                left_src = (func(processed_height), processed_height)
                left_dst = (func(processed_height2), processed_height2)
                left_sub =  (left_src[0] - left_dst[0], left_src[1] - left_dst[1])

                left_rad = np.arctan2(left_sub[1], left_sub[0])
                heading_deg = np.degrees(left_rad)

                heading_deg = filter_deg2.get_lpf(heading_deg)
                steering_deg = heading_deg - 90

                heading_rad = np.radians(heading_deg)

                gradient = np.tan(heading_rad) + 1e-10
                heading_src = (processed_width//2, processed_height)
                heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)

                heading_src = tuple(rint(heading_src))
                heading_dst = tuple(rint(heading_dst))

                cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
                cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
                cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)

            elif func_coef_right:
                func = np.poly1d(func_coef_right)
                new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                new_xs = func(new_ys)

                right_src = (func(processed_height), processed_height)
                right_dst = (func(processed_height2), processed_height2)
                right_sub =  (right_src[0] - right_dst[0], right_src[1] - right_dst[1])

                right_rad = np.arctan2(right_sub[1], right_sub[0])
                heading_deg = np.degrees(right_rad)

                heading_deg = filter_deg2.get_lpf(heading_deg)
                steering_deg = heading_deg - 90

                heading_rad = np.radians(heading_deg)

                gradient = np.tan(heading_rad) + 1e-10

                heading_src = (processed_width//2, processed_height)
                heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)

                heading_src = tuple(rint(heading_src))
                heading_dst = tuple(rint(heading_dst))

                cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
                cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
                cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
            else:
                new_ys = []
                new_xs = []


            for x, y in zip(new_xs, new_ys):
                if 0 < y < processed_frame.shape[0] and 0 < x < processed_frame.shape[1]:
                    x, y = int(x), int(y)
                    cv.circle(explain2, (x, y), 3, (255, 255, 0), -1)

        print(heading_deg)
        # Merge Explain
        vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
        explain_merge = np.hstack((explain2, vertical_line, explain1))
        cv.putText(explain_merge, "POS: {}".format(int(POS)), (0, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return explain_merge

    def merge_all(explain1_merge, explain2_merge):
        explain_merge_all = np.vstack((explain1_merge, explain2_merge))
        width = int(explain_merge_all.shape[1] * 0.8)
        height = int(explain_merge_all.shape[0] * 0.8)
        return cv.resize(explain_merge_all, (width, height), interpolation=cv.INTER_LINEAR)

    explain1_merge = routine(frame1, POS1)
    cv.imshow("merge all", explain1_merge)
    # explain2_merge = routine(frame2, POS2)
    # explain_merge_all = merge_all(explain1_merge, explain2_merge)
    # cv.imshow("merge all", explain_merge_all)

    # Rendering
    delay = int(1000//FPS)
    key = cv.waitKeyEx(delay)

    # cap2.set(cv.CAP_PROP_POS_FRAMES, POS2+5)

    if key == 32:
        while key != cv.waitKey(delay):
            explain1_merge = routine(frame1, POS1)
            cv.imshow("merge all", explain1_merge)
            # explain2_merge = routine(frame2, POS2)
            # explain_merge_all = merge_all(explain1_merge, explain2_merge)
            # cv.imshow("merge all", explain_merge_all)
    elif key == 65361:  # left arrow key
        cap1.set(cv.CAP_PROP_POS_FRAMES, max(0, min((POS1-30 + COUNT1) % COUNT1, COUNT1-1)));
        # cap2.set(cv.CAP_PROP_POS_FRAMES, max(0, min((POS2-30 + COUNT2) % COUNT2, COUNT2-1)));
    elif key == 65363:  # right arrow key
        cap1.set(cv.CAP_PROP_POS_FRAMES, max(0, min((POS1+30) % COUNT1, COUNT1-1)));
        # cap2.set(cv.CAP_PROP_POS_FRAMES, max(0, min((POS2+30) % COUNT2, COUNT2-1)));
    elif key == ord('q'):
        break

cv.destroyAllWindows()
cap.release()
