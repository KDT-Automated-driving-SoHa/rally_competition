#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
# home
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import filter
import discreatefilter

import matplotlib.pyplot as plt

freq = 20000
f_cut = 1000
# title = "steer_" + str(f_cut)
# lpf = discreatefilter.filter(f_cut, freq, num_data=2, title=title)
f_cut_steer = 3000
title_steer = "steer_" + str(f_cut_steer)
filter_steer = discreatefilter.filter(f_cut_steer, freq, num_data=1, title=title_steer)
mm5 = filter.MovingAverage(5)
mm10 = filter.MovingAverage(10)


pub = None

bridge = CvBridge()
image = np.empty(shape=[0])

Width = 640
Height = 480

Offset = 320
Gap = 40

x_center = Width / 2
y_center = Offset + Gap / 2

#handle_pic = rospkg.RosPack().get_path('hough_drive') + '/src/steer_arrow.png'

#arrow_pic = cv2.imread(handle_pic, cv2.IMREAD_COLOR)

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# home
# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width, params

    low_slope_threshold = 0
    high_slope_threshold = params[4]

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 150):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 150):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = float(x_sum) / float(size * 2)
    y_avg = float(y_sum) / float(size * 2)

    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)
    x1, x2 = 0, 0
    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

    return x1, x2, int(pos), m, b

Offset_vanishing_missing_x = 0 
def get_vanishing_points(m1, b1, m2, b2):
    global Width, Offset_vanishing_missing_x
    
    if m1 == 0 and m2 == 0:
        x = Width / 2
        y = 0
        return x, y
    
    if m1 == 0:
        y = 0
        x = -b2 / m2 + Offset_vanishing_missing_x
        print("m1 missing", x, y)

    elif m2 == 0:
        y = 0
        x = - b1 / m1
        print("m2 missing", x, y)

    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1*x + b1

    return x, y
    

# show image and return lpos, rpos
def process_image(frame, delta):
    global Width
    global Offset, Gap
    global cann
    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = cann[0] #60
    high_threshold = cann[1] #100
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    
    #morphological operation
    kernel=np.ones((2,2),np.uint8)
    result=cv2.dilate(edge_img, kernel,iterations=1)
    
    # cv2.imshow('test', result)
    
    # HoughLinesP, 
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return (0, 640), frame, delta

    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    lx1, lx2, lpos, m1, b1 = get_line_pos(left_lines, left=True)
    rx1, rx2, rpos, m2, b2 = get_line_pos(right_lines, right=True)

    # get Vanishing Point
    x1, y1 = get_vanishing_points(m1, b1, m2, b2)
    if x_center == x1:
        pass
    else:
        temp =  (x1 - x_center) / (y_center - y1)
        delta = np.arctan(temp)
        # print('vanishing', (x1,y1), "slope", temp, delta)
    
    # print(delta)
    frame = cv2.circle(frame, (int(x1),int(y1)), 2, (255,0,255), 5)
    frame = cv2.circle(frame, (x_center,y_center), 2, (255,0,255), 5)
    frame = cv2.line(frame,(int(x1),int(y1)), (x_center, y_center), (0,0,255), 2)
    frame = cv2.line(frame, (int(lx1), Height), (int(lx2), (Height/2)), (255, 0,0), 3)
    frame = cv2.line(frame, (int(rx1), Height), (int(rx2), (Height/2)), (255, 0,0), 3)
    
    text = str(delta)
    thickness = 2
    frame = cv2.putText(frame, text, (10,40), 0, 1, (0,255,255), thickness)
    
    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    frame = cv2.rectangle(frame, (0, Offset), (Width, Offset + Gap), (255,0,0), 2)
    return (lpos, rpos), frame, delta

def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    #cv2.imshow('steer', image)
def plot_moving_average():
    global angle_list, angle_p, angle_pid, time_list
    plt.figure()
    plt.plot(time_list, angle_list)
    
    plt.plot(time_list, angle_pid)
    plt.legend(['ori', 'PID'])
    #plt.show()
    path = rospkg.RosPack().get_path('hough_drive_3') + '/src/pid.png'
    plt.savefig(path)
    print("bye!")

def start():
    global pub
    global image
    global Width, Height
    global angle_list, angle_pid, time_list
    global params

    rospy.init_node('auto_drive')
    # home
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    #cap = cv2.VideoCapture('track4.avi')

    #time.sleep(0.03)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    #rospy.sleep(1)
    
    while not image.size == (640*480*3):
        continue
    window = 10
    kp = params[1]
    ki = params[2]
    kd = params[3]

    mm1 = filter.MovingAverage(window)
    mm2 = filter.MovingAverage(window)
    error_init = 0
    ki_l = 0
    time = 0.03

    ap = -(1 - params[5]) / (10)
    am = (1 - params[5]) / (10)
    bp = 1 - 20*ap
    bm = 1 + 20*am
    delta = 0   
    while not rospy.is_shutdown():    
        #ret, image = cap.read()
        rospy.sleep(time)
        pos, frame, delta = process_image(image, delta)
        # mm1.add_sample(pos[0])
        # mm2.add_sample(pos[1])
        
        # center = (mm1.get_wmm() + mm2.get_wmm()) / 2        
        
        # pos = np.array([pos[0], pos[1]])
        # pos = lpf.LPF(pos)        

        # center = (pos[0] + pos[1]) / 2        
        # error = (320 - center)
        
        
        
        # ki_l += error*time 

        # angle = error * kp + ki * ki_l + ((error - error_init)/time) * kd
        # error_init = error
        steer_angle = delta / 1.5 * 40.0 + 8
        steer_angle = int(filter_steer.LPF(np.array([steer_angle])))
        print(delta, steer_angle)
        # origin = -error
        
        # angle_pid.append(steer_angle)
        # time_list.append(rospy.get_time())
        # angle_list.append(origin)
        
        if np.abs(steer_angle) < 10 :
            speed = params[0]
            drive(steer_angle, int(speed)) #able:~180
        elif np.abs(steer_angle) < 30:
            if steer_angle > 0:
                cof = ap*steer_angle + bp
                speed = cof*params[0]
                drive(steer_angle, int(speed)) #able:~180
            
            else:
                cof = am*steer_angle + bm
                speed = cof*params[0]
                drive(steer_angle, int(speed)) #able:~180
                
        else:
            speed = params[5] * params[0]
            drive(steer_angle, int(speed)) #able:~180
        
        # print(pos[0], pos[1], steer_angle)
        # cv2.imshow('steer',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # filter_steer.analyze()
    # filter_steer.plot()
    # plot_moving_average()

if __name__ == '__main__':
    # params = [30, 0.8, 0.0005, 0.0001, 20, 0.8] # speed, p i d, line_thresh, cof_v_rev
    params = [40, 0.9, 0.005, 0.0001, 20, 0.8] # speed, p i d, line_thresh, cof_v_rev
    cann = [90, 120] # low high
    angle_list = []
    angle_pid = []
    angle_p = []    
    time_list = []
    start()
