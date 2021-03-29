#!/usr/bin/env python
import rospy
import cv2, math
import numpy as np
import matplotlib.pyplot as plt


from sensor_msgs.msg import LaserScan
from ros_manager import RosManager

class TrnasformCP():
    def __init__(self, param):
        ## distance ~= 435.662 PIXEL or 0.845m 
        ## 515.58 PIXEL / m
        ## Offset from lidar to ROI : 0.400 m

        ############### param set #########################
        
        self.C_roi_p_x = 480
        self.C_roi_p_y = 480
        self.C_roi_x = 0.931
        self.C_roi_y = 0.931
        self.Offset_roi = 0.350
        self.m_to_PIXEL = 515.58
        ## offset : distance from LIDAR to camera ROI with m unit
        ## C_x, C_y : Width & Height of camera ROI with m unit
        
        self.alphas = []
        self.del_alphas = param[0]
        self.number = param[1]

    def set_azimuth(self):
        self.alphas = [ self.del_alphas * i for i in range(self.number)]

    def rm_spherical(self, data, number):
        stand = int(self.number/2)
        number = int(number/2)
        return data[stand-number:stand+number], self.alphas[stand-number:stand+number]

    def get_cartesian(self, rs, als):
        cartesian = np.array([[0],[0],[0]])
        for r, alpha in zip (rs, als):
            cartesian = np.append(cartesian, np.array([[r*np.cos(alpha)], [r*np.sin(alpha)], [1]]), axis=1)
        return cartesian[:,1:]
        
    def transform_to_ROI(self, data, theta=-np.pi/2, dir_x=-1, dir_y=1):
        ## theta : rotate angle by z axis
        ## dir_x, dir_y : for invese axis, '-1' means invert result about a perendicular axis 
        data = np.array([dir_x*data[0,:], dir_y*data[1,:], data[2,:]])
        T = np.array([[np.cos(theta), -np.sin(theta), self.C_roi_y/2.0],
                    [np.sin(theta), np.cos(theta), +self.Offset_roi +self.C_roi_x],
                    [0, 0, 1]])
        return np.matmul(T, data)

    def rm_cartesian(self, cartesian, offset=0.0):
        result = np.array([[0],[0],[1]])
        num_car = cartesian.shape[1]
        for i in range(num_car):
            temp = cartesian[:,i]
            if temp[0] > 0 and temp[0] < self.C_roi_x and temp[1] > 0:
                result = np.append(result, np.array([[temp[0]],[temp[1]],[temp[2]]]), axis=1)
        return result.shape[1]-1, self.m_to_PIXEL*result[:,1:]
    
    def get_line(self, image, num, data):
        CP_bool = np.zeros([self.C_roi_p_x, self.C_roi_p_y], dtype=np.uint8)
        for i in range(num):
            temp_x = int(data[0,i])
            temp_y = int(data[1,i])
            if temp_x < self.C_roi_p_x and temp_y < self.C_roi_p_y:
                CP_bool[temp_y, temp_x] = 255
        lines = cv2.HoughLinesP(CP_bool,1,math.pi/180,1,1,1)
            
        if lines == None:
            return False, image
            # exit(0)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            image = cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 2)
        return True, image

    def get_segment(self, image, cp, left, right):
        min_dist_left = 480
        min_dist_right = 480
        min_ptset_left = None
        min_ptset_right = None
        if left == None and right == None:
            pass
        elif left == None:
            for i in range(cp.shape[1]):
                temp_x = cp[0,i]
                temp_y = cp[1,i]
                right_x = right(temp_y)
                dist_right = right_x - temp_x
                if dist_right > 0:

                    cv2.circle(image, (int(temp_x), int(temp_y)), 5, (255,255,0), -1)
        elif right == None:
            for i in range(cp.shape[1]):
                temp_x = cp[0,i]
                temp_y = cp[1,i]
                left_x = left(temp_y)
                if left_x < temp_x:
                    cv2.circle(image, (int(temp_x), int(temp_y)), 5, (255,0,0), -1)
        else:
            num_pt = [0, 0]
            for i in range(cp.shape[1]):
                temp_x = cp[0,i]
                temp_y = cp[1,i]
                left_x = left(temp_y)
                right_x = right(temp_y)

                dist_left = temp_x - left_x
                dist_right = right_x - temp_x
                if dist_left > 0 and dist_right > 0:     
                    if temp_x < left_x + (right_x - left_x) / 2:
                        cv2.circle(image, (int(temp_x), int(temp_y)), 5, (255,0,0), -1)
                        if dist_left < min_dist_left:
                            num_pt[0] += 1
                            min_dist_left = dist_left
                            min_ptset_left = (int(temp_x), int(temp_y))
                    else: 
                        cv2.circle(image, (int(temp_x), int(temp_y)), 5, (255,255,0), -1)
                        if dist_right < min_dist_right:
                            num_pt[1] += 1
                            min_dist_right = dist_right
                            min_ptset_right = (int(temp_x), int(temp_y))
            if num_pt[0] > num_pt[1]:
                if not min_ptset_right == None:
                    cv2.circle(image, min_ptset_right, 5, (0, 0, 255), -1)
                else: 
                    cv2.circle(image, min_ptset_left, 5, (0, 0, 255), -1)
                
            elif num_pt[0] < num_pt[1]:
                if not min_ptset_left == None:
                    cv2.circle(image, min_ptset_left, 5, (0, 0, 255), -1)
                else: 
                    cv2.circle(image, min_ptset_right, 5, (0, 0, 255), -1)
        

        return True, image
ax1 = None
ax2 = None
def set_animation():
    global ax1, ax2
    fig = plt.figure(figsize=(16,8))
    ax1 = fig.add_subplot(1,2,1)
    ax1.set_xlabel("X_rear")
    ax1.set_ylabel("Y_left")
    ax1.grid(True)
    ax2 = fig.add_subplot(1,2,2)
    ax2.set_xlabel("Y_left")
    ax2.set_ylabel("X_rear")
    ax2.grid(True)
    # plt.axis([-10,10,-10,10])
    

def plot_animation(data, data_transform, target):
    global ax1,ax2
    # plt.cla()
    ax1.plot(data[0,0:target], data[1,:target], 'ro', markersize=1)
    ax1.plot(data[0,target], data[1,target], 'bo', markersize=5)
    ax1.plot(data[0,target+1:], data[1,target+1:], 'ro', markersize=1)
    ax2.plot(data_transform[0,0:target], data_transform[1,:target], 'ro', markersize=1)
    ax2.plot(data_transform[0,target], data_transform[1,target], 'bo', markersize=5)
    ax2.plot(data_transform[0,target+1:], data_transform[1,target+1:], 'ro', markersize=1)
    plt.pause(0.01)


def listener():

    manager = RosManager()

    ret = False
    test_num = 0
    ret, cp_param = manager.get_lidar_param()

    while not ret:
        ret, cp_param = manager.get_lidar_param()
        continue

    tfcp = TrnasformCP(cp_param)

    tfcp.set_azimuth()
    set_animation()

    while not rospy.is_shutdown():
        _, cp_law = manager.get_lidar()
        cloud_r, cloud_al = tfcp.rm_spherical(cp_law, 200)
        cartesian = tfcp.get_cartesian(cloud_r, cloud_al)
        camera_mks = tfcp.transform_to_ROI(cartesian)
        num_data, camera_mks_rm = tfcp.rm_cartesian(camera_mks)
        plot_animation(camera_mks, camera_mks_rm, num_data/2)
    plt.show()
    

if __name__ == '__main__':
    listener()
