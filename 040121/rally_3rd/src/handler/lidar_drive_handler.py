#!/usr/bin/env python
# -*- coding: utf-8 -*-

from handler import AbstractHandler
from module.infos.motor_info import MotorInfo
from module.utils.discreate_filter import DiscreateFilter
from module.lidar_drive.transform_cloud_points import TrnasformCP

class LidarDriveHandler(AbstractHandler):

    def __init__(self):
        self.tfcp = None
        self.filter_obs = DiscreateFilter(f_cut=1000, freq=10000)
        self.filter_obs_vel = DiscreateFilter(f_cut=1000, freq=10000)


    def handle(self, handler_info):
        frame = handler_info.image
        lidar_info = handler_info.lidar_info
        lidar_param = handler_info.lidar_param

        if lidar_info:
            if self.tfcp is None:
                self.tfcp = TrnasformCP(lidar_param, K=180 / 3.14 * 1.0)
                self.tfcp.set_azimuth()

        cloud_r, cloud_al = tfcp.rm_spherical(lidar_info, 180)
        cartesian = tfcp.get_cartesian(cloud_r, cloud_al)
        camera_mks = tfcp.transform_to_ROI(cartesian)
        num_data, camera_mks_rm = tfcp.rm_cartesian(camera_mks)
        
        # ## lidar implement
        # _, cp_law = manager.get_lidar()
        # cloud_r, cloud_al = tfcp.rm_spherical(cp_law, 180)
        # cartesian = tfcp.get_cartesian(cloud_r, cloud_al)
        # camera_mks = tfcp.transform_to_ROI(cartesian)
        # # print(camera_mks) --
        # num_data, camera_mks_rm = tfcp.rm_cartesian(camera_mks)

        # 슬라이딩 윈도우 처리
        processed_sliding_window = preprocessing_sliding_window(frame)
        sliding_window_motor, linear_func_left, linear_func_right = sliding_window_dirve.get_motor_info(processed_sliding_window)

        speed = 35
        angle_obs, explain2, speed = tfcp.get_segment(processed_sliding_window, camera_mks_rm, linear_func_left, linear_func_right, speed)
        speed = filter_obs_vel.get_lpf(speed)[0]
        angle = sliding_window_motor.angle + int(angle_obs) + 8
        
        return MotorInfo(angle, speed)
