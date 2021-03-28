#!/usr/bin/env python
# -*- coding: utf-8 -*-

from motor_info import MotorInfo

class StoplineDrive:
    def _get_detected_line_y(self, speed):
        # 정지선을 여부를 확인하기 위한 검출 y좌표 계산
        detected_line_y = 90
        if speed <= 50:
            detected_line_y = detected_line_y
        elif speed <= 40:
            detected_line_y += 80
        elif speed <= 30:
            detected_line_y += 200
        return detected_line_y


    def _get_cross_value_count(self, frame, y):
        # y좌표를 기준으로 색상이 교차될 때, 각 생상의 value 갯수 
        value_counts = []

        # 처음 정지선을 확인하기 위해, (0, y)의 value 값을 초기값으로 설정
        value = frame[y][0]
        count = 0

        height, width = frame.shape[:2]
        for x in range(width):
            # (x, y)와 value 값이 같을 경우, 해당 좌표의 갯수를 카운트
            if frame[y][x] == value:
                count += 1
            # 다를 경우, 좌표 갯수를 리스트에 저장한 뒤, 새로운 value 값 및 카운트 초기화
            else:
                value_counts.append(count)
                value = frame[y][x]
                count = 1
        value_counts.append(count)

        # """
        # Explain Matrix
        # """
        # import cv2 as cv
        # explain_image = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
        # cv.rectangle(explain_image, (0, y-1), (explain_image.shape[1], y+1), (0, 0, 255), 1)
        # cv.imshow("origin" + str(y), explain_image)
        # margin = 1
        # cv.rectangle(explain_image, ())
        # cv2.line(explain_image,(0,y_th),(explain_image.shape[1],y_th),(0,0,255))
        # cv2.imshow(name,explain_image)
        return value_counts


    def _is_detect_crossline(self, frame, speed, delta_y=100, cross_count_thresh=17):
        # 정지선 여부를 검사하기 위한 y좌표의 상/하한 계산
        detected_line_up = self._get_detected_line_y(speed)
        detected_line_down = detected_line_up + delta_y

        # 상/하한에서 검출되는 교차되는 value 값의 갯수 계산
        cross_count_up = self._get_cross_value_count(frame, detected_line_up)
        cross_count_down = self._get_cross_value_count(frame, detected_line_down)

        # 상단에서 검출되는 갯수 세기
        if len(cross_count_up) > cross_count_thresh:
            print "[STOPLINE] CROSS LINE UP", len(cross_count_up)
            return True, "up"
        if len(cross_count_down) > cross_count_thresh:
            print "[STOPLINE] CROSS LINE DOWN", len(cross_count_down)
            return True, "down"
        return False, "no"

    
    def get_motor_info(self, frame, speed):
        ret, where = self._is_detect_crossline(frame, speed)

        if ret:
            motor_info_list = []

            if where == "up":
                motor_info_list.append(MotorInfo(angle=0, speed=speed, iterations=15))
            motor_info_list.append(MotorInfo(angle=0, speed=0, iterations=30, delay_sec=0.1))
            motor_info_list.append(MotorInfo(angle=0, speed=speed*2//3, iterations=20, delay_sec=0.1))
            
            return motor_info_list
        else:
            return MotorInfo(angle=0, speed=speed)


    # def is_stopline(self, y_th=240, len_th=100):
    #     lis = self._detect_stopline(y_th,'stop')
    #     if len(lis)!=0 and max(lis)>=len_th:
    #         print("stop line",max(lis))
    #         return True
    #     return False



# if __name__ == '__main__':
    
#     speed = 50
#     da = 7
#     thres_50 = 90
#     if speed == 30:
#         #thres_y = 200
#         thres_y = thres_50 + 200
#     elif speed == 40:
#         #thres_y = 100
#         thres_y = thres_50 + 80
#     elif speed == 50:
#         thres_y = thres_50
#     stop = Stopline('174', speed=speed, thres_L=200)
#     rospy.init_node('stopline')
#     rospy.Subscriber("/usb_cam/image_raw", Image, stop.camera_callback)
#     pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

#     while not rospy.is_shutdown():
#         if stop.cam_image.size != (640*480*3):
#             print("not yet")
#             continue

#         #is_stline = stop.is_stopline(y_th=240, len_th=300)
#         is_crossline, where  = stop.is_crossline(y_th=thres_y, num_th=15)
#         if is_crossline:
#             if where =='up':
#                 for _ in range(15):
#                     drive(da,20,pub)
#                     time.sleep(0.1)
#             #print("stopline")
#             for _ in range(30): # 60
#                 #print()
#                 drive(da, 0, pub)
#                 time.sleep(0.1)
#             for _ in range(20):
#                 #drive(da,0,pub)
#                 drive(da, speed*2//3, pub)  # 30
#                 time.sleep(0.1)
#             #speed = 0
#         else:
#             drive(da, speed ,pub)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
