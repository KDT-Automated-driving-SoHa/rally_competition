#!/usr/bin/env python

import rospy, math
import cv2, time
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

xycar_msg = Int32MultiArray()

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}

roll, pitch, yaw = 0, 0, 0

def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

def back():
    global motor_pub, xycar_msg
    for i in range(15):
        xycar_msg.data = [0,-10]
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)

rospy.init_node('ar_parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
xycar_msg = Int32MultiArray()
#angles=[i for i in range(50)]
#j=0
#while arData["AY"] ==0:
#    continue

while not rospy.is_shutdown():
    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    print("=======================")
    print(" roll : " + str(round(roll,1)))
    print("pitch : " + str(round(pitch,1)))
    print(" yaw  : " + str(round(yaw,1)))

    print(" x : " + str(arData["DX"]))
    print(" y : " + str(arData["DY"]))
    print(" z : " + str(arData["DZ"]))

    img = np.zeros((100,500,3))

    cv2.line(img,(10,55),(10,75),(0,0,255),2)
    cv2.line(img,(250,55),(250,75),(0,0,255),2)
    cv2.line(img,(490,55),(490,75),(0,0,255),2)
    cv2.line(img,(10,65),(490,65),(0,0,255),2)

    point = int(250*(arData["DX"]/arData["DY"])/0.789) if int(arData["DY"]) !=0 else 250
    """
    if point > 240:
        point = 240
    elif point < -240:
        point = -240
    """
    img = cv2.circle(img,(250+point,65),10,(0,255,0),-1)

    distance = (arData["DX"]**2 + arData["DY"]**2)**0.5

    cv2.putText(img,str(int(distance))+" pixel",(350,30),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"]))+" Yaw:"+str(round(yaw,1))
    cv2.putText(img, dx_dy_yaw, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255))
    
    cv2.imshow('AR Tag position',img)
    cv2.waitKey(10)

    angle = int(point/4)-yaw
    #angle = angles[j]
    #j=j+1 if j<49 else 0
    if int(arData["DY"]) > 0 and int(arData["DY"]) <= 70:
        if abs(yaw) >= 7:
            back()
        else:
            speed = 0
    elif int(arData["DY"]) > 70 and int(arData["DY"]) <=150:
        speed = 5
    else:
        speed = 20 
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
    if int(arData["DY"]) < 40:
        back()
cv2.destroyAllWindows()

