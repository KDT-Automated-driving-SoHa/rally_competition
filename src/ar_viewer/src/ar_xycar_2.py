#!/usr/bin/env python

import rospy, math, rospkg
import cv2, time
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor
xycar_msg = Int32MultiArray()

arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}

roll, pitch, yaw = 0, 0, 0
angle = 0
speed = 0
is_data = False
def callback(msg):
    global arData, is_data
    if len(msg.markers) == 0:
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
        for i in msg.markers:
            arData["DX"] = i.pose.pose.position.x * 100
            arData["DY"] = i.pose.pose.position.y * 100
            arData["DZ"] = i.pose.pose.position.z * 100

            arData["AX"] = i.pose.pose.orientation.x
            arData["AY"] = i.pose.pose.orientation.y
            arData["AZ"] = i.pose.pose.orientation.z
            arData["AW"] = i.pose.pose.orientation.w

def drive(Angle,Speed):
    global mpub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    mpub.publish(msg)

def back():
    global motor_pub, xycar_msg
    global mpub
    for i in range(15):
        xycar_msg.data = [0,-10]
        motor_pub.publish(xycar_msg)
        drive(0,-20)
        time.sleep(0.1)

rospy.init_node('ar_parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
mpub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)

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

    point = int(250*(arData["DX"]/arData["DZ"])/0.789) if int(arData["DZ"]) !=0 else 250
    """
    if point > 240:
        point = 240
    elif point < -240:
        point = -240
    """
    img = cv2.circle(img,(250+point,65),10,(0,255,0),-1)

    #distance = (arData["DX"]**2 + arData["DY"]**2)**0.5
    distance = arData["DZ"]

    cv2.putText(img,str(int(distance))+" pixel",(350,30),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DZ"]))+" Pitch:"+str(round(pitch,1))
    cv2.putText(img, dx_dy_yaw, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255))
    
    cv2.imshow('AR Tag position',img)
    cv2.waitKey(10)
    
    theta = math.atan(arData["DX"]/arData["DZ"])*180/math.pi if arData["DZ"] !=0 else 0
    w_t = 0.25
    w_p = 0.5
    
    angle = int(theta * w_t - pitch * w_p)
    print("theta : "+str(theta)+" angle: "+str(angle))
    #angle = angles[j]
    #j=j+1 if j<49 else 0
    if is_data:
        if int(arData["DZ"]) == 0:
            speed,angle = 0,0
        elif int(arData["DZ"]) > 0 and int(arData["DZ"]) <= 40:
            if abs(pitch) >= 10:
                back()
            else:
                speed = 0
        elif int(arData["DZ"]) > 40 and int(arData["DZ"]) <=80:
            speed = 20
        else:
            speed = 30
    else:
        speed,angle =0,0 
    xycar_msg.data = [angle, speed]
    #motor_pub.publish(xycar_msg)
    drive(angle,speed)
    roll,pitch,yaw = 0,0,0
    #arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
    
cv2.destroyAllWindows()

