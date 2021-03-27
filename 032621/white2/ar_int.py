#!/usr/bin/env python

import cv2, time, math
import numpy as np

import rospy, rospkg
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

class Parking:
    def __init__(self,name='ar_parking'):

        self.arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
        self.ultrasonic_data = {"FL": 0, "FM": 0, "FR": 0, "L": 0, "BL": 0, "BM": 0, "BR": 0, "R": 0}

        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.angle = 0
        self.speed = 0

        self.is_data = False
        self.zs=[]
        self.xs=[]

        self.is_parking = False
        self.stage = 0
        if name != 'int':
            rospy.init_node(name)


    def callback(self,msg):
        #print(len(msg.markers))
        if len(msg.markers) == 0:
            self.is_data = False

            self.arData["DX"] = 0.0
            self.arData["DY"] = 0.0
            self.arData["DZ"] = 0.0

            self.arData["AX"] = 0.0
            self.arData["AY"] = 0.0
            self.arData["AZ"] = 0.0
            self.arData["AW"] = 0.0
        else:
            self.is_data = True

            new = []
            new2 = []

            for i in msg.markers:
                self.arData["DX"] = i.pose.pose.position.x * 100
                self.arData["DY"] = i.pose.pose.position.y * 100
                self.arData["DZ"] = i.pose.pose.position.z * 100

                new.append(self.arData["DZ"])
                new2.append(self.arData["DX"])

                self.arData["AX"] = i.pose.pose.orientation.x
                self.arData["AY"] = i.pose.pose.orientation.y
                self.arData["AZ"] = i.pose.pose.orientation.z
                self.arData["AW"] = i.pose.pose.orientation.w
            self.zs = new
            self.xs = new2
            (roll, pitch, yaw) = euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))

            self.roll = math.degrees(roll)
            self.pitch = math.degrees(pitch)
            self.yaw = math.degrees(yaw)

    def ultrasonic_callback(self,data):

        self.ultrasonic_data["L"] = data.data[0]
        self.ultrasonic_data["FL"] = data.data[1]
        self.ultrasonic_data["FM"] = data.data[2]
        self.ultrasonic_data["FR"] = data.data[3]
        self.ultrasonic_data["R"] = data.data[4]
        self.ultrasonic_data["BR"] = data.data[5]
        self.ultrasonic_data["BM"] = data.data[6]
        self.ultrasonic_data["BL"] = data.data[7]


    def park(self):
        if self.is_parking:
            if self.stage == 1:
                self.angle,self.speed = 0,0
                return 0

            elif self.stage == 2:
                if self.ultrasonic_data["BL"] < 50:
                    if self.ultrasonic_data["BM"] < 30:
                        print("===Stage3===")
                        self.stage = 3

                        self.angle,self.speed = 0,0

                    else:
                        self.angle, self.speed = -30, -18

                else:
                    self.angle, self.speed = 2, -20

            elif self.stage == 3:
                if self.arData["DZ"] > 50:
                    print(self.pitch, self.arData["DX"], self.arData["DZ"])

                    self.angle = self.arData["DX"] + 9
                    self.speed = 17

                else:
                    self.stage = 4
                    print("===Parked===")
                    self.angle, self.speed = 0, 0

            elif self.stage == 4:
                self.angle, self.speed = 0, 0

        else:
            if self.is_data:
                print("=======================")
                print("AR found")
                print("pitch : " + str(round(self.pitch, 1)))
                print(" x : " + str(self.arData["DX"]))
                print(" z : " + str(self.arData["DZ"]))
                k1 = 1
                k2 = 1
                if max(self.zs) < 70 and min(self.zs) > 0:
                    print("===Stage1===")
                    self.stage=1
                    self.is_parking = True
                    self.angle, self.speed = 0, 15
                    return 0
                if max(self.zs) == 0:
                    self.angle, self.speed = 0,17
                    return 0

                self.angle = k1 * math.atan((max(self.xs) - 35) / max(self.zs)) + self.pitch * k2
                self.speed = 20

            else:
                print("=======================")
                print("No AR tags")
                self.angle, self.speed = 0, 17

        self.roll, self.pitch, self.yaw = 0, 0, 0

def drive(Angle,Speed,pub):
    global da
    msg = xycar_motor()
    msg.angle = Angle+da
    msg.speed = Speed
    pub.publish(msg)


if __name__ == '__main__':
    da = 5
    ar = Parking('ar_parking')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    count = 3 #cross line num
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar.callback)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ar.ultrasonic_callback)

    while not rospy.is_shutdown():
        if count >=3:
            ar.park()
            if ar.stage == 1:
                for _ in range(30):
                    drive(0, 20, pub)
                    time.sleep(0.1)
                for _ in range(20):
                    drive(-50, 20, pub)
                    time.sleep(0.1)
                ar.stage = 2
                print("===Stage2===")

            else:
                drive(ar.angle,ar.speed,pub)
                time.sleep(0.1)
        else:
            print("count ",count)
            drive(0,20,pub)
            time.sleep(0.1)





