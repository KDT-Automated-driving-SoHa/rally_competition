import rospy

from xycar_motor.msg import xycar_motor
from subscribers.camera_subscriber import CameraSubscriber

class RosManager():
    def __init__(self, name="driver", rate=10):
        self.motor_msg = xycar_motor()

        self.motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        self.camera_sub = CameraSubscriber()

        rospy.init_node("driver")
        self.rate = rospy.Rate(10)

    
    def get_image(self):
        return self.camera_sub.get()


    def publish_motor(self, angle, speed=0):
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor_pub.publish(self.motor_msg)
        self.rate.sleep()


if __name__ == "__main__":
    import cv2 as cv

    controller = RosManager()
    while not rospy.is_shutdown():
        ret, img = controller.get_image()
        if ret:
            cv.imshow("test", img)
            cv.waitKey(10)
