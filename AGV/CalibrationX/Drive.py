#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from pymycobot.myagv import MyAgv
import time

class EnhancedMyAgv(MyAgv):
    def __init__(self, port, baudrate):
        super().__init__(port, baudrate)

    def forward(self, vertical_speed=10, horizontal_speed=0, turn_speed=0, timeout=5):
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128 + vertical_speed, 128 + horizontal_speed, 128 + turn_speed)
            time.sleep(0.1)
        self.stop()

    def forward_L(self, vertical_speed=10, horizontal_speed=0, turn_speed=10, timeout=5):
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128 + vertical_speed, 128 + horizontal_speed, 128 + turn_speed)
            time.sleep(0.1)
        self.stop()

    def forward_R(self, vertical_speed=10, horizontal_speed=0, turn_speed=10, timeout=5):
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128 + vertical_speed, 128 + horizontal_speed, 0 + turn_speed)
            time.sleep(0.1)
        self.stop()

class AGVController:
    def __init__(self):
        self.agv = EnhancedMyAgv('/dev/ttyAMA2', 115200)
        rospy.init_node('agv_camera_controller', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('video_frames', Image, queue_size=10)
        self.sub = rospy.Subscriber('control_commands', String, self.control_callback)
    
        self.cap = cv2.VideoCapture('/dev/video0')  # 카占쌨띰옙 占쏙옙占쏙옙決占?占쏙옙占쏙옙



        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
	
		#주행속력 <- Calibration
    def control_callback(self, data):
        command = data.data
        rospy.loginfo(f"Received command: {command}")
        if command == "Forward":
            self.agv.forward(2, 0, 0, 0.1)
            #self.print('forward')

        elif command == "Forward_L":
            self.agv.forward_L(2, 0, 1, 0.1)
            #self.print('forward_L')

        elif command == "Forward_R":
            self.agv.forward_R(2, 0, 1, 0.1)
            #self.print('forward_R')

    def publish_camera_frames(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.pub.publish(ros_image)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))
            rate.sleep()

if __name__ == '__main__':
    controller = AGVController()
    try:
        controller.publish_camera_frames()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cap.release()
