#!/usr/bin/env python3

import time
import picamera
import picamera.array
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class CameraNode:

    def __init__(self):
        self.camera = picamera.PiCamera()
        #self.camera.start_preview()
        self.camera.resolution = (640,480)
        self.framerate = 30
        time.sleep(2)
        self.stream = picamera.array.PiRGBArray(self.camera, size=(640, 480))
        self.rate = rospy.Rate(10)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("/camera", Image, queue_size=10)
        

    def run(self):
        for frame in self.camera.capture_continuous(self.stream, format="bgr", use_video_port=True):
            if rospy.is_shutdown():
                break
            #self.camera.capture(self.stream, format="bgr")
            #image = self.stream.array
            image = frame.array
            self.stream.truncate(0)
            if image is not None:
                img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.image_pub.publish(img_msg)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("lampo_camera_node")
    camera = CameraNode()
    camera.run()