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
        camera_width = rospy.get_param("~camera_width", default=640)
        camera_height = rospy.get_param("~camera_height", default=480)
        publish_rate = rospy.get_param("~publish_rate", default=10)
        self.camera.resolution = (640,480)
        self.framerate = 30
        time.sleep(2)
        self.stream = picamera.array.PiRGBArray(self.camera, size=(camera_width, camera_height))
        self.rate = rospy.Rate(publish_rate)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("/camera", Image, queue_size=10)
        

    def run(self):
        for frame in self.camera.capture_continuous(self.stream, format="bgr", use_video_port=True):
            if rospy.is_shutdown():
                break
            image = frame.array
            self.stream.truncate(0)
            if image is not None:
                img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.image_pub.publish(img_msg)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("camera_node")
    camera = CameraNode()
    camera.run()