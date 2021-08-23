#!/usr/bin/env python3

import time
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class ExampleNode:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("/camera", Image, callback=self.image_callback, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # TODO

if __name__ == "__main__":
    rospy.init_node("example_camera_node")
    node = ExampleNode
    
    rospy.spin()