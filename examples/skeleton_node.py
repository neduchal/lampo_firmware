#!/usr/bin/env python3

from pyzbar.pyzbar import decode
import time
import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SkeletonNode:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("/camera",
            Image, callback=self.image_callback, queue_size=1)
        self.display_pub = rospy.Publisher("/display",
            String, queue_size=10)
        self.motors_pub = rospy.Publisher("/cmd_vel",
            Twist, queue_size=10)
        self.image = None
        self.state = "idle"
        display_text = String()
        display_text.data = self.state
        self.display_pub.publish(display_text)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def lines_position(self, image, crop_top, threshold):
        image_thr = image[crop_top:,:,0]>threshold
        image_bin = np.array(image_thr*255, dtype=np.uint8)
        
        img_sum = np.sum(image_bin, axis=0)
        img_sum_len = len(img_sum)
        # LEFT
        state = 0
        first_index_left = 0
        last_index_left = (img_sum_len//2)
        for index, value in enumerate(img_sum[0:(img_sum_len//2)][::-1]):
            if state == 0 and value != 0:
                first_index_left = index
                print(index, value)
                state = 1
            elif state == 1 and value == 0:
                last_index_left = index
                print(index, value)            
                break       
        mean_index_left = -(first_index_left + last_index_left)/2
        
        # RIGHT
        state = 0
        first_index_right = 0
        last_index_right = (img_sum_len//2)
        for index, value in enumerate(img_sum[(img_sum_len//2):]):
            if state == 0 and value != 0:
                first_index_right = index
                state = 1
                print(index, value)            
            elif state == 1 and value == 0:
                last_index_right = index
                print(index, value)            
                break       
        mean_index_right = (first_index_right + last_index_right)/2    
        
        mean_index = (mean_index_left + mean_index_right)/2
        return mean_index

    def run(self):
        r = rospy.Rate(0.5)
        i = 0
        while not rospy.is_shutdown():
            if self.state == "idle":
                if self.image is None:
                    continue
                image = self.image.copy()
                codes = decode(image)
                for code in codes:
                    text = code.data.decode("utf-8")
                    if text == "dopredu":
                        self.state = "run"
                        display_text = String()
                        display_text.data = self.state
                        self.display_pub.publish(display_text)
                        time.sleep(2)
            elif self.state == "run":
                if self.image is None:
                    continue
                image = self.image.copy()
                image_gray = cv2.cvtColor(image,
                                          cv2.COLOR_BGR2GRAY)
                lines_index = self.lines_position(image_gray, crop_top=200, threshold=220)
                msg = Twist()
                msg.linear.x = 0.4
                if lines_index > 10:                    
                    msg.angular.z = 0.5
                elif lines_index < -10:
                    msg.angular.z = -0.5
                self.motors_pub.publish(msg)
                r.sleep()


if __name__ == "__main__":
    rospy.init_node("example_skeleton_node")
    node = SkeletonNode()
    node.run()
