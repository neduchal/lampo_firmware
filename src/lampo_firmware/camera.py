import time
import picamera
import picamera.array
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class CameraNode:

    def __init__(self):
        self.camera = picamera.Picamera()
        self.camera.start_preview()
        time.sleep(2)
        self.stream = picamera.array.PiRGBArray(self.camera)
        self.rate = rospy.Rate(10)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("/camera", Image, queue_size=10)
        

    def run(self):
        while True:
            if rospy.is_shutdown():
                break
            self.camera.capture(self.stream, format="brg")
            image = self.stream.array
            if image is not None:
                img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.image_pub.publish(img_msg)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("lampo_camera_node")
    camera = CameraNode()
    camera.run()