#!/usr/bin/env python3

import rospy
from motors import RobotMotors
from geometry_msgs import Twist

class Robot:

    def __init__(self):
        self.motors = RobotMotors()
        self.control_subscriber = rospy.Subscriber("/cmd_vel", Twist, callback=self.control_callback, queue_size=10)

    def control_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        Ml = (1 + angular) * linear * 100
        Mr = (1 - angular) * linear * 100

        Ml = max(min(Ml, 100), -100)
        Mr = max(min(Mr, 100), -100)

        MlDir = "forward" if Ml >= 0 else "backward"
        MrDir = "forward" if Mr >= 0 else "backward"

        self.motors.motorRun(0, MlDir, Ml)
        self.motors.motorRun(1, MrDir, Mr)

if __name__ == "__main__":
    rospy.init_node("robot_node")
    R = Robot()








