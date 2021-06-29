#!/usr/bin/env python3

import rospy
from motors import RobotMotors
from geometry_msgs import Twist

class Robot:

    def __init__(self):
        self.motors = RobotMotors()
        self.control_subscriber = rospy.





