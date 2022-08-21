#!/usr/bin/env python3

import rospy
from display import Display
from motors import RobotMotors
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32

class Robot:

    def __init__(self):
        self.motors = RobotMotors()
        self.display = Display()
        self.control_subscriber = rospy.Subscriber("/cmd_vel", Twist, callback=self.control_callback, queue_size=10)
        self.left_motor_subscriber = rospy.Subscriber("/cmd_left_motor", Int32, callback=self.left_motor_callback, queue_size=10)
        self.right_motor_subscriber = rospy.Subscriber("/cmd_right_motor", Int32, callback=self.right_motor_callback, queue_size=10)
        self.display_subscriber = rospy.Subscriber("/display", String, callback=self.display_callback, queue_size=10)

    def display_callback(self, msg):
        self.display.addString(msg.data)
        self.display.push()
        pass

    def constraint_motor_value(self, value):
        value = max(min(value, 100), -100)   
        motor_direction = "forward" if value >= 0 else "backward"
        return motor_direction, abs(value)

    def left_motor_callback(self, msg):
        motor_direction, value = self.constraint_motor_value(msg.data)
        self.motors.motorRun(0, motor_direction, value)

    def right_motor_callback(self, msg):
        motor_direction, value = self.constraint_motor_value(msg.data)
        self.motors.motorRun(1, motor_direction, value)

    def compute_motor_values(self, linear, angular):
        Ml = 0
        Mr = 0
        if (linear != 0):
            Ml = (1 - angular) * linear * 100
            Mr = (1 + angular) * linear * 100
        else:
            Ml = - angular * 100
            Mr = angular * 100    
        return Ml, Mr

    def control_callback(self, msg):
        Ml, Mr = self.compute_motor_values(msg.linear.x, msg.angular.z)

        MlDir, Ml = self.constraint_motor_value(Ml)
        MrDir, Mr = self.constraint_motor_value(Mr)
        self.motors.motorRun(0, MlDir, abs(Ml))
        self.motors.motorRun(1, MrDir, abs(Mr))

    def run(self):
        r = rospy.Rate(0.25) # 10hz 
        i = 0
        while not rospy.is_shutdown():
            if i % 2 == 1:
                self.display.drawIP()
            else:
                self.display.push()
            i += 1
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("robot_node")
    R = Robot()
    R.run()
    R.display.clear()








