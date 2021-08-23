#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("example_motors")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    t = Twist()
    t.linear.x = 0.5
    print(t)
    pub.publish(t)
    
