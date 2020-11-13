#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0.5 # Velocidad linel en m/s
    msg.angular.z = 0.5 # Velocidad angular en rad/s
    rospy.init_node('nav_ctrl')
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    move()