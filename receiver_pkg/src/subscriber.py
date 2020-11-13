#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callbackfunc(msg):
    rospy.loginfo(rospy.get_caller_id() + " Recibi %s", msg)

def listener():
    rospy.init_node('subscriber')
    rospy.Subscriber('demo_pub',String, callbackfunc)
    rospy.spin()

if __name__ == "__main__":
    listener()