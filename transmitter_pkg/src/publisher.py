#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('demo_pub', String, queue_size=1)
    rospy.init_node('publisher')
    rate = rospy.Rate(1) # 10 Hz
    while not rospy.is_shutdown():
        message_str = "ROS Publisher Demo %s" % rospy.get_time()
        rospy.loginfo(message_str)
        pub.publish(message_str)
        rate.sleep()

if __name__ == '__main__':
    talker()
