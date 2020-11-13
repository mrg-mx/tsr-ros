#! /usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from localization_monitor.msg import Robot

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    robot_situation = Robot()
    robot_situation.estatus = 'x: {} y: {} z: {}'.format(x, y, z)
    pub.publish(robot_situation)

def main():
    global pub
    
    rospy.init_node('localization_monitor')
    pub = rospy.Publisher('robot_estatus', Robot, queue_size=1)
    rospy.Subscriber('/odom', Odometry, callback )
    rospy.spin()

if __name__ == "__main__":
    main()