#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import (GetWorldProperties, GetModelState)

class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(wp.status_message)    
        except rospy.ServiceException, e:
            rospy.logerr("/gazebo/get_world_properties: %s"%e)

    def getModelState(self, model_name, relative_entity_name):
        try:
            get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:
                rospy.logwarn(ms.status_message)    
        except rospy.ServiceException, e:
            rospy.logerr("/gazebo/get_model_state: %s"%e)


def main():
    gu = GazeboUtils()
    wp = gu.getWorldProperties()
    if wp.success:
        print(wp.model_names)
    else:
        print("Ver registro de errores 1")
    ms = gu.getModelState('turtlebot3_waffle', 'unit_cylinder')        
    if ms.success:
        print(ms.pose.position)
    else:
        print("Ver registro de errores 2")

if __name__ == "__main__":
    main()