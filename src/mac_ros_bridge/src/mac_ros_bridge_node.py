#!/usr/bin/env python2

import roslib
import rospy

import geometry_msgs.msg

class MacRosBridge:
    def __init__(self):

        rospy.init_node('mac_ros_bridge_node')

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')


if __name__ == '__main__':
    try:

        bridge = MacRosBridge()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
