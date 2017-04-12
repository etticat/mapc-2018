#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction

class RhbpAgent:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        rospy.init_node('agent_node', anonymous=True)

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        self._agent_topic_prefix = 'bridge_node_' + self.agent_name + '/'

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self.callback)

        #TODO provide/subscribe start services, might also be necessary to implemented the missing functionality in the bridge_node
        #in this initialization callback we should create the behaviours sensors etc, because this allows to adapt to a particular role


    def callback(self, msg):
        """
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        rospy.logdebug("RhbpAgent::callback %s", msg)
        #TODO trigger manager.step
        #TODO adjust RBHP in order to select only a maximum amount of parallel actions (in our case 1)
        #action send is triggered by a particular behaviour

if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
