#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction

class DummyAgent:
    def __init__(self):
        print "DummyAgent::init"
        
        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        rospy.init_node('agent_node', anonymous=True)

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        self._agent_topic_prefix = 'bridge_node_' + self.agent_name + '/'
        
        self._pub_generic_action = rospy.Publisher('~generic_action', GenericAction, queue_size = 10)
        
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self.callback)

    def publish_action(self, related_request):
        """
        :param related_request: the request we are answering
        :type related_request: RequestAction
        """
        action_str = "skip"

        msg = GenericAction()
        msg.id=related_request.id
        msg.target_deadline=related_request.deadline
        msg.action_type = action_str
        msg.param=""

        rospy.loginfo("Published action %s", msg)

        self._pub_generic_action.publish(msg)

    def callback(self, msg):
        """
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        print "DummyAgent::callback", msg
        self.publish_action(related_request=msg)
        
if __name__ == '__main__':
    print "dummy_agent::main"
    try:
        dummy = DummyAgent()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
