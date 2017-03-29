#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class DummyAgent:
    def __init__(self):
        print "DummyAgent::init"
        
        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        rospy.init_node('agent_node', anonymous=True)
        
        self.pub = rospy.Publisher('action', String, queue_size = 10)
        
        self.rate = rospy.Rate(10) # 10hz
        
        rospy.Subscriber("perception", String, self.callback)

    def publish_action(self):
        action_str = "skip"
        rospy.loginfo(action_str)
        self.pub.publish(action_str)
        self.rate.sleep()
        
    def callback(self, data):
        print "DummyAgent::callback", data
        self.publish_action()
        
if __name__ == '__main__':
    print "dummy_agent::main"
    try:
        dummy = DummyAgent()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
