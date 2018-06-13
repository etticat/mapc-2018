import time

import rospy
from mac_ros_bridge.msg import Item, Position, RequestAction

from agent_knowledge.resource import ResourceKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from coordination.assemble_contractor import AssembleContractor
from coordination.assemble_manager import AssembleManager
from provider.product_provider import ProductProvider


class DebugAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)
        time.sleep(1)
        DebugUtils.instant_find_resources(ResourceKnowledgebase())
        # time.sleep(1)
        # rospy.signal_shutdown("end of debug code")
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)


    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        rospy.logerr("-----------------------------------")
        rospy.logerr("Ingredients:")
        stock = DebugUtils.show_total_stock_with_goals()
        for i in range(0,5):
            item = "item" + str(i)
            if item in stock.keys():
                rospy.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])

        rospy.logerr("Finished products:")
        for i in range(5,11):
            item = "item" + str(i)
            if item in stock.keys():
                rospy.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])


if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = DebugAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
