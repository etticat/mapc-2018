import time

import rospy
from mac_ros_bridge.msg import RequestAction

from agent_knowledge.resource import ResourceKnowledgebase
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class DebugAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)
        time.sleep(1)
        DebugUtils.instant_find_resources(ResourceKnowledgebase())
        # time.sleep(1)
        # rospy.signal_shutdown("end of debug code")
        # DebugUtils.add_build_well_task()
        # DebugUtils.assign_assembly_task()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)


    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        ettilog.logerr("-----------------------------------")
        ettilog.logerr("Ingredients:")
        stock = DebugUtils.show_total_stock_with_goals()
        for i in range(0,5):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])

        ettilog.logerr("Finished products:")
        for i in range(5,11):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])


if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = DebugAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
