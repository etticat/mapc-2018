import time

import rospy
from mac_ros_bridge.msg import SimStart, RequestAction

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from decisions.assembly_combination import ChooseBestAssemblyCombination
from decisions.gathering import ChooseIngredientToGather
from provider.provider_info_distributor import ProviderInfoDistributor
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.test')

class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self._well_provider = WellProvider()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        self._provider_info_distributor = ProviderInfoDistributor()

        self.initialied = False

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._callback_sim_start)
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)


    def _callback_sim_start(self, simStart):
        """

        :param simStart:
        :type simStart: SimStart
        :return:
        """
        self._provider_info_distributor.callback_sim_start(simStart)


    def _action_request_callback(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """

        self._provider_info_distributor.callback_request_action(request_action)

        if not self.initialied:
            time.sleep(2)
            self.initialied = True

        else:
            self.show_items()
            choose_ingredient_to_gather = ChooseIngredientToGather(agent_name="agentA1")
            choose_best_assembly_combination = ChooseBestAssemblyCombination()
            items = choose_best_assembly_combination.finished_items_priority()
            ettilog.logerr(items)
            rospy.signal_shutdown("test over")


    def show_items(self):
        ettilog.logerr("-----------------------------------")
        ettilog.logerr("Ingredients:")
        stock = DebugUtils.show_total_stock_with_goals()
        for i in range(0, 5):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])
        ettilog.logerr("Finished products:")
        for i in range(5, 11):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d", item, stock[item]["stock"], stock[item]["goal"])
if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = TestAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
