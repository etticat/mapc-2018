import time

import rospy
from mac_ros_bridge.msg import RequestAction

from agent_knowledge.resource import ResourceBaseKnowledgeBase
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from decisions.assembly_combination import ChooseBestAssemblyCombination
from decisions.gathering import ChooseIngredientToGather
from provider.facility_provider import FacilityProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class DebugAgent(object):

    def __init__(self):

        self.facility_provider = FacilityProvider()
        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)
        time.sleep(1)

        self.choose_ingredient_to_gather = ChooseIngredientToGather(agent_name="agentA1")
        self.choose_best_assembly_combination= ChooseBestAssemblyCombination()
        DebugUtils.instant_find_resources(ResourceBaseKnowledgeBase())
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
        self.show_items()

    def show_items(self):
        ettilog.logerr("-----------------------------------")
        ettilog.logerr("Ingredients:")

        ingredient_priority = self.choose_ingredient_to_gather.ingredient_priority_dict()
        desired_ingredients = self.choose_ingredient_to_gather.get_desired_ingredients(consider_intermediate_ingredients=True)
        finished_product_priority = self.choose_best_assembly_combination.finished_items_priority_dict()
        stock = DebugUtils.show_total_stock_with_goals()
        storage_stock = self.facility_provider.get_all_stored_items()
        for i in range(0, 5):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d  - stored: %d ingredient_prio: %d finished_product_prio: %d, desired ingredients: %d", item, stock[item]["stock"], stock[item]["goal"], storage_stock.get(item, 0), ingredient_priority.get(item, 0), finished_product_priority.get(item, 0), desired_ingredients.get(item, 0))

        ettilog.logerr("Finished products:")
        for i in range(5, 11):
            item = "item" + str(i)
            if item in stock.keys():
                ettilog.logerr("%s: %d/%d - stored: %d  ingredient_prio: %d finished_product_prio: %d, desired ingredients: %d", item, stock[item]["stock"], stock[item]["goal"], storage_stock.get(item, 0), ingredient_priority.get(item, 0), finished_product_priority.get(item, 0), desired_ingredients.get(item, 0))


if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = DebugAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
