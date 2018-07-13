import time

import rospy
from mac_ros_bridge.msg import RequestAction

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from decisions.assembly_combination import AssemblyCombinationDecision
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from sensor.sensor_map import SensorAndConditionMap

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class DebugAgent(object):

    def __init__(self):
        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self.product_provider = ProductProvider(agent_name="agentA1")
        self.facility_provider = FacilityProvider()
        self.sensor_map = SensorAndConditionMap(agent_name="agentA1")
        time.sleep(1)

        self.choose_best_assembly_combination= AssemblyCombinationDecision()
        # DebugUtils.instant_find_resources(ResourceBaseKnowledgeBase())
        # time.sleep(1)
        # rospy.signal_shutdown("end of debug code")
        # DebugUtils.add_build_well_task()
        # DebugUtils.assign_assembly_task()
        # DebugUtils.add_build_well_task(agent_name="agentA1")
        # DebugUtils.add_build_well_task(agent_name="agentA2")
        # DebugUtils.add_build_well_task(agent_name="agentA3")
        # DebugUtils.add_build_well_task(agent_name="agentA4")
        # DebugUtils.add_build_well_task(agent_name="agentA5")
        # DebugUtils.add_build_well_task(agent_name="agentA6")

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

        ingredient_priority = self.sensor_map.gather_decision_mechanism.ingredient_priority_dict()
        desired_ingredients = self.sensor_map.gather_decision_mechanism.get_desired_ingredients(consider_intermediate_ingredients=True)
        finished_product_priority = self.choose_best_assembly_combination.finished_items_priority_dict()
        desired_finished_products = self.choose_best_assembly_combination.finished_product_goals

        stock_amount = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_STOCK])
        stock_goals = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
                                                            ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL,
                                                                        ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
                                                                        ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL])
        storage_stock = self.product_provider.get_stored_items()
        for i in range(0, 5):
            item = "item" + str(i)
            ettilog.logerr("%s: %d/%d  - stored: %d ingredient_prio: %.2f finished_product_prio: %.2f, desired i: %d dfp: %f",
                               item,
                           stock_amount.get(item,0),
                           stock_goals.get(item,0),
                           storage_stock.get(item, 0),
                           ingredient_priority.get(item, 0),
                           finished_product_priority.get(item, 0),
                           desired_ingredients.get(item, 0),
                           desired_finished_products.get(item, 0))

        ettilog.logerr("Finished products:")
        for i in range(5, 11):
            item = "item" + str(i)
            ettilog.logerr("%s: %d/%d  - stored: %d ingredient_prio: %.2f finished_product_prio: %.2f, desired i: %d dfp: %f",
                               item,
                           stock_amount.get(item,0),
                           stock_goals.get(item,0),
                           storage_stock.get(item, 0),
                           ingredient_priority.get(item, 0),
                           finished_product_priority.get(item, 0),
                           desired_ingredients.get(item, 0),
                           desired_finished_products.get(item, 0))

if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = DebugAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
