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

        agent_name = "agentA1"
        
        self.product_provider = ProductProvider(agent_name=agent_name)
        self.facility_provider = FacilityProvider()
        self.sensor_map = SensorAndConditionMap(agent_name=agent_name)
        time.sleep(1)

        self.choose_best_assembly_combination= AssemblyCombinationDecision(agent_name=agent_name)
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

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
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
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("  ")
        rospy.logerr("-----------------------------------")
        rospy.logerr("Ingredients:")

        as_ = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_STOCK])
        ag_ = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL])
        aa_ = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL])
        ah_ = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL])
        ad_ = self.product_provider.total_items_in_stock(types=[ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL])

        ss_ = self.product_provider.get_stored_items(include_job_goals=False, include_stock=True, include_hoarding_goal=False)
        sd_ = self.product_provider.get_stored_items(include_job_goals=True, include_stock=False, include_hoarding_goal=True)
        sh_ = self.product_provider.get_stored_items(include_job_goals=False, include_stock=False, include_hoarding_goal=False)

        gip_ = self.sensor_map.gather_decision_mechanism.ingredient_priority_dict()
        gi_ = self.sensor_map.gather_decision_mechanism.get_desired_ingredients(consider_intermediate_ingredients=True)
        gfp_ = self.choose_best_assembly_combination.finished_items_priority_dict()
        gf_ = self.choose_best_assembly_combination.get_desired_finished_products()


        for i in range(0, 11):
            item = "item" + str(i)
            rospy.logerr("%7s: agents:[s:%-3s g:%-3s a:%-3s h:%-3s d:%-3s] storage:[s:%-3s d:%-3s h:%-3s] goal:[i:%2.2f f:%2.2f ip:%2.2f fp:%2.2f]",
                            item,

                           as_.get(item,0),
                           ag_.get(item,0),
                           aa_.get(item,0),
                           ah_.get(item,0),
                           ad_.get(item,0),

                           ss_.get(item,0),
                           sd_.get(item,0),
                           sh_.get(item,0),

                           gi_.get(item,0),
                           gf_.get(item,0),
                           gip_.get(item,0),
                           gfp_.get(item,0)
                         )
            if i ==4:
                rospy.logerr("Finished products:")

if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = DebugAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
