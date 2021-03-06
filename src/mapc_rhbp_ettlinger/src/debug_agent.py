#!/usr/bin/env python2

import time

import rospy
from mac_ros_bridge.msg import RequestAction

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.best_agent_assembly_combination import BestAgentAssemblyCombinationDecision
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from agent_components.shared_components import SharedComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class DebugAgent(object):
    """
    Agent that allows to execute debugging code. By default it keeps track of all items (agent stock, storage and goals)
    Additionally it allows to execute debugging code from DebugUtil e.g. find all Resources automatically
    """

    def __init__(self):
        rospy.init_node('debug_node', anonymous=True, log_level=rospy.ERROR)

        agent_name = rospy.get_param('~agent_name', "agentA1")

        # Init providers
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self.shared_components = SharedComponents(agent_name=agent_name)

        # Sleep just to make sure everything is initialised. DebugAgent does not use any components, so without this
        # It may lead to errors.
        time.sleep(1)

        self.choose_best_assembly_combination = BestAgentAssemblyCombinationDecision(agent_name=agent_name)

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="request_action"), RequestAction,
                         self._callback_action_request)

    def _callback_action_request(self, msg):
        """
        Callback for action_request
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        rospy.logerr("-------------------- Simulation step: %s --------------------", msg.simulation_step)
        self.show_items()

    def show_items(self):
        """
        prints a list of all items (agent stock, storage and goals)
        :return:
        """
        rospy.logerr("Ingredients:")

        as_ = self._product_provider.get_agent_stock_items(types=[ProductProvider.STOCK_ITEM_TYPE_STOCK])
        ag_ = self._product_provider.get_agent_stock_items(types=[ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL])
        aa_ = self._product_provider.get_agent_stock_items(types=[ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL])
        ah_ = self._product_provider.get_agent_stock_items(types=[ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL])
        ad_ = self._product_provider.get_agent_stock_items(types=[ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL])

        ss_ = self._product_provider.get_stored_items(include_job_goals=False, include_stock=True,
                                                     include_hoarding_goal=False)
        sd_ = self._product_provider.get_stored_items(include_job_goals=True, include_stock=False,
                                                     include_hoarding_goal=True)
        sh_ = self._product_provider.get_stored_items(include_job_goals=False, include_stock=False,
                                                     include_hoarding_goal=False)

        gip_ = self.shared_components.gather_decision_decision.ingredient_priority_dict()
        gi_ = self.shared_components.gather_decision_decision.get_desired_ingredients(
            consider_intermediate_ingredients=True)
        gfp_ = self.choose_best_assembly_combination.finished_items_priority_dict()
        gf_ = self.choose_best_assembly_combination.finished_product_goals

        printed_fp = False

        for i in range(0, len(self._product_provider.product_infos.keys())):
            item = "item" + str(i)
            product = self._product_provider.product_infos[item]
            if len(product.consumed_items) > 0 and not printed_fp:
                printed_fp = True
                rospy.logerr("Finished products:")
            rospy.logerr(
                "%7s: agents:[s:%-3s g:%-3s a:%-3s h:%-3s d:%-3s] storage:[s:%-3s d:%-3s h:%-3s] goal:[i:%2.2f "
                "f:%2.2f ip:%2.2f fp:%2.2f]",
                item, as_.get(item, 0), ag_.get(item, 0), aa_.get(item, 0), ah_.get(item, 0), ad_.get(item, 0),
                ss_.get(item, 0), sd_.get(item, 0), sh_.get(item, 0), gi_.get(item, 0), gf_.get(item, 0),
                gip_.get(item, 0), gfp_.get(item, 0))


if __name__ == '__main__':

    try:
        job_planner = DebugAgent()
        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
