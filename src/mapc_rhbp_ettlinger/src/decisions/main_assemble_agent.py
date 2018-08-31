
import copy

import rospy
from mac_ros_bridge.msg import SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decision.assembly_agent_decision')


class MainAssembleAgentDecision(object):
    """
    Class that is responsible for taking assembly bids and deciding the best combination of agents to start assembly
    """

    WEIGHT_FINISHED_PRODUCT_FACTOR = 1
    WEIGHT_BID_SKILL = -100
    WEIGHT_BID_SPEED = 100

    def __init__(self, agent_name):
        self._product_provider = ProductProvider(agent_name=agent_name)

        self._init_config()
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):
        MainAssembleAgentDecision.WEIGHT_FINISHED_PRODUCT_FACTOR = rospy.get_param(
            "MainAssembleAgentDecision.WEIGHT_FINISHED_PRODUCT_FACTOR",
            MainAssembleAgentDecision.WEIGHT_FINISHED_PRODUCT_FACTOR)
        MainAssembleAgentDecision.WEIGHT_BID_SKILL = rospy.get_param("MainAssembleAgentDecision.WEIGHT_BID_SKILL",
                                                                     MainAssembleAgentDecision.WEIGHT_BID_SKILL)
        MainAssembleAgentDecision.WEIGHT_BID_SPEED = rospy.get_param("MainAssembleAgentDecision.WEIGHT_BID_SPEED",
                                                                     MainAssembleAgentDecision.WEIGHT_BID_SPEED)

    def generate_assembly_instructions(self, accepted_bids, finished_products):
        res = {}

        # Track how many items we are successfully distributing
        items_to_assemble = 0

        # We use this array to calculate the estimated stock of items after the assembly which changes the bids.
        # Therefore we need a copy of the item array
        prioritized_bids = copy.deepcopy(accepted_bids)
        bids_sorted_lexicographically = []

        for bid in prioritized_bids:
            res[bid.agent_name] = ""
            bids_sorted_lexicographically.append(bid)

        # Sort the agents by name because the first one coming in this list are preferred to give items
        bids_sorted_lexicographically.sort(key=lambda bid: CalcUtil.natural_keys(bid.agent_name))

        # Sort accepted_bids, so that the agents who should be preferred are in front of the list
        # We prefer agents that
        #  * are then likely to be able to move to the delivery destination afterwards fast (speed)
        #  * have low skill -> We should rather use those to gather/build well/dismantle...
        #  * already have many finished_products -> So not everyone has to go to storage afterwards
        prioritized_bids.sort(key=lambda
            bid: bid.skill * MainAssembleAgentDecision.WEIGHT_BID_SKILL + bid.speed * MainAssembleAgentDecision.WEIGHT_BID_SPEED +
                 bid.finished_product_factor * MainAssembleAgentDecision.WEIGHT_FINISHED_PRODUCT_FACTOR, reverse=True)

        # From this sorted list pick the ones that have the capacity to hold the assembled item
        for item in finished_products:

            selected_agent = None

            items_needed = CalcUtil.get_string_list_from_dict(self._product_provider.get_ingredients_of_product(item))

            # For each agent in prioritised list, try if they can assemble it by checkign
            # a) if the finished product fits in stock while subtracting the
            # b) the items that they will use for assembly
            for bid in prioritized_bids:
                # Still available capacity of agent
                capacity_after_assembly = bid.capacity
                # - minus the capacity needed for the finished item
                capacity_after_assembly -= self._product_provider.get_volume_of_item(item)
                # - plus the capacity that gets free through assembly
                used_items = CalcUtil.list_intersect(bid.items, items_needed)
                capacity_after_assembly += self._product_provider.calculate_total_volume_list(used_items)

                # If capacity after assembly is greater/equal 0 -> assembly possible -> use this agent for assembly
                if capacity_after_assembly >= 0:
                    # Change bid items to reflect assembly
                    bid.items = CalcUtil.list_diff(bid.items, used_items)
                    bid.items += [item]  # This is important as we may need the item for further assembly in same turn
                    bid.capacity = capacity_after_assembly

                    items_needed = CalcUtil.list_diff(items_needed, used_items)

                    selected_agent = bid.agent_name
                    break

            if selected_agent is None:
                # No agent has the capacity to assemble the item -> continue with next
                ettilog.loginfo("ChooseAgentsToAssemble:: No agent has the capacity to assemble the item -> continue with next")
                continue

            items_to_assemble += 1

            # Assembly items are taken from agents in lexicographical order
            for bid in bids_sorted_lexicographically:
                agent = bid.agent_name
                if res[agent] is not "":
                    res[agent] = res[agent] + ","

                if agent == selected_agent:
                    res[agent] = res[agent] + "assemble:" + item
                    # Don't need to calculate anything here as we have done it already above.
                    # This can't be moved in here as assemble agent always has a higher priority to give items than
                    # lexographically sorted assist agents
                else:
                    res[agent] = res[agent] + "assist:" + selected_agent

                    # Change bid items to reflect assembly
                    used_items = CalcUtil.list_intersect(bid.items, items_needed)
                    bid.items = CalcUtil.list_diff(bid.items, used_items)

                    # Adjust still needed items -> for next round
                    items_needed = CalcUtil.list_diff(items_needed, used_items)

            assert len(items_needed) == 0

        # If we were not able to distribute any assembly instructions at all, return None (e.g. No one has the capacity)
        if items_to_assemble is 0:
            return None

        return res
