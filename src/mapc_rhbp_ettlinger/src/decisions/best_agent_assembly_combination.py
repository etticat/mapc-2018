import itertools
import json
import math
import numpy as np
import re
import time

from mac_ros_bridge.msg import SimStart
from mapc_rhbp_ettlinger.msg import StockItem

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.choose_best_available_job import ChooseBestAvailableJobDecision
from decisions.main_assemble_agent import MainAssembleAgentDecision
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
import cProfile as profile

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_combination')


class BestAgentAssemblyCombinationDecision(object):
    """
    Selects the combination of agents that together can perform the most valuable assembly tasks
    """
    WEIGHT_NUMBER_OF_AGENTS = -0.2
    MAX_NR_OF_AGENTS_TO_CONSIDER = 17
    MAX_PRIORITY_NOT_NEEDED_ITEMS = 0.30
    MAX_COUNT_NOT_NEEDED_ITEMS = 7
    WEIGHT_MISSION_JOB_PRIORITY = 2.0

    MAX_AGENTS = 7
    MIN_AGENTS = 3
    MAX_STEPS = 20

    def __init__(self, agent_name):
        self._assembly_agent_chooser = MainAssembleAgentDecision(agent_name=agent_name)
        self.distance_provider = DistanceProvider(agent_name=agent_name)
        self._init_config()

        self.finished_products = None
        self.products = None
        self.finished_item_list = None
        self._finished_product_goals = {}
        self._roles = ["car", "motorcycle", "drone", "truck"]

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)

        rospy.Subscriber(ChooseBestAvailableJobDecision.TOPIC_FINISHED_PRODUCT_GOAL, StockItem, queue_size=10,
                         callback=self._planner_goal_callback)
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self.reset)

    def reset(self, sim_start=None):

        self.finished_products = None
        self.products = None
        self.finished_item_list = None
        self._finished_product_goals = {}
        self._init_config()

    def _init_config(self):
        BestAgentAssemblyCombinationDecision.MAX_AGENTS = int(round(rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS", BestAgentAssemblyCombinationDecision.MAX_AGENTS)))
        BestAgentAssemblyCombinationDecision.MIN_AGENTS = int(round(rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS", BestAgentAssemblyCombinationDecision.MIN_AGENTS)))
        BestAgentAssemblyCombinationDecision.MAX_STEPS = int(round(rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_STEPS", BestAgentAssemblyCombinationDecision.MAX_STEPS)))
        BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER = int(round(rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER",
            BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER)))
        BestAgentAssemblyCombinationDecision.MAX_COUNT_NOT_NEEDED_ITEMS = int(round(rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_COUNT_NOT_NEEDED_ITEMS",
            BestAgentAssemblyCombinationDecision.MAX_COUNT_NOT_NEEDED_ITEMS)))
        BestAgentAssemblyCombinationDecision.MAX_PRIORITY_NOT_NEEDED_ITEMS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_PRIORITY_NOT_NEEDED_ITEMS",
            BestAgentAssemblyCombinationDecision.MAX_PRIORITY_NOT_NEEDED_ITEMS)
        BestAgentAssemblyCombinationDecision.WEIGHT_MISSION_JOB_PRIORITY = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_MISSION_JOB_PRIORITY",
            BestAgentAssemblyCombinationDecision.WEIGHT_MISSION_JOB_PRIORITY)

    def _planner_goal_callback(self, stock_item):
        """
        Save the finished product goals from planner
        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        self._finished_product_goals = {}
        for goal in stock_item.amounts:
            self._finished_product_goals[goal.key] = goal.value

    def best_combination(self, bids):
        """
        Returns a sorted list of combinations of bids, that together can create products that we currently need.
        Sorting according to activation
        :param bids:
        :type bids: TaskBid[]
        :return:
        """
        if self.finished_products is None:
            self.init_finished_products()

        finished_item_priority = self.finished_items_priority_dict()

        workshop_distances, best_facility = self.find_best_workshop(bids)

        bids.sort(key=lambda bid: workshop_distances[best_facility][bid])

        # Create numpy arrays of items of each bid for faster calculation
        bid_with_array = []
        for bid in bids[:BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER]:
            if bid.expected_steps <= BestAgentAssemblyCombinationDecision.MAX_STEPS:
                bid_array = self.bid_to_numpy_array(bid)
                bid_with_array.append((bid, bid_array))

        best_combination = self.find_best_combination(best_facility, bid_with_array, finished_item_priority)


        return best_combination

    def find_best_combination(self, best_facility, bid_with_array, finished_item_priority):
        start_time = rospy.get_rostime()
        time_passed = 0
        best_combination_activation = 0
        best_combination = None
        # If min number of agents bid for assembly try all combinations
        if len(bid_with_array) >= BestAgentAssemblyCombinationDecision.MIN_AGENTS:
            # Go through all combinations

            # try all combinations using MIN_AGENTS to MAX_AGENTS agents.
            for number_of_agents in range(BestAgentAssemblyCombinationDecision.MIN_AGENTS, min(len(bid_with_array) + 1,
                                                                                               BestAgentAssemblyCombinationDecision.MAX_AGENTS)):

                rospy.loginfo("BestAgentAssemblyCombinationDecision:: Trying with %d agents", number_of_agents)

                # For each subset of bids find the best possible combination
                for subset in itertools.combinations(bid_with_array, number_of_agents):

                    # Generate an numpy array combining all items of the current subset of bids
                    array_bid_subset, bid_subset = self.extrac_bids_and_bid_arrays(subset)

                    finished_item_list = self.get_possible_finished_items(array_bid_subset)

                    if len(finished_item_list) <= 0:
                        continue

                    # Get the best combination to build with the current subset
                    combination = self.try_build_item(array_bid_subset, priorities=finished_item_priority,
                                                      finished_item_list=finished_item_list)


                    # The number of step until all agents can be at the closest workshop
                    destination = best_facility

                    activation = self.item_list_activation(combination, priorities=finished_item_priority)

                    activation += number_of_agents * BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS

                    if activation > best_combination_activation:
                        assembly_instructions = self._assembly_agent_chooser.generate_assembly_instructions(
                            bid_subset, combination)
                        if assembly_instructions is not None:
                            best_combination_activation = activation
                            best_combination = (bid_subset, combination, activation, destination, assembly_instructions)

                    time_passed = (rospy.get_rostime() - start_time).to_sec()
        ettilog.logerr("BestAgentAssemblyCombinationDecision:: Time to decide: %.2fs Combination found: %s bids: %d",
                       time_passed, str(best_combination is not None), len(bid_with_array))
        return best_combination

    def extrac_bids_and_bid_arrays(self, subset):
        array_bid_subset = np.zeros(len(self.products) + len(self._roles))
        bid_subset = []
        for bid, array_bid in subset:
            bid_subset.append(bid)
            array_bid_subset += array_bid
        return array_bid_subset, bid_subset

    def get_possible_finished_items(self, array_bid_subset):
        finished_item_list = []
        for finished_item in self.finished_item_list:
            products = array_bid_subset - self.finished_products[finished_item]

            if min(products) >= 0:
                finished_item_list.append(finished_item)
        return finished_item_list

    def find_best_workshop(self, bids):
        workshop_distances = {}
        workshops = self._facility_provider.workshops.values()
        best_facility = None
        best_facility_value = np.inf
        for facility in workshops:
            workshop_distances[facility] = {}
            for bid in bids:
                workshop_distances[facility][bid] = self.distance_provider.calculate_steps(end_position=facility.pos,
                                                                                           use_in_facility_flag=False,
                                                                                           start_position=bid.pos,
                                                                                           can_fly=bid.role == "drone",
                                                                                           estimate=True,
                                                                                           speed=bid.speed)

            workshop_distance_values = workshop_distances[facility].values()
            workshop_distance_values.sort()

            workshop_value = sum(
                workshop_distance_values[:BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER])
            if workshop_value < best_facility_value:
                best_facility_value = workshop_value
                best_facility = facility
        return workshop_distances, best_facility

    def init_finished_products(self):
        """
        Creates numpy arrays describing the change in item stock for an assembly action, which is later used for faster
        calculation. It also contains values for the required roles.
        e.g. there are 4 items and 2 roles. item 3 requires 2x item0 and 1x item 2 and the second role, the result
        is dict = { "item3" : [2, 0, 1, -1, 0, 1]
        :return:
        """
        finished_products = self._product_provider.assemblable_items
        self.finished_item_list = finished_products.keys()

        # Get a finished item list sorted by the number
        self.finished_item_list.sort(key=CalcUtil.natural_keys)
        self.finished_products = {}

        # Get an item list sorted by the number
        self.products = self._product_provider.product_infos.keys()
        self.products.sort(key=CalcUtil.natural_keys)

        for finished_product in finished_products.values():
            self.finished_products[finished_product.name] = np.zeros(len(self.products) + len(self._roles))

            for item in finished_product.consumed_items:
                index = self.products.index(item.name)
                self.finished_products[finished_product.name][index] += item.amount

            for role in finished_product.required_roles:
                self.finished_products[finished_product.name][len(self.products) + self._roles.index(role)] = 1

            self.finished_products[finished_product.name][self.products.index(finished_product.name)] += -1

    def try_build_item(self, products, priorities, finished_item_list, item=None):
        """
        Finds the best item combination by trying to build all possible finished products and iteratively invoking
        the next round. This has the benefit that it allows building items, that require other assembled items first
        :param products:
        :param priorities:
        :param item: the item to build. If none provided, build all items and select the best.
        :return:
        """
        res = []
        best_value = -np.inf

        # if i
        if item is not None:
            # apply the assembly numpy array for the item to build
            products = products - self.finished_products[item]

            if min(products) < 0:
                # if the assembly results in a negative value, this means not all items are available
                return None

            res = [item]
            try_items = finished_item_list[finished_item_list.index(item):]
        else:
            try_items = finished_item_list

        best_item_list = None
        for item in try_items:
            # Start the same function iteratively and accept the best result.
            item_list = self.try_build_item(products, priorities, item=item, finished_item_list=finished_item_list)
            if item_list:
                value = self.item_list_activation(item_list, priorities)

                # Find the best combination and return it then
                if value > best_value:
                    best_value = value
                    best_item_list = item_list

        if best_item_list is not None:
            res = res + best_item_list

        return res

    def finished_items_priority_dict(self, normalize_to_zero=False):
        """
        Returns a dictionary with priorities from 0 to 1 of how much each finished product is needed.
        :return:
        """
        res = {}
        # Get expected stock items of all agents
        finished_stock_items = self._product_provider.get_agent_stock_items(
            types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)

        intermediary_finished_product_goal= {}

        assemblable_items_keys = self._product_provider._assemblable_items.keys()
        assemblable_items_keys.sort(reverse=True, key=CalcUtil.natural_keys)
        for key in assemblable_items_keys:
            priority = self._finished_product_goals.get(key, 0) - finished_stock_items.get(key, 0) + intermediary_finished_product_goal.get(key, 0)

            if normalize_to_zero and priority < 0:
                priority = 0

            for consumed_item in self._product_provider.get_product_by_name(key).consumed_items:
                intermediary_finished_product_goal[consumed_item.name] = intermediary_finished_product_goal.get(consumed_item.name, 0) + max(priority, 0) * consumed_item.amount

            res[key] = priority
        return res

    def bid_to_numpy_array(self, bid):
        """
        creates a numpy array out of a bid
        The first n-4 values descirbe thenumber of items an agent holds.
        The last 4 values match the roles. They are 999 at the index of the current role and 0 otherwise.
        :param bid:
        :type bid: TaskBid
        :return:
        """
        bid_array = np.zeros(len(self.products) + len(self._roles))

        for item in bid.items:
            bid_array[self.products.index(item)] += 1

        bid_array[len(self.products) + self._roles.index(bid.role)] = 999

        return bid_array

    def item_list_activation(self, item_list, priorities):
        """
        Returns the activation of assemblig a given list of items
        :param item_list:
        :param priorities:
        :return:
        """

        res = 0

        # Keep track of occurances of each item in the assembly list to assign proper priorities
        occurances = {}

        for item in item_list:
            # Take negative priorities and treat them like 0
            priority = priorities[item] - occurances.get(item, 0)
            if priority > 0:
                res += priorities[item] * BestAgentAssemblyCombinationDecision.WEIGHT_MISSION_JOB_PRIORITY
            else:
                res += (priority * BestAgentAssemblyCombinationDecision.MAX_PRIORITY_NOT_NEEDED_ITEMS / (BestAgentAssemblyCombinationDecision.MAX_COUNT_NOT_NEEDED_ITEMS)) + 0.30

            occurances[item] = occurances.get(item, 0) + 1
        return res

    @property
    def finished_product_goals(self):
        return self._finished_product_goals
