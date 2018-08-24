import itertools
import numpy as np
import re

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

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_combination')


class BestAgentAssemblyCombinationDecision(object):
    """
    Selects the combination of agents that together can perform the most valuable assembly tasks
    """
    WEIGHT_AGENT_BID = 4
    MIN_ITEMS = 3
    PREFERRED_AGENT_COUNT = 4
    PRIORITY_EXPONENT = 1  # [ .05 - 10]
    WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT = -2.1
    WEIGHT_PRIORITY = 100
    WEIGHT_NUMBER_OF_AGENTS = -4
    WEIGHT_PRIORITISATION_ACTIVATION = 30
    WEIGHT_IDLE_STEPS = -1
    WEIGHT_MAX_STEP_COUNT = -2

    MAX_AGENTS = 7
    MIN_AGENTS = 1
    MAX_STEPS = 20

    ACTIVATION_THRESHOLD = -5000

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
        self._product_provider = ProductProvider(agent_name=agent_name)  # TODO: make independent from agent

        rospy.Subscriber(ChooseBestAvailableJobDecision.TOPIC_FINISHED_PRODUCT_GOAL, StockItem, queue_size=10,
                         callback=self._planner_goal_callback)
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):
        BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT",
            BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT)
        BestAgentAssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT",
            BestAgentAssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT)
        BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY",
            BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY)
        BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS",
            BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS)
        BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION",
            BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION)
        BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS",
            BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS)
        BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT",
            BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT)
        BestAgentAssemblyCombinationDecision.MAX_AGENTS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS", BestAgentAssemblyCombinationDecision.MAX_AGENTS)
        BestAgentAssemblyCombinationDecision.MIN_AGENTS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS", BestAgentAssemblyCombinationDecision.MIN_AGENTS)
        BestAgentAssemblyCombinationDecision.MAX_STEPS = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.MAX_STEPS", BestAgentAssemblyCombinationDecision.MAX_STEPS)
        BestAgentAssemblyCombinationDecision.PREFERRED_AGENT_COUNT = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.PREFERRED_AGENT_COUNT",
            BestAgentAssemblyCombinationDecision.PREFERRED_AGENT_COUNT)

        BestAgentAssemblyCombinationDecision.ACTIVATION_THRESHOLD = rospy.get_param(
            "BestAgentAssemblyCombinationDecision.ACTIVATION_THRESHOLD",
            BestAgentAssemblyCombinationDecision.ACTIVATION_THRESHOLD)

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

    def choose_best_combinations(self, bids):
        """
        Returns a sorted list of combinations of bids, that together can create products that we currently need.
        Sorting according to activation
        :param bids:
        :type bids: TaskBid[]
        :return:
        """
        bids.sort(key=lambda bid: bid.bid)

        if self.finished_products is None:
            self.init_finished_products()

        finished_item_priority = self.finished_items_priority_dict()

        best_combination_activation = -np.inf
        best_combination = None

        # Create numpy arrays of items of each bid for faster calculation
        bid_with_array = []
        for bid in bids:
            bid_array = self.bid_to_numpy_array(bid)
            bid_with_array.append((bid, bid_array))

        start_time = rospy.get_rostime()

        # If min number of agents bid for assembly try all combinations
        if len(bid_with_array) >= BestAgentAssemblyCombinationDecision.MIN_AGENTS:
            # Go through all combinations

            # try all combinations using MIN_AGENTS to MAX_AGENTS agents.
            for number_of_agents in range(BestAgentAssemblyCombinationDecision.MIN_AGENTS, min(len(bid_with_array) + 1,
                                                                                               BestAgentAssemblyCombinationDecision.MAX_AGENTS)):
                # For each subset of bids find the best possible combination
                for subset in itertools.combinations(bid_with_array, number_of_agents):

                    # Generate an numpy array combining all items of the current subset of bids
                    array_bid_subset = np.zeros(len(self.products) + len(self._roles))
                    bid_subset = []
                    for bid, array_bid in subset:
                        bid_subset.append(bid)
                        array_bid_subset += array_bid

                    # Get the best combination to build with the current subset
                    combination = self.try_build_item(array_bid_subset, priorities=finished_item_priority)

                    # If no agent has space for many more items, the combination is desperate
                    desperate = sum([bid.bid < -8 for bid in bid_subset]) <= 1

                    if len(combination) >= BestAgentAssemblyCombinationDecision.MIN_ITEMS or desperate:

                        # The number of step until all agents can be at the closest workshop
                        pos_speed_role = [(bid.pos, bid.speed, bid.role) for bid in bid_subset]
                        workshops = self._facility_provider.workshops.values()
                        max_step_count, destination = self.distance_provider.get_steps_to_closest_facility(pos_speed_role,
                                                                                                           workshops)

                        # If max nr of steps is too high, ignore combination
                        if max_step_count > BestAgentAssemblyCombinationDecision.MAX_STEPS and not desperate:
                            continue

                        # Number of steps agents will have to wait at storage until the last agent arrives
                        idle_steps = sum([max_step_count - bid.expected_steps for bid in bid_subset])

                        prioritisation_activation = self.item_list_activation(combination,
                                                                              priorities=finished_item_priority)
                        # If prioritisation is negative -> do not assemble
                        if prioritisation_activation <= 0:
                            continue

                        # Number of steps agents will have to wait at storage until the last agent arrives
                        agent_bid =  sum([bid.bid for bid in bid_subset])

                        value = self.rate_combination(max_step_count, idle_steps, prioritisation_activation,
                                                      number_of_agents, agent_bid)

                        if value > best_combination_activation:
                            # TODO: Timeout
                            assembly_instructions = self._assembly_agent_chooser.generate_assembly_instructions(bid_subset, combination)
                            rospy.logerr("assembly instructions: %s", str(assembly_instructions))
                            if assembly_instructions is not None:
                                best_combination_activation = value
                                best_combination = (bid_subset, combination, value, destination, assembly_instructions)

                    time_passed = (rospy.get_rostime() - start_time).to_sec()
        if best_combination is not None:
            bid_subset, combination, value, destination, assembly_instructions = best_combination
            items = {}
            roles = []
            for bid in bid_subset:
                items = CalcUtil.dict_sum(items, CalcUtil.dict_from_string_list(bid.items))
                roles += bid.role


        return best_combination

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

    def try_build_item(self, products, priorities, item=None):
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
            try_items = self.finished_item_list[self.finished_item_list.index(item):]
        else:
            try_items = self.finished_item_list

        best_item_list = None
        for item in try_items:
            # Start the same function iteratively and accept the best result.
            item_list = self.try_build_item(products, priorities, item)
            if item_list:
                value = self.item_list_activation(item_list, priorities)

                # Find the best combination and return it then
                if value > best_value:
                    best_value = value
                    best_item_list = item_list

        if best_item_list is not None:
            res = res + best_item_list

        return res

    def finished_items_priority_dict(self):
        """
        Returns a dictionary with priorities from 0 to 1 of how much each finished product is needed.
        :return:
        """
        res = {}
        # Get expected stock items of all agents
        finished_stock_items = self._product_provider.get_agent_stock_items(
            types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)

        for key in self._product_provider._assemblable_items.keys():
            # Priority value
            # priority = 1.0 - (float(finished_stock_items.get(key, 0)) / self._finished_product_goals.get(key,
            #                                                                                              ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL))
            # priority can not be lower than 0 -> this would mean we want to destroy finished products
            # priority = max(priority, 0.0) **
            priority = self._finished_product_goals.get(key, ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL) - finished_stock_items.get(key, 0)
            res[key] = priority ** BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT  # Square it to really emphasize the importance of items with few
        return res

    def rate_combination(self, max_step_count, idle_steps, prioritisation_activation, number_of_agents, agent_bid):
        """
        Returns an activation value rating a combination of items to assemble
        :param max_step_count:
        :param idle_steps:
        :param prioritisation_activation:
        :param number_of_agents:
        :return:
        """
        return max_step_count * BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT + \
               idle_steps * BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS + \
               prioritisation_activation * BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION + \
               number_of_agents * BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS + \
               agent_bid *  BestAgentAssemblyCombinationDecision.WEIGHT_AGENT_BID

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
            occurances[item] = occurances.get(item, -1) + 1
            # Take negative priorities and treat them like 0
            res += max((priorities[item] -1), 0) * BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY

        return res

    @property
    def finished_product_goals(self):
        return self._finished_product_goals
