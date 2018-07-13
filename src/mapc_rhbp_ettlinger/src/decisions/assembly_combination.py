import itertools
import numpy as np
import re

from mapc_rhbp_ettlinger.msg import StockItem

import rospy
from common_utils import etti_logging
from decisions.job_activation import JobDecider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_combination')

class AssemblyCombinationDecision(object):

    WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT = -2.1
    WEIGHT_PRIORITY = 10
    WEIGHT_NUMBER_OF_AGENTS = -10
    WEIGHT_PRIORITISATION_ACTIVATION = 30
    WEIGHT_IDLE_STEPS = -3
    WEIGHT_MAX_STEP_COUNT = -10

    MAX_AGENTS = 7
    MIN_AGENTS = 1

    ACTIVATION_THRESHOLD = -100

    def __init__(self):

        self.facility_provider = FacilityProvider()
        self._product_provider = ProductProvider(agent_name="agentA1") # TODO: make independent from agent


        self.roles = ["car", "motorcycle", "drone", "truck"]
        self.finished_products = None
        self.finished_item_list = None

        self.finished_product_goals = {}

        rospy.Subscriber(JobDecider.TOPIC_FINISHED_PRODUCT_GOAL, StockItem, queue_size=10, callback=self._planner_goal_callback)

    def _planner_goal_callback(self, stock_item):
        """

        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        self.finished_product_goals = {}
        for goal in stock_item.amounts:
            self.finished_product_goals[goal.key] = goal.value


    def choose(self, bids):
        """

        :param bids:
        :type bids: TaskBid[]
        :return:
        """
        if self.finished_products == None:
            self.init_finished_products()


        best_combination = []
        best_value = -np.inf
        best_finished_products = None

        priorities = self.finished_items_priority_dict()

        total_bid_array = np.zeros(len(self.products) + len(self.roles))
        bid_with_array = []
        for bid in bids:
            bid_array = self.bid_to_array(bid)
            bid_with_array.append((bid, bid_array))

            total_bid_array += bid_array

        if len(bid_with_array) >= AssemblyCombinationDecision.MIN_AGENTS:
            # Go through all combinations
            for number_of_agents in range(AssemblyCombinationDecision.MIN_AGENTS, min(len(bid_with_array) + 1, AssemblyCombinationDecision.MAX_AGENTS)): # We try all combinations using 2-7 agents
                for subset in itertools.combinations(bid_with_array, number_of_agents):
                    array_bid_subset = np.zeros(len(self.products) + len(self.roles))
                    bid_subset = []

                    for bid, array_bid in subset:
                        bid_subset.append(bid)
                        array_bid_subset += array_bid

                    prioritisation_activation, combination = self.try_build_item(array_bid_subset, priorities=priorities)
                    prioritisation_activation -= sum(array_bid_subset[0:-len(self.roles)]) * AssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT

                    # The number of step until all agents can be at the workshop
                    max_step_count = max([bid.expected_steps for bid in bid_subset])

                    # Number of steps agents will have to wait at storage until the last agent arrives
                    idle_steps = sum([max_step_count - bid.expected_steps for bid in bid_subset])

                    value = self.rate_combination(max_step_count, idle_steps, prioritisation_activation, number_of_agents)

                    if value > best_value:
                        best_value = value
                        best_combination = bid_subset
                        best_finished_products = combination
                if number_of_agents >= 4 and best_value > AssemblyCombinationDecision.ACTIVATION_THRESHOLD:
                    # we only try combinations with more than 4 agents if we could not find anything with less
                    break
        ettilog.logerr("AssembleContractNetManager:: best bid: %f: %s Starting: %s", best_value, str(best_finished_products), best_value > AssemblyCombinationDecision.ACTIVATION_THRESHOLD)
        if best_value > AssemblyCombinationDecision.ACTIVATION_THRESHOLD:
            return (best_combination, best_finished_products)
        else:
            # rospy.logerr("No bid found. best: %f: %s with %d agents", best_value, str(best_finished_products), len(best_combination))

            return (None, None)

    def init_finished_products(self):
        finished_products = self._product_provider.finished_products
        self.finished_item_list = finished_products.keys()
        self.finished_item_list.sort(key=self.natural_keys)
        self.finished_products = {}
        self.products = self._product_provider.products.keys()
        self.products.sort(key=self.natural_keys)

        for finished_product in finished_products.values():
            self.finished_products[finished_product.name] = np.zeros(len(self.products) + len(self.roles))

            for item in finished_product.consumed_items:
                index = self.products.index(item.name)
                self.finished_products[finished_product.name][index] += item.amount

            for role in finished_product.required_roles:
                self.finished_products[finished_product.name][len(self.products) + self.roles.index(role)] = 1

            self.finished_products[finished_product.name][self.products.index(finished_product.name)] += -1

    def try_build_item(self, products, priorities, item=None):
        res = []
        best_value = -np.inf
        try_items = self.finished_item_list

        if item is not None:
            products = products - self.finished_products[item]

            if min(products) < 0:
                return np.inf, None

            res = [item]
            best_value = sum(products[0:-len(self.roles)]) * AssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT + priorities[item] * \
                         AssemblyCombinationDecision.WEIGHT_PRIORITY
            try_items = self.finished_item_list[self.finished_item_list.index(item):]

        best_item_list = None
        for i in try_items:
            value, item_list = self.try_build_item(products, priorities, i)
            if item_list:
                if value > best_value:
                    best_value = value
                    best_item_list = item_list
        if best_item_list is not None:
            res = res + best_item_list

        return best_value, res

    def finished_items_priority_dict(self):
        stock_items = self._product_provider.total_items_in_stock(types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)
        stored_items = self._product_provider.get_stored_items()

        finished_stock_items = {}
        for item in self._product_provider.finished_products.keys():
            finished_stock_items[item] = stock_items.get(item, 0) + stored_items.get(item, 0)
        values = finished_stock_items.values()
        if len(values) == 0:
            ettilog.logerr("ChooseBestAssemblyCombination:: Finished products not yet loaded")
            return {}

        for key, count in finished_stock_items.iteritems():
            # Priority value is the percentage of items still needed to reach finished product goal
            priority = 1.0 - (float(count) / self.finished_product_goals.get(key, JobDecider.DEFAULT_FINISHED_PRODUCT_GOAL))
            # priority can not be lower than 0 -> this would mean we want to destroy finished products
            priority = max(priority, 0.0)
            finished_stock_items[key] = priority
        return finished_stock_items

    def rate_combination(self, max_step_count, idle_steps, prioritisation_activation, number_of_agents):
        return max_step_count * AssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT + \
               idle_steps * AssemblyCombinationDecision.WEIGHT_IDLE_STEPS + \
               prioritisation_activation * AssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION + \
               number_of_agents * AssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS

    def bid_to_array(self, bid):
        """

        :param bid:
        :param bid: TaskBid
        :return:
        """
        bid_array = np.zeros(len(self.products) + len(self.roles))

        for item in bid.items:
            bid_array[self.products.index(item)] += 1

        bid_array[len(self.products) + self.roles.index(bid.role)] = 999

        return bid_array

    def atof(self, text):
        try:
            retval = float(text)
        except ValueError:
            retval = text
        return retval

    def natural_keys(self, text):
        '''
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        float regex comes from https://stackoverflow.com/a/12643073/190597
        '''
        return [self.atof(c) for c in re.split(r'[+-]?([0-9]+(?:[.][0-9]*)?|[.][0-9]+)', text)]
