import operator

import numpy as np
from mac_ros_bridge.msg import Agent

import rospy

from common_utils import etti_logging

from common_utils.agent_utils import AgentUtils
from decisions.assembly_combination import AssemblyCombinationDecision
from provider.action_provider import ActionProvider
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern


ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')


class GatherDecisionMechanism(DecisionPattern):

    WEIGHT_STEPS = 1
    WEIGHT_ASSEMBLY_ROLE_MATCH = 2.5
    WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT = 0.5
    WEIGHT_PRIORITY = 200
    THRESHOLD = -np.inf

    def __init__(self, agent_name, assembly_combination_decision):

        self.init_config()

        self.agent_name = agent_name
        self._last_agent_position = None
        self._last_calculated_agent_position = None

        self.facility_provider = FacilityProvider()
        self.distance_provider = DistanceProvider()

        self.action_provider = ActionProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self.chosen_resource = None

        self.facility_provider = FacilityProvider()
        self.step_provider = DistanceProvider()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider()
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)
        self.choose_best_assembly_combination = assembly_combination_decision

        super(GatherDecisionMechanism, self).__init__(buffer=None, frame=None, requres_pos=False)

    def init_config(self):
        GatherDecisionMechanism.WEIGHT_STEPS = rospy.get_param("~GatherDecisionMechanism.WEIGHT_STEPS",
                                                               GatherDecisionMechanism.WEIGHT_STEPS)
        GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH = rospy.get_param(
            "~GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH", GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH)
        GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT = rospy.get_param(
            "~GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT",
            GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT)
        GatherDecisionMechanism.WEIGHT_PRIORITY = rospy.get_param("~GatherDecisionMechanism.WEIGHT_PRIORITY",
                                                                  GatherDecisionMechanism.WEIGHT_PRIORITY)

    def calc_value(self):
        if self.chosen_resource is None or not self._product_provider.fits_in_store(self.chosen_resource.item.name):
            resource = self.choose_resource()
            # TODO: Pull in fuctionality

            if resource is not None:
                if resource.item is not None:
                    self.chosen_resource = resource
                    ettilog.loginfo("ChooseResourceMechanism(%s):: Choosing item %s", self.agent_name, resource.item.name)
                    self.chosen_resource = resource
                else:
                    ettilog.logerr("ChooseResourceMechanism(%s):: Trying to choose item, but none fit in stock",
                                   self.agent_name)
                    self.chosen_resource = None

            else:
                ettilog.logerr(
                    "ChooseResourceMechanism(%s):: Trying to choose item, but no resource chosen",
                    self.agent_name)
                self.chosen_resource = None
        if self.chosen_resource is not None:
            self._product_provider.update_gathering_goal(self.chosen_resource.item.name)
        else:
            self._product_provider.remove_gathering_goal()
        return [self.chosen_resource, self.state]

    def end_gathering(self):
        self._product_provider.remove_gathering_goal()
        self.chosen_resource = None

    def choose_resource(self):
        choosen_resource = None

        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        max_activation = GatherDecisionMechanism.THRESHOLD

        item_priority = self.ingredient_priority_dict()
        for item, already_in_stock_items in item_priority.iteritems():
            load_after_gathering = self.load_after_gathering(item)
            usefulness_for_assembly = self._product_provider.usages_for_assembly(item)
            if load_after_gathering >= 0 and item in gatherable_items:
                steps, resource = self.steps_to_closest_resource(resources, item)
                activation = already_in_stock_items * GatherDecisionMechanism.WEIGHT_PRIORITY + \
                             steps * GatherDecisionMechanism.WEIGHT_STEPS + \
                             int(usefulness_for_assembly > 0) * GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH + \
                             usefulness_for_assembly * GatherDecisionMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT
                if activation > max_activation:
                    max_activation = activation
                    choosen_resource = resource

        return choosen_resource

    def load_after_gathering(self, item):
        product = self._product_provider.get_product_by_name(item)
        load_after_gathering = self._product_provider.load_free - product.volume
        return load_after_gathering

    def ingredient_priority_sorted(self):
        finished_stock_items = self.ingredient_priority_dict()

        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1))

    def ingredient_priority_dict(self):
        desired_ingredients = self.get_desired_ingredients(consider_intermediate_ingredients=False)
        # Stock items that we currently have or are in the process of gathering
        stock_items = self._product_provider.total_items_in_stock(types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)
        stored_items = self._product_provider.get_stored_items()
        current_stock = {}

        max_percentage = 0.01 # ignore everything lower than 1%
        for item in desired_ingredients.keys():
            current_stock[item] = stock_items.get(item,0) + stored_items.get(item, 0)
            percentage = (current_stock[item] + 1)/ (desired_ingredients.get(item, 0) + 1)
            if percentage  > max_percentage:
                max_percentage = percentage



        priority_dict = {}
        for item in desired_ingredients.keys():
            if current_stock[item] == 0:
                priority = 1.0
            elif desired_ingredients.get(item, 0) == 0:
                priority = 0.0
            else:
                priority = 1.0 - (float(current_stock[item]) / (desired_ingredients.get(item, 0) * max_percentage))

            priority_dict[item] = priority
        return priority_dict

    def get_desired_ingredients(self, consider_intermediate_ingredients):
        # Get the desired finished product stock
        finished_product_priority = self.choose_best_assembly_combination.finished_items_priority_dict()
        # Check how many ingredients we need to build this stock
        desired_ingredients = self._product_provider.get_ingredients_of_dict(finished_product_priority,
                                                                             consider_intermediate_ingredients=consider_intermediate_ingredients)
        return desired_ingredients

    def steps_to_closest_resource(self, resources, item):
        min_steps = 999
        best_resource = None

        for resource in resources.values():
            if resource.item.name == item:
                steps = self.step_provider.calculate_steps(self._agent_info_provider.pos, resource.pos)
                if steps < min_steps:
                    min_steps = steps
                    best_resource = resource

        return min_steps, best_resource
