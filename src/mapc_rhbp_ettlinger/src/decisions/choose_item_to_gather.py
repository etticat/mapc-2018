import operator

import numpy as np
from mac_ros_bridge.msg import Agent, SimStart

import rospy

from common_utils import etti_logging

from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.best_agent_assembly_combination import BestAgentAssemblyCombinationDecision
from provider.action_provider import ActionProvider
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')


class ChooseItemToGatherMechanism(DecisionPattern):
    """
    Chooses an item to gather next by looking at which items are needed most
    """

    FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION = 1.9
    WEIGHT_STEPS = -1
    WEIGHT_ASSEMBLY_ROLE_MATCH = 2.5
    WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT = 0.5
    WEIGHT_PRIORITY = 200
    THRESHOLD = -np.inf

    def __init__(self, agent_name, assembly_combination_decision):

        self.agent_name = agent_name
        self.choose_best_assembly_combination = assembly_combination_decision

        self._init_config()

        self._chosen_resource = None
        self._last_agent_position = None
        self._last_calculated_agent_position = None

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)
        self._action_provider = ActionProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)

        super(ChooseItemToGatherMechanism, self).__init__(buffer=None, frame=None, requres_pos=False)
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):
        ChooseItemToGatherMechanism.WEIGHT_STEPS = rospy.get_param("ChooseItemToGatherMechanism.WEIGHT_STEPS",
                                                                   ChooseItemToGatherMechanism.WEIGHT_STEPS)
        ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH = rospy.get_param(
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH",
            ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH)
        ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT = rospy.get_param(
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT",
            ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT)
        ChooseItemToGatherMechanism.WEIGHT_PRIORITY = rospy.get_param("ChooseItemToGatherMechanism.WEIGHT_PRIORITY",
                                                                      ChooseItemToGatherMechanism.WEIGHT_PRIORITY)
        ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION = rospy.get_param(
            "ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION",
            ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION)

    def calc_value(self):
        """
        Returns the resource whith the highest activation to gather there
        :return:
        """
        # Calculate a new resource if we currently don't have any or the selected item doesnt fit in the stock
        if self._chosen_resource is None or not self._product_provider.fits_in_store(self._chosen_resource.item.name):
            resource = self.choose_resource()

            if resource is not None:
                self._chosen_resource = resource
                ettilog.loginfo("ChooseResourceMechanism(%s):: Choosing item %s", self.agent_name, resource.item.name)
            else:
                ettilog.logerr("ChooseResourceMechanism(%s):: Trying to choose item, but no resource chosen",
                               self.agent_name)
                self._chosen_resource = None

        # Uptdate gather goal every time to reflect changed resource or changed agent load
        if self._chosen_resource is not None:
            self._product_provider.update_gathering_goal(self._chosen_resource.item.name)
        else:
            self._product_provider.remove_gathering_goal()

        self.value = self._chosen_resource
        return [self._chosen_resource, self.state]

    def end_gathering(self):
        """
        When gathering is over, remove goal and reset mechanism
        :return:
        """
        self._product_provider.remove_gathering_goal()
        self._chosen_resource = None

    def choose_resource(self):
        """
        Chooses the resource, that offers an item that is needed most and is also not too far away
        :return:
        """
        chosen_resource = None

        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        max_activation = ChooseItemToGatherMechanism.THRESHOLD

        item_priority = self.ingredient_priority_dict()

        # Iterate over all ingredient priorities
        for item, already_in_stock_items in item_priority.iteritems():
            load_after_gathering = self.load_after_gathering(item)
            usefulness_for_assembly = self._product_provider.usages_for_assembly(item)
            # if another of those items fits in stock and item can be gathered, look for the best resource to gather it
            if load_after_gathering >= 0 and item in gatherable_items:
                steps, resource = self.steps_to_closest_resource(resources, item)

                # Calculate activation of going to that exact resource
                activation = already_in_stock_items * ChooseItemToGatherMechanism.WEIGHT_PRIORITY + \
                             steps * ChooseItemToGatherMechanism.WEIGHT_STEPS + \
                             int(usefulness_for_assembly > 0) * ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH + \
                             usefulness_for_assembly * ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT

                # If its the best resource until now, choose it
                if activation > max_activation:
                    max_activation = activation
                    chosen_resource = resource

        return chosen_resource

    def load_after_gathering(self, item):
        """
        Returns the load of the agent after gathering one instance of the item
        :param item:
        :return:
        """
        product = self._product_provider.get_product_by_name(item)
        load_after_gathering = self._product_provider.load_free - product.volume
        return load_after_gathering

    def ingredient_priority_dict(self):
        """
        Returns a dictionary with all products as keys. The values describe how much they are needed on a scale from 0 to 1
        for assembly of further items.
        :return:
        """
        # Get currently desired ingredients
        desired_ingredients = self.get_desired_ingredients(consider_intermediate_ingredients=False)
        # Stock items that we currently have or are in the process of gathering
        stock_items = self._product_provider.get_agent_stock_items(types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)

        # Calculate the priority by dividing the current percentage by the max percentage
        priority_dict = {}
        for item in desired_ingredients.keys():

            if stock_items.get(item, 0) == 0:
                priority = 1.0
            elif desired_ingredients.get(item, 0) == 0:
                priority = 0.0
            else:
                priority = 1.0 - (float(stock_items.get(item, 0)) / desired_ingredients.get(item, 0))
            priority_dict[item] = priority

        min_value = min(priority_dict.values())
        max_value = max(priority_dict.values())

        for key, value in priority_dict.iteritems():
            priority_dict[key] = (value - min_value) / max((max_value-min_value), 0.01)

        return priority_dict

    def get_desired_ingredients(self, consider_intermediate_ingredients):
        """
        Returns the desired number of all ingredients calculated back from the finished item goal
        :param consider_intermediate_ingredients:
        :return:
        """

        # Get the desired finished product stock
        finished_product_priority = self.choose_best_assembly_combination.finished_items_priority_dict(normalize_to_zero=True)

        finished_product_desired_stock = CalcUtil.multiply_dict_by_factor(finished_product_priority,
                                      ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION)

        # Check how many ingredients we need to assemble the stock
        desired_ingredients = self._product_provider.get_gatherable_ingredients_of_dict(finished_product_desired_stock,
                                            consider_intermediate_ingredients=consider_intermediate_ingredients)

        for key, value in desired_ingredients.iteritems():
            desired_ingredients[key] += 30

        return desired_ingredients

    def steps_to_closest_resource(self, resources, item):
        """
        Returns the number of steps needed to get to a resource that allows gathering of a specified item
        This does not take battery charging into account.
        :param resources:
        :param item:
        :return:
        """
        min_steps = 999
        best_resource = None

        for resource in resources.values():
            if resource.item.name == item:
                steps = self._distance_provider.calculate_steps(resource.pos)
                if steps < min_steps:
                    min_steps = steps
                    best_resource = resource

        return min_steps, best_resource

    def destination_not_found(self, pos):
        pass