import operator

from agent_knowledge.item import StockItemBaseKnowledge
from common_utils import etti_logging
from decisions.assembly_combination import ChooseBestAssemblyCombination
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')


class ChooseIngredientToGather(object):

    WEIGHT_STEPS = 3
    WEIGHT_PRIORITY = 5
    THRESHOLD = -999

    def __init__(self, agent_name):
        self.step_provider = DistanceProvider()
        self._stock_item_knowledgebase = StockItemBaseKnowledge()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider()
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)
        self.load_free = 0
        self.choose_best_assembly_combination = ChooseBestAssemblyCombination()

    def update(self, msg):
        self.load_free = msg.load_max - msg.load

    def choose_resource(self):
        choosen_resource = None

        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        # TODO: Take distance into account

        max_activation = ChooseIngredientToGather.THRESHOLD

        item_priority = self.ingredient_priority()
        for item, already_in_stock_items in item_priority:
            load_after_gathering = self.load_after_gathering(item)
            if load_after_gathering >= 0 and item in gatherable_items:
                steps, resource = self.steps_to_closest_resource(resources, item)
                activation = already_in_stock_items * ChooseIngredientToGather.WEIGHT_PRIORITY + \
                             steps * ChooseIngredientToGather.WEIGHT_STEPS
                if activation > max_activation:
                    max_activation = activation
                    choosen_resource = resource

        return choosen_resource

    def load_after_gathering(self, item):
        product = self._product_provider.get_product_by_name(item)
        load_after_gathering = self.load_free - product.volume
        return load_after_gathering

    def ingredient_priority(self):
        finished_stock_items = self.ingredient_priority_dict()

        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1))

    def ingredient_priority_dict(self):
        # Get the desired finished product stock
        finished_product_priority = self.choose_best_assembly_combination.finished_items_priority_dict()
        # Check how many ingredients we need to build this stock
        desired_ingredients = self._product_provider.get_base_ingredients_of_dict(finished_product_priority)
        # Stock items that we currently have or are in the process of gathering
        stock_items = self._stock_item_knowledgebase.get_total_stock_and_goals()
        finished_stock_items = {}
        for item in self._product_provider.base_ingredients.keys():
            current_stock = max(stock_items[item]["stock"], stock_items[item]["goal"])
            finished_stock_items[item] = desired_ingredients.get(item, 0) - current_stock
        # Desired ingredient stock minus current stock
        return finished_stock_items


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
