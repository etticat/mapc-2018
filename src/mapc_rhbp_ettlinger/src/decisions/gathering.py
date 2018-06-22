import operator

from agent_knowledge.item import StockItemKnowledgebase
from common_utils import rhbp_logging
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')

class ChooseIngredientToGather(object):

    WEIGHT_STEPS = -5
    WEIGHT_ALREADY_IN_STOCK = -5
    THRESHOLD = -999

    def __init__(self, agent_name):
        self.step_provider = DistanceProvider()
        self._stock_item_knowledgebase = StockItemKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider()
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)

    def update(self, msg):
        self.load_free = msg.load_max - msg.load

    def choose_resource(self):
        choosen_resource  = None

        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        # TODO: Take distance into account

        max_activation = ChooseIngredientToGather.THRESHOLD

        for item, already_in_stock_items in self.ingredient_priority():
            load_after_gathering = self.load_after_gathering(item)
            if load_after_gathering >= 0 and item in gatherable_items:
                steps, resource = self.steps_to_closest_resource(resources, item)
                activation = already_in_stock_items * ChooseIngredientToGather.WEIGHT_ALREADY_IN_STOCK *\
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
        stock_items = self._stock_item_knowledgebase.get_total_stock_and_goals()
        finished_stock_items = {}
        for item in self._product_provider.base_ingredients.keys():
            finished_stock_items[item] = stock_items[item]["stock"] + stock_items[item]["goal"]
        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1))

    def steps_to_closest_resource(self, resources, item):
        min_steps = 999
        resource = None

        for resource in resources.values():
            steps = self.step_provider.calculate_steps(self._agent_info_provider.pos, resource.pos)
            if steps < min_steps:
                min_steps = steps
                resource = resource

        return min_steps, resource