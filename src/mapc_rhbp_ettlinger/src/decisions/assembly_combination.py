import copy
import itertools
import operator

from agent_knowledge.item import StockItemKnowledgeBase
from common_utils import etti_logging
from common_utils.calc import CalcUtil
from common_utils.product_util import ProductUtil
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_combination')

class ChooseBestAssemblyCombination(object):

    WEIGHT_NUMBER_OF_AGENTS = -10
    WEIGHT_VOLUME = 10
    WEIGHT_PRIORITISATION_ACTIVATION = 30
    WEIGHT_IDLE_STEPS = -3
    WEIGHT_MAX_STEP_COUNT = -10

    ACTIVATION_THRESHOLD = 77

    def __init__(self):

        self.facility_provider = FacilityProvider()
        self._stock_item_knowledgebase = StockItemKnowledgeBase()
        self._product_provider = ProductProvider(agent_name="agentA1") # TODO: make independent from agent


    def choose(self, bids):

        best_combination = None
        best_value = 5
        best_finished_products = None

        if len(bids) >= 2:
            # Go through all combinations
            for number_of_agents in range(2, min(len(bids) + 1, 7)): # We try all combinations using 2-7 agents
                for subset in itertools.combinations(bids, number_of_agents):
                    stringi = ""
                    for item in subset:
                        stringi = stringi + item.agent_name + "(" + item.role + ")" + " - "

                    combination = self.generate_best_combination(subset)

                    # The number of step until all agents can be at the workshop
                    max_step_count = max([bid.expected_steps for bid in subset])

                    # Number of steps agents will have to wait at storage until the last agent arrives
                    idle_steps = sum([max_step_count - bid.expected_steps for bid in subset])

                    # Value represents how imporatant the items are to us right now. Rare items are more imporant
                    prioritisation_activation = self.get_prioritisation_activation(combination)

                    # Volume of ingredients. The more item we can build at once, the better
                    volume = self._product_provider.calculate_total_volume(combination)

                    value = self.rate_combination(max_step_count, idle_steps, prioritisation_activation, volume, number_of_agents)



                    if value > best_value:
                        best_value = value
                        best_combination = subset
                        best_finished_products = combination
                if number_of_agents >= 4 and best_value > ChooseBestAssemblyCombination.ACTIVATION_THRESHOLD:
                    # we only try combinations with more than 4 agents if we could not find anything with less
                    break

        return (best_combination, best_finished_products)

    def generate_best_combination(self, subset):
        item_dict, roles = ProductUtil.get_items_and_roles_from_bids(subset)

        item_dict = copy.copy(item_dict)
        combination = {}

        finished_items_to_build = self.finished_items_priority_tuple_list()

        max_priority = 0

        # TODO: This can be done better: We are looking for the combination which is most needed (Priority)
        # TODO: And using the most items
        for item, priority in finished_items_to_build:

            if priority > max_priority:
                max_priority = priority

            if priority < max_priority - 10:
                # Ignore items where we have 10 more than other items
                continue


            required_roles = self._product_provider.get_roles_of_product(item)
            if set(required_roles).issubset(roles):
                ingredients = self._product_provider.get_ingredients_of_product(item)
                item_count = CalcUtil.dict_max_diff(ingredients, item_dict)

                if item_count > 0:
                    combination[item] = item_count

                item_dict = CalcUtil.dict_diff(item_dict, self._product_provider.get_ingredients_of_product(item, item_count))

        return combination


    def get_prioritisation_activation(self, combination):
        # TODO: There needs to be a better logic behind this

        value = 0

        finished_items_priority_list = self.finished_items_priority_dict()
        for item, count in combination.iteritems():
            value += finished_items_priority_list[item]
            finished_items_priority_list[item] = max(finished_items_priority_list[item]-1, 1)
            # For each item we make decrease value by one -> reflects prioritisation

        return value


    def finished_items_priority_tuple_list(self):
        """
        Value of item is defined by base ingredient count. Therefore as priority we only use the ones we have the fewest
        of currently.
        :return:
        """
        finished_stock_items = self.finished_items_priority_dict()
        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1), reverse=True)

    def finished_items_priority_dict(self):
        stock_items = self._stock_item_knowledgebase.get_total_stock_and_goals()
        stored_items = self.facility_provider.get_all_stored_items()
        finished_stock_items = {}
        for item in self._product_provider.finished_products.keys():
            finished_stock_items[item] = max(stock_items.amounts.get(item, 0), stock_items.goals.get(item, 0)) + stored_items.get(item, 0)
        values = finished_stock_items.values()
        if len(values) == 0:
            ettilog.logerr("ChooseBestAssemblyCombination:: Finished products not yet loaded")
            return {}
        max_value = max(values)
        for key, count in finished_stock_items.iteritems():
            finished_stock_items[key] = max_value - count + 1
        return finished_stock_items

    def rate_combination(self, max_step_count, idle_steps, prioritisation_activation, volume, number_of_agents):
        return max_step_count * ChooseBestAssemblyCombination.WEIGHT_MAX_STEP_COUNT + \
                idle_steps *  ChooseBestAssemblyCombination.WEIGHT_IDLE_STEPS + \
                prioritisation_activation * ChooseBestAssemblyCombination.WEIGHT_PRIORITISATION_ACTIVATION + \
                volume * ChooseBestAssemblyCombination.WEIGHT_VOLUME + \
                number_of_agents * ChooseBestAssemblyCombination.WEIGHT_NUMBER_OF_AGENTS

