import copy
import itertools
import operator

from agent_knowledge.item import StockItemBaseKnowledge
from common_utils import etti_logging
from common_utils.calc import CalcUtil
from common_utils.product_util import ProductUtil
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_combination')

class ChooseBestAssemblyCombination(object):

    WEIGHT_NUMBER_OF_AGENTS = -10
    WEIGHT_VOLUME = 25
    WEIGHT_PRIORITISATION_ACTIVATION = 20
    WEIGHT_IDLE_STEPS = -3
    WEIGHT_MAX_STEP_COUNT = -10

    ACTIVATION_THRESHOLD = 77

    def __init__(self):

        self._stock_item_knowledgebase = StockItemBaseKnowledge()
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

        finished_items_to_build = self.finished_items_priority()
        # TODO: This can be done better: We are looking for the combination which is most needed (Priority)
        # TODO: And using the most items
        for item, count in finished_items_to_build:
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

        max_priority = 0

        finished_items_priority_list = self.finished_items_priority()
        priority_dict = {}
        for item, count in finished_items_priority_list:
            if count > max_priority:
                max_priority = count
            priority_dict[item] = count

        for item, count in combination.iteritems():
            value += (1+(max_priority - priority_dict[item]))  * count

        return value


    def finished_items_priority(self):
        stock_items = self._stock_item_knowledgebase.get_total_stock_and_goals()
        finished_stock_items = {}
        for item in self._product_provider.finished_products.keys():
            finished_stock_items[item] = stock_items[item]["stock"] + stock_items[item]["goal"]
        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1))

    def rate_combination(self, max_step_count, idle_steps, prioritisation_activation, volume, number_of_agents):
        return max_step_count * ChooseBestAssemblyCombination.WEIGHT_MAX_STEP_COUNT + \
                idle_steps *  ChooseBestAssemblyCombination.WEIGHT_IDLE_STEPS + \
                prioritisation_activation * ChooseBestAssemblyCombination.WEIGHT_PRIORITISATION_ACTIVATION + \
                volume * ChooseBestAssemblyCombination.WEIGHT_VOLUME + \
                number_of_agents * ChooseBestAssemblyCombination.WEIGHT_NUMBER_OF_AGENTS

