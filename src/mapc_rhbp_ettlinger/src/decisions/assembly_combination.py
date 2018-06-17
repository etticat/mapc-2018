import copy
import itertools
import operator

from agent_knowledge.item import StockItemKnowledgebase
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider


class ChooseBestAssemblyCombination(object):

    WEIGHT_LOAD = 8
    WEIGHT_INGREDIENT_LOAD = 8
    WEIGHT_STEPS = -9

    ACTIVATION_THRESHOLD = 77

    def __init__(self):

        self._stock_item_knowledgebase = StockItemKnowledgebase()
        self._product_provider = ProductProvider(agent_name="agentA1") # TODO: make independent from agent
        self.max_load = 100
        self.load = 60
        self.load_ingredients = 40
        self.load_finished_products = 35
        self.speed = 10
        self.transport = "air"
        self.items = {}

    def choose(self, bids):

        best_combination = []
        best_value = 5
        best_finished_products = {}

        if len(bids) >= 2:
            # Go through all combinations
            for L in range(2, min(len(bids) + 1, 7)): # We try all combinations using 2-7 agents
                for subset in itertools.combinations(bids, L):
                    stringi = ""
                    for item in subset:
                        stringi = stringi + item.agent_name + "(" + item.role + ")" + " - "

                    combination = self.generate_best_combination(subset)

                    value = self.generate_value_from_combination(combination)

                    if value > best_value:
                        best_value = value
                        best_combination = subset
                        best_finished_products = combination
                if L >= 4 and best_value > 5:
                    # we only try combinations with more than 4 agents if we could not find anything with less
                    break

        return (best_combination, best_finished_products)

    def generate_best_combination(self, subset):
        item_dict, roles = self._product_provider.get_items_and_roles_from_bids(subset)

        finished_products = self.generate_best_finished_product_combination(item_dict, roles)

        return finished_products

    def generate_best_finished_product_combination(self, item_dict, roles):
        item_dict = copy.copy(item_dict)
        combination = {}

        finished_items_to_build = self.finished_items_priority()
        for item, count in finished_items_to_build:
            required_roles = self._product_provider.get_roles_of_product(item)
            if set(required_roles).issubset(roles):
                ingredients = self._product_provider.get_ingredients_of_product(item)
                item_count = CalcUtil.dict_max_diff(ingredients, item_dict)

                if item_count > 0:
                    combination[item] = item_count

                item_dict = CalcUtil.dict_diff(item_dict, self._product_provider.get_ingredients_of_product(item, item_count))

        return combination

    def generate_value_from_combination(self, combination):
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
        # Currently prioritize only by which items we haev the fewest of
        # TODO: Use more data to decide
        stock_items = self._stock_item_knowledgebase.get_total_stock_and_goals()
        finished_stock_items = {}
        for item in self._product_provider.finished_products.keys():
            finished_stock_items[item] = stock_items[item]["stock"] + stock_items[item]["goal"]
        return sorted(finished_stock_items.iteritems(), key=operator.itemgetter(1))
