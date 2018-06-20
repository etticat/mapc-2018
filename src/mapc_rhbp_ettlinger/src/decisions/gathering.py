import operator

from agent_knowledge.item import StockItemKnowledgebase
from common_utils import rhbp_logging
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')

class ChooseIngredientToGather(object):

    def __init__(self, agent_name):
        self._stock_item_knowledgebase = StockItemKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent_name)

    def update(self, msg):
        self.load_free = msg.load_max - msg.load

    def choose(self):
        choosen_item  = None

        # TODO: Take distance into account

        for item, already_in_stock_items in self.ingredient_priority():
            load_after_gathering = self.load_after_gathering(item)
            if load_after_gathering >= 0:
                choosen_item = item
                break

        return choosen_item

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