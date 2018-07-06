from agent_knowledge.item import StockItemKnowledgeBase
from behaviour_components.sensors import RawTopicSensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.gathering import ChooseIngredientToGather
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.agent')


class FinishedProductLoadSensor(RawTopicSensor):

    def __init__(self, name, agent_name):
        self._stock_item_knowledgebase = StockItemKnowledgeBase()
        self._agent_name = agent_name
        agent_topic = AgentUtils.get_bridge_topic_agent(agent_name=agent_name)
        super(FinishedProductLoadSensor, self).__init__(name, topic=agent_topic, initial_value=0)
        self._product_provider = ProductProvider(agent_name=agent_name)

        self.choose_ingredient_to_gather= ChooseIngredientToGather(agent_name=agent_name)

    def subscription_callback(self, msg):
        # Check all the products we have in stock
        finished_product_stock = self._product_provider.get_finished_products_in_stock()
        # Check all the products we have, that can be used to make another item
        desired_ingredients = self.choose_ingredient_to_gather.get_desired_ingredients(consider_intermediate_ingredients=True)

        items_to_store = CalcUtil.dict_diff(finished_product_stock, desired_ingredients, normalize_to_zero=True)

        finished_product_volume = self._product_provider.calculate_total_volume(items_to_store)
        ettilog.loginfo("FinishedProductLoadSensor(%s):: Volume: %d", self._agent_name, finished_product_volume)
        super(FinishedProductLoadSensor, self).subscription_callback(finished_product_volume)
