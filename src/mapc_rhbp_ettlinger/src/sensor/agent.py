from behaviour_components.sensors import RawTopicSensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.agent')


class FinishedProductLoadSensor(RawTopicSensor):

    def __init__(self, name, agent_name):
        self._agent_name = agent_name
        agent_topic = AgentUtils.get_bridge_topic_agent(agent_name=agent_name)
        super(FinishedProductLoadSensor, self).__init__(name, topic=agent_topic)
        self._product_provider = ProductProvider(agent_name=agent_name)

    def subscription_callback(self, msg):
        finished_product = self._product_provider.get_finished_products_in_stock()
        finished_product_volume = self._product_provider.calculate_total_volume(finished_product)
        ettilog.loginfo("FinishedProductLoadSensor(%s):: Volume: %d", self._agent_name, finished_product_volume)
        super(FinishedProductLoadSensor, self).subscription_callback(finished_product_volume)
