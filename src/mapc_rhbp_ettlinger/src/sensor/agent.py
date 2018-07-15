from behaviour_components.sensors import RawTopicSensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.agent')


class StorableItemsLoadSensor(GradientSensor):
    """
    Sensor to track the percentage of the load that is used for finished products
    """

    def __init__(self, name, agent_name, choose_finished_product_decision):
        super(StorableItemsLoadSensor, self).__init__(name, mechanism=choose_finished_product_decision, initial_value=0,
                                                      sensor_type=SENSOR.VALUE)

        self._product_provider = ProductProvider(agent_name=agent_name)

    def calc(self):
        val = super(StorableItemsLoadSensor, self).calc()
        if val is None:
            return 0
        else:
            return self._product_provider.calculate_total_volume_dict(val)
