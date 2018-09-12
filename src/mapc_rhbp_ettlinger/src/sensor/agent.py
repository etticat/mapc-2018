from mac_ros_bridge.msg import SimStart

from behaviour_components.sensors import RawTopicSensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.agent')


class FinishedProductsLoadSensor(GradientSensor):
    """
    Sensor to track the percentage of the load that is used for finished products
    """

    def __init__(self, name, agent_name, choose_finished_product_decision):
        super(FinishedProductsLoadSensor, self).__init__(name, mechanism=choose_finished_product_decision,
                                                         initial_value=0, sensor_type=SENSOR.VALUE)

        self._product_provider = ProductProvider(agent_name=agent_name)

    def calc(self):
        val = super(FinishedProductsLoadSensor, self).calc()
        if val is None:
            return 0
        else:
            return self._product_provider.calculate_total_volume_dict(val)


class CanFlySensor(RawTopicSensor):

    def __init__(self, name, agent_name, message_type=None, initial_value=None, create_log=False, print_updates=False):

        topic = AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start")
        super(CanFlySensor, self).__init__(name, topic, message_type=message_type, initial_value=initial_value,
                                           create_log=create_log, print_updates=print_updates)
        self.asd = None

    def subscription_callback(self, msg):
        """

        :param msg:
        :type msg: SimStart
        :return:
        """
        super(CanFlySensor, self).subscription_callback(msg.role.name == "drone")
