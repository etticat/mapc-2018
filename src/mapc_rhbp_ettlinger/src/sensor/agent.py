from mac_ros_bridge.msg import Agent

from behaviour_components.sensors import RawTopicSensor
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from decisions.gathering import ChooseIngredientToGather

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.sensors.agent')

class StorageFitsMoreItemsSensor(RawTopicSensor):

    def __init__(self, agent_name, **kwargs):
        super(StorageFitsMoreItemsSensor, self).__init__(
            topic=AgentUtils.get_bridge_topic_agent(agent_name=agent_name), initial_value=True,  **kwargs)

        self.gathering = ChooseIngredientToGather(agent_name=agent_name)

    def subscription_callback(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        self.gathering.update(msg)
        item = self.gathering.choose()
        self.update(item is not None)
