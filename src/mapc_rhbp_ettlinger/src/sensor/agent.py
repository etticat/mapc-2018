from mac_ros_bridge.msg import Agent

from behaviour_components.sensors import RawTopicSensor
from common_utils.agent_utils import AgentUtils
from decisions.gathering import ChooseIngredientToGather


class StorageFitsMoreItemsSensor(RawTopicSensor):

    def __init__(self, agent_name, **kwargs):
        super(StorageFitsMoreItemsSensor, self).__init__(
            topic=AgentUtils.get_bridge_topic_agent(agent_name=agent_name), **kwargs)

        self.gathering = ChooseIngredientToGather(agent_name=agent_name)

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        self.gathering.update(msg)
        item = self.gathering.choose()
        self.update(item != None)
