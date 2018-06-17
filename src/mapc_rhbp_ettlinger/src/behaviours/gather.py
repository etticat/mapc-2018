import rospy
from mac_ros_bridge.msg import Agent

from behaviour_components.behaviours import BehaviourBase
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from reactions.gathering import ChooseIngredientToGather


class ChooseIngredientBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(ChooseIngredientBehaviour, self).__init__(**kwargs)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._choose_item = ChooseIngredientToGather(agent_name=agent_name)
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, callback=self.callback_agent)

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        self._choose_item.update(msg)

    def do_step(self):
        item = self._choose_item.choose()

        if item is not None:
            self._product_provider.start_gathering(item)
            rospy.logerr("ChooseIngredientBehaviour:: Chosing item %s", item)
        else:
            rospy.logerr("ChooseIngredientBehaviour:: Trying to choose item, but none fit in stock")
