import rospy
from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import Movement

from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from decisions.gathering import ChooseIngredientToGather
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.behaviours.gather')


class ChooseIngredientBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        self.agent_name = agent_name
        super(ChooseIngredientBehaviour, self).__init__(**kwargs)
        self._movement_knowledgebase = MovementKnowledgebase()
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
        resource = self._choose_item.choose_resource()

        if resource.item is not None:
            self._product_provider.start_gathering(resource.item.name)
            self._movement_knowledgebase.start_movement(Movement(
                identifier=MovementKnowledgebase.IDENTIFIER_GATHERING,
                agent_name=self.agent_name,
                pos=resource.pos
            ))
            ettilog.logerr("ChooseIngredientBehaviour:: Chosing item %s", resource.item)
        else:
            ettilog.logerr("ChooseIngredientBehaviour:: Trying to choose item, but none fit in stock")
