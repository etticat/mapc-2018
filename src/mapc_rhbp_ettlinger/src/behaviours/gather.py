import rospy
from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.gathering import ChooseIngredientToGather
from provider.action_provider import ActionProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.gather')


class ChooseIngredientBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.agent_name = agent_name
        super(ChooseIngredientBehaviour, self).__init__(requires_execution_steps=True, **kwargs)
        self._task_knowledge_base = TaskKnowledgeBase()
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

        if resource is not None:
            if resource.item is not None:
                self._product_provider.start_gathering(resource.item.name)
                self._task_knowledge_base.create_task(Task(
                    type=TaskKnowledgeBase.TYPE_GATHERING,
                    agent_name=self.agent_name,
                    pos=resource.pos,
                    task=resource.item.name
                ))
                self.action_provider.action_go_to_location(resource.pos)
                ettilog.logerr("ChooseIngredientBehaviour(%s):: Choosing item %s", self.agent_name, resource.item)
            else:
                ettilog.logerr("ChooseIngredientBehaviour(%s):: Trying to choose item, but none fit in stock", self.agent_name)

        else:
            ettilog.logerr("ChooseIngredientBehaviour(%s):: Trying to choose item, but resources no resource chosen", self.agent_name)
