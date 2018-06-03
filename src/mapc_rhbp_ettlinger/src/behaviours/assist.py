
import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from common_utils.agent_utils import AgentUtils
from agent_knowledge.assist import AssistKnowledgebase
from agent_knowledge.tasks import TaskKnowledgebase
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import AmountInListActivator
from sensor.movement import DestinationDistanceSensor



class GoToAssistSpotBehaviour(GotoLocationBehaviour):

    def __init__(self, agent_name, **kwargs):
        super(GoToAssistSpotBehaviour, self).__init__(agent_name=agent_name, **kwargs)
        self._assist_knowledge = AssistKnowledgebase()

    def _select_pos(self):
        assistTask = self._assist_knowledge.get_assist_task(self._agent_name)

        if assistTask == None:
            return None
        else:
            return assistTask.pos


class AssistBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(AssistBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._assist_knowledge = AssistKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

    def action_assist_assemble(self, agent):
        action = GenericAction()
        action.action_type = Action.ASSIST_ASSEMBLE
        action.params = [
            KeyValue("Agent", str(agent))]

        self._pub_generic_action.publish(action)

    def do_step(self):
        assistTask = self._assist_knowledge.get_assist_task(self._agent_name)
        if assistTask != None:
            self.action_assist_assemble(assistTask.agent_name)
            rospy.logerr("AssistBehaviour(%s):: assisting %s", self._agent_name, assistTask.agent_name)
