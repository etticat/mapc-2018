
import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from agent_knowledge.assemble_task import AssembleKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from behaviours.movement import GotoLocationBehaviour
from common_utils.agent_utils import AgentUtils


class GoToAssistLocationBehaviour(GotoLocationBehaviour):
    """
    When an agent is requested to assist, this behaviour ensures, that the agent moves to the location before they can assist
    """
    def __init__(self, agent_name, **kwargs):
        super(GoToAssistLocationBehaviour, self).__init__(agent_name=agent_name, **kwargs)
        self._assist_knowledge = AssembleKnowledgebase()

    def _select_pos(self):
        # TODO: We assume there is always just 1 assist task. Need to check this after refactoring
        assistTask = self._assist_knowledge.get_assist_task(self._agent_name)

        if assistTask == None:
            return None
        else:
            return assistTask.pos


class AssistBehaviour(BehaviourBase):
    """
    Behaviour that allows an agent to assist another agent with their assembly
    """
    def __init__(self, agent_name, **kwargs):
        super(AssistBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._assist_knowledge = AssembleKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

    def action_assist_assemble(self, agent):
        """
        Performs the assist asemble action and publishes it towards the mac_ros_bridge
        :param agent: str
        :return:
        """
        action = GenericAction()
        action.action_type = Action.ASSIST_ASSEMBLE
        action.params = [
            KeyValue("Agent", str(agent))]

        self._pub_generic_action.publish(action)

    def do_step(self):
        """

        :return:
        """
        assistTask = self._assist_knowledge.get_assist_task(self._agent_name)
        if assistTask != None:
            self.action_assist_assemble(assistTask.agent_name)
            rospy.logerr("AssistBehaviour(%s):: assisting %s", self._agent_name, assistTask.agent_name)
