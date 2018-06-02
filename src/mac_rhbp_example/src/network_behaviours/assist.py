import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from common_utils.agent_utils import AgentUtils
from agent_knowledge.assist import AssistKnowledgebase
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import IngredientSensor, FinishedProductSensor, AmountInListActivator
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


class AssistBehaviourNetwork(NetworkBehaviour):

    def __init__(self, agent_name, name, msg, **kwargs):

        proximity = msg.proximity

        super(AssistBehaviourNetwork, self).__init__(name, **kwargs)


        self.go_to_assist_spot_behaviour = GoToAssistSpotBehaviour(
            agent_name = agent_name,
            name="go_to_assist_spot_behaviour",
            plannerPrefix=self.get_manager_prefix()
        )

        self.assist_assigned_sensor = KnowledgeSensor(
            name='assist_assigned_sensor',
            pattern=AssistKnowledgebase.generate_tuple(
                assisting_agent=agent_name,
                active=True))

        self.at_assist_spot_sensor = DestinationDistanceSensor(
            name = "at_assist_spot_sensor",
            agent_name=agent_name,
            behaviour_name=self.go_to_assist_spot_behaviour.name
        )

        self.at_assist_spot_condition = Condition(
            sensor=self.at_assist_spot_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

        self.assist_assigned_condition = Condition(
            sensor=self.assist_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

        self.go_to_assist_spot_behaviour.add_precondition(
            precondition=Negation(self.at_assist_spot_condition)
        )

        self.assist_behaviour = AssistBehaviour(
            name="assist_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.assist_behaviour.add_precondition(
            precondition=self.at_assist_spot_condition
        )

        self.assist_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.assist_assigned_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        # Overall effect
        self.add_effect(
            effect=Effect(
                sensor_name=self.assist_assigned_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )