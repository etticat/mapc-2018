import rospy
from mac_ros_bridge.msg import GenericAction
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.task import TaskKnowledgeBase
from behaviours.movement import GoToTaskDestinationBehaviour
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from provider.distance_provider import DistanceProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from rhbp_selforga.behaviours import DecisionBehaviour
from self_organisation.decisions import ExplorationDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.exploration')


class ChooseDestinationBehaviour(DecisionBehaviour):
    def __init__(self, name, agent_name, **kwargs):

        self._task_knowledge_base = TaskKnowledgeBase()
        self.agent_name = agent_name

        self.destination_provider = DistanceProvider()
        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.exploration_decision = ExplorationDecision(self.self_organisation_provider.buffer)
        self.action_provider = ActionProvider(agent_name=agent_name)

        super(ChooseDestinationBehaviour, self).__init__(self.exploration_decision, name=name, **kwargs)

    def do_step(self):
        xy = super(ChooseDestinationBehaviour, self).do_step()
        if xy is not None:
            x, y = xy
            destination = self.destination_provider.position_from_xy(x, y)
            self._task_knowledge_base.create_task(Task(
                type=TaskKnowledgeBase.TYPE_EXPLORATION,
                agent_name=self.agent_name,
                pos=destination
            ))

            self.action_provider.action_go_to_location(lat=destination.lat, lon=destination.long)
        else:
            ettilog.logerr("ChooseDestinationBehaviour:: could not choose position. ")
