import copy
import traceback

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, WellMsg

from agent_knowledge.local_knowledge_sensors import LocalKnowledgeFirstFactSensor
from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from provider.distance_provider import DistanceProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.well_provider import WellProvider
from rhbp_selforga.behaviours import DecisionBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor
from self_organisation.decisions import WellPositionDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.well')


class BuildWellBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(BuildWellBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._current_task = None
        self._agent_name = agent_name
        self._task_knowledge_base = TaskKnowledgeBase()
        self._action_provider = ActionProvider(agent_name=agent_name)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_build_well(self, type):
        action = GenericAction()
        action.action_type = Action.BUILD
        action.params = [
            KeyValue("WellType", str(type))]

        self._action_provider.send_action(action)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        if self._current_task is not None:
            if agent.last_action == "build":
                if agent.last_action_result == "failed_resources":
                    ettilog.logerr("GoToWellBehaviour:: Error: trying to build well but resources are not available")
                    self._task_knowledge_base.finish_task(agent_name=self._agent_name,
                                                          type=TaskKnowledgeBase.TYPE_BUILD_WELL)

    def stop(self):
        self._current_task = None
        super(BuildWellBehaviour, self).stop()

    def do_step(self):
        if self._current_task is None:
            self._current_task = self._task_knowledge_base.get_task(self._agent_name,
                                                                    type=TaskKnowledgeBase.TYPE_BUILD_WELL)

        if self._current_task is not None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self._current_task.task)
            self.action_build_well(self._current_task.task)
            # TODO: Check what happens when the delivery fails -> TODO: Pass an acnowledgement object into the mac
            # TODO: ros bridge. Once it finishes/fails it notifies the application


class BuildUpWellBehaviour(BuildWellBehaviour):

    def action_build_well(self, well_type):
        action = GenericAction()
        action.action_type = Action.BUILD

        self._pub_generic_action.publish(action)


class WellIntegritySensor(LocalKnowledgeFirstFactSensor):

    def __init__(self, agent_name, name, optional=False, knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME):

        super(WellIntegritySensor, self).__init__(
            pattern=TaskKnowledgeBase.generate_tuple(agent_name=agent_name, type=TaskKnowledgeBase.TYPE_BUILD_WELL),
            optional=optional, knowledge_base_name=knowledge_base_name,
            name=name, initial_value=-1)
        self._well_provider = WellProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider()

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        val = self._initial_value

        if len(facts) > 0:
            fact_tuple = facts.pop()  # only getting the first fact

            try:
                well_task = TaskKnowledgeBase.generate_task_from_fact(fact_tuple)
                for well in self._well_provider.get_existing_wells().values():
                    if self._distance_provider.at_same_location(well.pos, well_task.pos):
                        well_prototype = self._well_provider.get_well(well_task.task)
                        integrity = well.integrity
                        max_integrity = well_prototype.integrity
                        val = integrity / max_integrity
            except Exception:
                ettilog.logwarn("Couldn't get last tuple element of: %s. Resetting to initial_value", str(fact_tuple))

        return val


class ChooseWellPositionBehaviour(DecisionBehaviour):

    def __init__(self, name, agent_name, **kwargs):

        self.agent_name = agent_name
        self._task_knowledge_base = TaskKnowledgeBase()
        self.destination_provider = DistanceProvider()
        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.well_position_decision = WellPositionDecision(self.self_organisation_provider.buffer)

        super(ChooseWellPositionBehaviour, self).__init__(self.well_position_decision, name=name, **kwargs)

    def do_step(self):
        try:
            xy = super(ChooseWellPositionBehaviour, self).do_step()
            if xy is not None:
                x, y = xy
                destination = self.destination_provider.position_from_xy(x, y)
                task = self._task_knowledge_base.get_task(agent_name=self.agent_name,
                                                          type=TaskKnowledgeBase.TYPE_BUILD_WELL)
                new_task = copy.copy(task)
                new_task.pos = destination

                update_successful = self._task_knowledge_base.update_task(task, new_task)
                ettilog.logerr("ChooseDestinationBehaviour:: Build well behaviour updated %s", str(update_successful))
            else:
                ettilog.logerr("ChooseDestinationBehaviour:: could not choose position. ")


        except Exception as e:
            print(e)
            print(traceback.format_exc())