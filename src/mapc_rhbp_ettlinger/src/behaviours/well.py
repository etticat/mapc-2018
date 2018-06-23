import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, WellMsg

from agent_knowledge.task import TaskKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from behaviours.movement import GotoLocationBehaviour
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.well_provider import WellProvider
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.behaviours.well')


class GoToWellBehaviour(GotoLocationBehaviour):
    """
    Behaviour, that allows going to the Storage location to finish a job
    """

    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToWellBehaviour, self).__init__(
            agent_name=agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self._task_knowledgebase = TaskKnowledgebase()
        self.facility_provider = FacilityProvider()

    def _select_pos(self):
        task = self._task_knowledgebase.get_task(
            agent_name=self._agent_name,
            type=TaskKnowledgebase.TYPE_BUILD_WELL)
        if task is not None:
            return task.pos


class BuildWellBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(BuildWellBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self.current_task = None
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_build_well(self, type):
        action = GenericAction()
        action.action_type = Action.BUILD
        action.params = [
            KeyValue("WellType", str(type))]

        self._pub_generic_action.publish(action)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # TODO: Also add a timeout here: if it doesnt work for 5 steps -> Fail with detailed error
        if self.current_task != None and agent.last_action == "build":
            ettilog.logerr("DeliverJobBehaviour(%s):: Deleting own task. Status: %s", agent.last_action_result,
                           agent.last_action_result)

    def stop(self):
        self.current_task = None
        super(BuildWellBehaviour, self).stop()

    def do_step(self):
        if self.current_task == None:
            self.current_task = self._task_knowledge.get_task(self._agent_name,
                                                              type=TaskKnowledgebase.TYPE_BUILD_WELL)

        if self.current_task != None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.task)
            self.action_build_well(self.current_task.task)
            # TODO: Check what happens when the delivery fails ->
            # TODO: Pass an acnowledgement object into the mac ros bridge. Once it finishes/fails it notifies the application


class BuildUpWellBehaviour(BuildWellBehaviour):

    def action_build_well(self, type):
        action = GenericAction()
        action.action_type = Action.BUILD

        self._pub_generic_action.publish(action)


class FinishTaskBehaviour(BehaviourBase):

    def __init__(self, agent_name, type, **kwargs):
        super(FinishTaskBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._type = type
        self._agent_name = agent_name
        self._task_knowledge_base = TaskKnowledgebase()

    def do_step(self):
        self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=self._type)
        super(FinishTaskBehaviour, self).do_step()


class WellIntegritySensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, name, optional=False, knowledge_base_name=TaskKnowledgebase.KNOWLEDGE_BASE_NAME):

        super(WellIntegritySensor, self).__init__(
            pattern=TaskKnowledgebase.generate_tuple(agent_name=agent_name, type=TaskKnowledgebase.TYPE_BUILD_WELL),
            optional=optional, knowledge_base_name=knowledge_base_name,
            name=name, initial_value=-1)
        self._well_provider = WellProvider()
        self._distance_provider = DistanceProvider()

        rospy.Subscriber(WellProvider.WELL_TOPIC, WellMsg, self._callback_well)

    def _callback_well(self, wellMsg):
        """

        :param wellMsg:
        :type wellMsg: WellMsg
        :return:
        """
        try:
            self._cache_update_callback()
        except Exception:
            # KB not loaded first well callback is  called.
            # Can be ignored as it is called immediately after Knowledgebase is loaded again.
            pass

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
                well_task = TaskKnowledgebase.generate_task_from_fact(fact_tuple)
                for well in self._well_provider.get_existing_wells().values():
                    if self._distance_provider.at_same_location(well.pos, well_task.pos):
                        well_prototype = self._well_provider.get_well(well_task.task)
                        integrity = well.integrity
                        max_integrity = well_prototype.integrity
                        val = integrity / max_integrity
            except Exception:
                ettilog.logwarn("Couldn't get last tuple element of: %s. Resetting to initial_value", str(fact_tuple))

        return val
