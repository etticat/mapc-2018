import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, WellMsg

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.well_provider import WellProvider
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor

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


class WellIntegritySensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, name, optional=False, knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME):

        super(WellIntegritySensor, self).__init__(
            pattern=TaskKnowledgeBase.generate_tuple(agent_name=agent_name, type=TaskKnowledgeBase.TYPE_BUILD_WELL),
            optional=optional, knowledge_base_name=knowledge_base_name,
            name=name, initial_value=-1)
        self._well_provider = WellProvider()
        self._distance_provider = DistanceProvider()

        rospy.Subscriber(WellProvider.WELL_TOPIC, WellMsg, self._callback_well)

    def _callback_well(self, well_msg):
        """

        :param well_msg:
        :type well_msg: WellMsg
        :return:
        """
        try:
            self._cache_update_callback()
        except Exception:
            # KB not loaded first well callback is  called.
            # Can be ignored as it is called immediately after Knowledge base is loaded again.
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
