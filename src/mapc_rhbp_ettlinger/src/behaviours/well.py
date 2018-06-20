import rospy
from diagnostic_msgs.msg import KeyValue
from knowledge_base.knowledge_base_manager import KnowledgeBase
from mac_ros_bridge.msg import GenericAction, Agent, WellMsg

from agent_knowledge.well import WellTaskKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from behaviours.movement import GotoLocationBehaviour
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
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
        self._well_knowledgebase = WellTaskKnowledgebase()
        self.facility_provider = FacilityProvider()

    def _select_pos(self):
        task = self._well_knowledgebase.get_task(agent_name=self._agent_name)
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
        self._task_knowledge = WellTaskKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent, self._action_request_agent)

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
            ettilog.logerr("DeliverJobBehaviour(%s):: Deleting own task. Status: %s", agent.last_action_result, agent.last_action_result)
            self._task_knowledge.build_finished(self.current_task)


    def stop(self):
        self.current_task = None
        super(BuildWellBehaviour, self).stop()

    def do_step(self):
        if self.current_task == None:
            self.current_task = self._task_knowledge.get_task(self._agent_name)

        if self.current_task != None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.well_type)
            self.action_build_well(self.current_task.well_type)
            # TODO: Check what happens when the delivery fails ->
            # TODO: Pass an acnowledgement object into the mac ros bridge. Once it finishes/fails it notifies the application


class BuildUpWellBehaviour(BuildWellBehaviour):

    def action_build_well(self, type):
        action = GenericAction()
        action.action_type = Action.BUILD

        self._pub_generic_action.publish(action)

    def stop(self):
        self._task_knowledge.build_up_finished(self.current_task)
        super(BuildUpWellBehaviour, self).stop()


class WellIntegritySensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, name, proximity, optional=False, knowledge_base_name=KnowledgeBase.DEFAULT_NAME):

        super(WellIntegritySensor, self).__init__(pattern=WellTaskKnowledgebase.generate_tuple(agent_name=agent_name),
                                               optional=optional, knowledge_base_name=knowledge_base_name,
                                               name=name, initial_value=-1)
        self._well_provider = WellProvider()
        self.proximity = proximity

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
                well_task = WellTaskKnowledgebase.generate_well_task_from_fact(fact_tuple)
                for well in self._well_provider.get_existing_wells().values():
                    if AgentUtils.calculate_distance(well.pos, well_task.pos) < self.proximity:
                        well_prototype = self._well_provider.get_well(well_task.well_type)
                        integrity = well.integrity
                        max_integrity = well_prototype.integrity
                        val = integrity*100/max_integrity
            except Exception:
                ettilog.logwarn("Couldn't get last tuple element of: %s. Resetting to initial_value", str(fact_tuple))


        return val
