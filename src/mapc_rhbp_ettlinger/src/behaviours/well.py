from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import Agent

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import Action
from provider.action_provider import ActionProvider
from rhbp_selforga.behaviours import DecisionBehaviour

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.well')


class BuildWellBehaviour(DecisionBehaviour):
    """
    Behaviour to build a well
    """
    def __init__(self, name, mechanism, agent_name, **kwargs):
        super(BuildWellBehaviour, self) \
            .__init__(name=name, mechanism=mechanism,
            **kwargs)

        self._agent_name = agent_name

        self._current_task = None

        self._action_provider = ActionProvider(agent_name=agent_name)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_build_well(self, type):
        """
        Perform the buiold well action
        :param type:
        :return:
        """
        self._action_provider.send_action(action_type=Action.BUILD, params=[
            KeyValue("WellType", str(type))])

    def _action_request_agent(self, agent):
        """
        Check if building was unsuccessful and end task if this was the case
        :param agent:
        :type agent: Agent
        :return:
        """
        if self._current_task is not None:
            if agent.last_action == "build":
                if agent.last_action_result is not "successful":
                    ettilog.logerr("GoToWellBehaviour:: Building well failed with error %s", agent.last_action_result)
                    self.mechanism.end_task()

    def stop(self):
        """
        Delete reference of task on stop
        :return:
        """
        self._current_task = None
        super(BuildWellBehaviour, self).stop()

    def do_step(self):
        """
        Builds a well
        :return:
        """
        self._current_task = super(BuildWellBehaviour, self).do_step()

        if self._current_task is not None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self._current_task.task)
            self.action_build_well(self._current_task.task)


class BuildUpWellBehaviour(BuildWellBehaviour):
    """
    Builds up integrity of a well
    """

    def action_build_well(self, well_type):
        """
        Overwrides method from BuildWellBehaviour, ommits the parameters
        :param well_type:
        :return:
        """
        self._action_provider.send_action(Action.BUILD)