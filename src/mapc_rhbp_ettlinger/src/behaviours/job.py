import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent

from behaviour_components.behaviours import BehaviourBase
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from rhbp_selforga.behaviours import DecisionBehaviour

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class DeliverJobBehaviour(DecisionBehaviour):

    def __init__(self, name, agent_name, mechanism, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(            name=name, mechanism=mechanism, **kwargs)
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.current_task = None
        self._agent_name = agent_name
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_deliver_job(self, job):
        self.action_provider.send_action(action_type=Action.DELIVER_JOB, params=[
            KeyValue("Job", str(job))])

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # TODO: Also add a timeout here: if it doesnt work for 5 steps -> Fail with detailed error
        if self.current_task is not None and agent.last_action == "deliver_job":
            ettilog.logerr("DeliverJobBehaviour(%s):: Deleting own task. Status: %s", agent.last_action_result,
                           agent.last_action_result)
            self.mechanism.end_task()
            self.current_task = None

    def stop(self):
        self.current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        self.current_task = super(DeliverJobBehaviour, self).do_step()
        ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.task)
        self.action_deliver_job(self.current_task.task)
