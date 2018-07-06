import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.current_task = None
        self._agent_name = agent_name
        self._task_knowledge_base = TaskKnowledgeBase()
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_deliver_job(self, job):
        action = GenericAction()
        action.action_type = Action.DELIVER_JOB
        action.params = [
            KeyValue("Job", str(job))]

        self.action_provider.send_action(action)

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
            self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=TaskKnowledgeBase.TYPE_DELIVER)
            self.current_task = None

    def stop(self):
        self.current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        if self.current_task is None:
            self.current_task = self._task_knowledge_base.get_task(
                agent_name=self._agent_name, type=TaskKnowledgeBase.TYPE_DELIVER)

        if self.current_task is not None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.task)
            self.action_deliver_job(self.current_task.task)
            # TODO: Check what happens when the delivery fails -> TODO: Pass an acknowledgement object into the mac
            # ros bridge. Once it finishes/fails it notifies the application
