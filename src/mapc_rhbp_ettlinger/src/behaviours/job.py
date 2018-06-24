import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent

from agent_knowledge.task import TaskBaseKnowledge
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self.current_task = None
        self._agent_name = agent_name
        self._task_knowledge_base = TaskBaseKnowledge()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_deliver_job(self, job):
        action = GenericAction()
        action.action_type = Action.DELIVER_JOB
        action.params = [
            KeyValue("Job", str(job))]

        self._pub_generic_action.publish(action)

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
            self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=TaskBaseKnowledge.TYPE_DELIVER)
            self.current_task = None

    def stop(self):
        self.current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        if self.current_task is None:
            self.current_task = self._task_knowledge_base.get_task(
                agent_name=self._agent_name, type=TaskBaseKnowledge.TYPE_DELIVER)

        if self.current_task is not None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.task)
            self.action_deliver_job(self.current_task.task)
            # TODO: Check what happens when the delivery fails -> TODO: Pass an acknowledgement object into the mac
            # ros bridge. Once it finishes/fails it notifies the application
