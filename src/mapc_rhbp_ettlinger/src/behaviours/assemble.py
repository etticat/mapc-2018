import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent
from mapc_rhbp_ettlinger.msg import TaskProgress, TaskStop

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class AssembleProductBehaviour(BehaviourBase):
    """
    Behaviour for the assembly of a product
    """

    def __init__(self, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._task = None
        self._agent_name = agent_name
        self._last_task = None
        self._last_goal = None
        self._task_knowledge_base = TaskKnowledgeBase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

        self._task_progress_dict = {}

        progress_topic = AgentUtils.get_coordination_prefix(TaskKnowledgeBase.TYPE_ASSEMBLE) + "progress"
        self._pub_assemble_progress = rospy.Publisher(progress_topic, TaskProgress, queue_size=10)

        rospy.Subscriber(progress_topic, TaskProgress,
                         self._callback_task_progress)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

        self._pub_assemble_stop = rospy.Publisher(AgentUtils.get_coordination_prefix(TaskKnowledgeBase.TYPE_ASSEMBLE) + "stop",
                                                  TaskStop, queue_size=10)

    def _callback_task_progress(self, task_progress):
        """

        :param task_progress:
        :type task_progress: TaskProgress
        :return:
        """
        if task_progress.type == TaskKnowledgeBase.TYPE_ASSEMBLE:
            self._task_progress_dict[task_progress.id] = task_progress.step

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # TODO: Also add a timeout here: if it doesnt work for 5 steps -> Fail with detailed error
        if self._last_task == "assemble" and agent.last_action == "assemble":
            ettilog.logerr("AssembleProductBehaviour(%s):: Last assembly: %s", self._agent_name,
                           agent.last_action_result)
            if agent.last_action_result in ["successful", "failed_capacity"]:
                self._task_progress_dict[self._task.id] = self._task_progress_dict.get(self._task.id, 0) + 1
                if self._get_assemble_step() < len(self._task.task.split(",")):
                    # If there are still tasks to do, inform all others that the next task will be performed
                    ettilog.logerr(
                        "AssembleProductBehaviour(%s):: Finished assembly of product, going on to next task ....",
                        self._agent_name)
                    assemble_task_coordination = TaskProgress(
                        id=self._task.id,
                        step=self._get_assemble_step(),
                        type=TaskKnowledgeBase.TYPE_ASSEMBLE
                    )
                    self._pub_assemble_progress.publish(assemble_task_coordination)
                else:
                    # If this was the last task -> notify all contractors to end task
                    ettilog.logerr(
                        "AssembleProductBehaviour(%s):: Last product of assembly task assembled, ending assembly",
                        self._agent_name)
                    self._pub_assemble_stop.publish(TaskStop(id=self._task.id, reason="assembly finished"))

    def action_assemble(self, item):
        """
        Specific "goto" action publishing helper function
        :param facility_name: name of the facility we want to go to
        :param publisher: publisher to use
        """
        action = GenericAction()
        action.action_type = Action.ASSEMBLE
        action.params = [
            KeyValue("item", str(item))]

        self._pub_generic_action.publish(action)

    def action_assist_assemble(self, agent):
        """
        Performs the assist asemble action and publishes it towards the mac_ros_bridge
        :param agent: str
        :return:
        """
        action = GenericAction()
        action.action_type = Action.ASSIST_ASSEMBLE
        action.params = [
            KeyValue("Agent", str(agent))]

        self._pub_generic_action.publish(action)

    def start(self):

        self._task = self._task_knowledge_base.get_task(agent_name=self._agent_name,
                                                        type=TaskKnowledgeBase.TYPE_ASSEMBLE)

        super(AssembleProductBehaviour, self).start()

    def do_step(self):
        assert self._task != None
        products = self._task.task.split(",")
        if len(products) > 0 and self._get_assemble_step() < len(products):
            (self._last_task, self._last_goal) = products[self._get_assemble_step()].split(":")
            if self._last_task == "assemble":
                ettilog.logerr("AssembleProductBehaviour(%s):: step %d/%d current task: %s", self._agent_name,
                               self._get_assemble_step() + 1, len(products), products[self._get_assemble_step()])
                self.action_assemble(self._last_goal)
            elif self._last_task == "assist":
                self.action_assist_assemble(self._last_goal)
            else:
                ettilog.logerr("AssembleProductBehaviour(%s):: Invalid task", self._agent_name)

        else:
            ettilog.logerr("This should never happen. Assembly is executed after all tasks are finished")

    def stop(self):
        self._task = None
        super(AssembleProductBehaviour, self).stop()

    def _get_assemble_step(self):
        return self._task_progress_dict.get(self._task.id, 0)
