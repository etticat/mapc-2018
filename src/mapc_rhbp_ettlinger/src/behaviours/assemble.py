
import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent
from mapc_rhbp_ettlinger.msg import TaskProgress, TaskStop

from behaviour_components.behaviours import BehaviourBase
from decisions.p_task_decision import CurrentTaskDecision
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from provider.product_provider import ProductProvider
from rhbp_selforga.behaviours import DecisionBehaviour
from rospy.my_publish_subscribe import MyPublisher, MySubscriber

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class AssembleProductBehaviour(DecisionBehaviour):
    """
    Behaviour for the assembly of a product
    """

    def __init__(self, mechanism, name, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(mechanism=mechanism, name=name,
            requires_execution_steps=True,
            **kwargs)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._task = None
        self._agent_name = agent_name
        self._last_task = None
        self._last_goal = None
        self.action_provider = ActionProvider(agent_name=agent_name)

        self._task_progress_dict = {}

        topic = AgentUtils.get_coordination_topic()
        self._pub_assemble_progress = MyPublisher(topic, message_type="progress", task_type=CurrentTaskDecision.TYPE_ASSEMBLE, queue_size=10)

        MySubscriber(topic, message_type="progress", task_type=CurrentTaskDecision.TYPE_ASSEMBLE, callback=self._callback_task_progress)

        self._pub_assemble_stop = MyPublisher(topic, message_type="stop", task_type=CurrentTaskDecision.TYPE_ASSEMBLE, queue_size=10)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _callback_task_progress(self, task_progress):
        """

        :param task_progress:
        :type task_progress: TaskProgress
        :return:
        """
        if task_progress.type == CurrentTaskDecision.TYPE_ASSEMBLE:
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
                if agent.last_action_result == "failed_capacity":
                    ettilog.logerr("AssembleProductBehaviour(%s)::Failed capacity!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", self._agent_name)
                self._task_progress_dict[self._task.id] = self._task_progress_dict.get(self._task.id, 0) + 1
                if self._get_assemble_step() < len(self._task.task.split(",")):
                    # If there are still tasks to do, inform all others that the next task will be performed
                    ettilog.logerr(
                        "AssembleProductBehaviour(%s):: Finished assembly of product, going on to next task ....",
                        self._agent_name)
                    assemble_task_coordination = TaskProgress(
                        id=self._task.id,
                        step=self._get_assemble_step(),
                        type=CurrentTaskDecision.TYPE_ASSEMBLE
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
        self.action_provider.send_action(action_type = Action.ASSEMBLE, params=[
            KeyValue("item", str(item))])


    def action_assist_assemble(self, agent):
        """
        Performs the assist asemble action and publishes it towards the mac_ros_bridge
        :param agent: str
        :return:
        """
        self.action_provider.send_action(action_type = Action.ASSIST_ASSEMBLE, params=[
            KeyValue("Agent", str(agent))])

    def do_step(self):
        self._task = super(AssembleProductBehaviour, self).do_step()
        assert self._task != None
        products = self._task.task.split(",")
        self._product_provider.update_assembly_goal(self._task.task, self._get_assemble_step())
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

    def start(self):
        super(AssembleProductBehaviour, self).start()

    def _get_assemble_step(self):
        return self._task_progress_dict.get(self._task.id, 0)
