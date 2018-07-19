from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import TaskProgress, TaskStop

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.current_task import CurrentTaskDecision
from provider.action_provider import Action
from provider.action_provider import ActionProvider
from provider.product_provider import ProductProvider
from rhbp_selforga.behaviours import DecisionBehaviour
from rospy.my_publish_subscribe import MyPublisher, MySubscriber

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class AssembleProductBehaviour(DecisionBehaviour):
    """
    Behaviour, that allows to execute the assemble and assist_assemble commands
    """

    def __init__(self, mechanism, name, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(mechanism=mechanism, name=name,
                      requires_execution_steps=True,
                      **kwargs)
        # Constructor parameters
        self._agent_name = agent_name

        self._task = None
        self._last_action = None
        self._last_target = None

        # Init providers
        self._product_provider = ProductProvider(agent_name=agent_name)
        self.action_provider = ActionProvider(agent_name=agent_name)

        # Dictionary, that keeps track of all assembly progresses.
        # We not only track our own, to avoid missing out on information, when the assembly behaviour did not
        # get the information in time from the mechanism.
        self._task_progress_dict = {}

        # Initialise subscribers and publishers
        topic = AgentUtils.get_coordination_topic()
        MySubscriber(topic, message_type="progress", task_type=CurrentTaskDecision.TYPE_ASSEMBLE,
                     callback=self._callback_task_progress)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)
        self._pub_assemble_progress = MyPublisher(topic, message_type="progress",
                                                  task_type=CurrentTaskDecision.TYPE_ASSEMBLE, queue_size=10)
        self._pub_assemble_stop = MyPublisher(topic, message_type="stop", task_type=CurrentTaskDecision.TYPE_ASSEMBLE,
                                              queue_size=10)

    def _callback_task_progress(self, task_progres):
        """
        Saves the progress of all assemble tasksin a dictionary
        :param task_progres:
        :type task_progres: TaskProgress
        :return:
        """
        if task_progres.type == CurrentTaskDecision.TYPE_ASSEMBLE:
            self._task_progress_dict[task_progres.id] = task_progres.step

    def _action_request_agent(self, agent):
        """
        Updates the assemble task after assembly
        :param agent:
        :type agent: Agent
        :return:
        """
        # if our last task was assemble and the last_action in the simulation was assemble too
        if self._last_action == "assemble" and agent.last_action == "assemble":
            ettilog.loginfo("AssembleProductBehaviour(%s):: Last assembly: %s", self._agent_name, agent.last_action_result)

            if agent.last_action_result == "failed_capacity":
                # Only in a few edge cases this can happen. Generally we can ignore it and pretend it was succesful
                ettilog.logerr("AssembleProductBehaviour(%s)::Failed capacity", self._agent_name)
                agent.last_action_result = "successful"

            if agent.last_action_result == "successful":
                # Update the assemble step of the assemble task
                next_assemble_step = self._get_assemble_step() + 1

                step = self._get_assemble_step()
                total_steps = len(self._task.task.split(",")) -1
                if step < total_steps:
                    # If there are still tasks to do, inform all agents that the next task will be performed
                    ettilog.logerr(
                        "AssembleProductBehaviour(%s):: Assembled item %d/%d, going on to next task ....",
                        self._agent_name, step+1, total_steps +1)
                    assemble_task_coordination = TaskProgress(
                        id=self._task.id,
                        step=next_assemble_step,
                        type=CurrentTaskDecision.TYPE_ASSEMBLE
                    )
                    self._pub_assemble_progress.publish(assemble_task_coordination)
                else:
                    # If this was the last task -> notify all contractors to end task
                    ettilog.logerr(
                        "AssembleProductBehaviour(%s):: Assembled item %d/%d, stopping assembly",
                        self._agent_name, step+1, total_steps+1)
                    self._pub_assemble_stop.publish(TaskStop(id=self._task.id, reason="assembly finished"))

    def action_assemble(self, item_to_assemble):
        """
        Performs the assemble action to assemble one item
        :param item_to_assemble:
        :type item_to_assemble: str
        :return:
        """
        self.action_provider.send_action(action_type=Action.ASSEMBLE, params=[
            KeyValue("item", item_to_assemble)])

    def action_assist_assemble(self, agent_name):
        """
        Performs the assist_assemble action to help another agent assemble one item
        :param agent_name: The agent to support
        :type agent_name: str
        :return:
        """
        self.action_provider.send_action(action_type=Action.ASSIST_ASSEMBLE, params=[
            KeyValue("Agent", agent_name)])

    def do_step(self):
        """
        Each step get the current assembly task, and perform the assigned action of the current step
        :return:
        """
        self._task = super(AssembleProductBehaviour, self).do_step()
        if self._task is None:
            ettilog.logerr("AssembleProductBehaviour:: ERROR: Assembly task not available during assembly.")
            return

        # Action list contains all assemble, and assist_assemble actions of a task
        action_list = self._task.task.split(",")

        # Update the goals
        self._product_provider.update_assembly_goal(self._task.task, self._get_assemble_step())

        # If there is an action for the current step, perform it
        if len(action_list) > 0 and self._get_assemble_step() < len(action_list):
            (self._last_action, self._last_target) = action_list[self._get_assemble_step()].split(":")
            if self._last_action == "assemble":
                ettilog.logerr("AssembleProductBehaviour(%s):: step %d/%d Assembling %s", self._agent_name,
                               self._get_assemble_step() + 1, len(action_list), self._last_target)
                self.action_assemble(self._last_target)
            elif self._last_action == "assist":
                self.action_assist_assemble(self._last_target)
            else:
                ettilog.logerr("AssembleProductBehaviour(%s):: ERROR: Invalid action", self._agent_name)

        else:
            ettilog.logerr("AssembleProductBehaviour:: ERROR: Assembly is executed after all tasks are finished")

    def _get_assemble_step(self):
        """
        Returns the current step of the assigned task
        :return: int
        """
        return self._task_progress_dict.get(self._task.id, 0)
