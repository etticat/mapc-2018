import rospy
from mac_ros_bridge.msg import SimEnd
from mapc_rhbp_ettlinger.msg import TaskStop

from common_utils.agent_utils import AgentUtils
from my_subscriber import MyPublisher
from provider.product_provider import ProductProvider
from so_data.patterns import DecisionPattern


class CurrentTaskDecision(DecisionPattern):
    """
    Decision mechanism that keeps track of the current task and returns it in calc
    """

    TYPE_ASSEMBLE = "assemble"
    TYPE_DELIVER = "deliver"
    TYPE_BUILD_WELL = "build_well"

    def __init__(self, agent_name, task_type):
        self.agent_name = agent_name
        self.task_type = task_type

        super(CurrentTaskDecision, self).__init__(buffer=None, frame=None, requres_pos=False, value=None)

        self._pub_task_stop = MyPublisher(AgentUtils.get_coordination_topic(), message_type="stop", task_type=task_type,
                                          queue_size=10)

        # Reset variables when simulation ends
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd,
                         self.callback_simulation_end)

    def calc_value(self):
        """
        Returns the current task
        :return:
        """
        return [self.value, self.state]

    def start_task(self, task):
        """
        Saves the current task.
        Can be overwritten to perform code on task start
        :param task:
        :return:
        """
        assert self.value is None
        rospy.loginfo("CurrentTaskDecision(%s)::starting task %s current state: %s", self.task_type, task.id,
                      self.value)
        self.value = task

    def end_task(self, notify_others=True):
        """
        Stops the current task.
        Can be overwritten to perform code on task end
        :param notify_others: If set to True, all other agents with the same receive a message to cancel their task too
        :return:
        """
        if self.value is not None and notify_others:
            self._pub_task_stop.publish(TaskStop(
                id=self.value.id,
                reason='stopped by client'))

        self.value = None

    def callback_simulation_end(self, sim_end=None):
        """
        Reset task when simulation ends
        :return:
        """
        self.end_task(notify_others=False)

    def has_task(self):
        return self.value is not None


class AssembleTaskDecision(CurrentTaskDecision):
    """
    Decision mechanism that keeps track of the assemble current task and returns it in calc
    """

    def __init__(self, agent_name, task_type):
        super(AssembleTaskDecision, self).__init__(agent_name, task_type)

        self._product_provider = ProductProvider(agent_name=agent_name)

    def end_task(self, notify_others=True):
        """
        Stops the current assembly task.
        Also clears the assembly goal
        :param notify_others: If set to True, all other agents with the same receive a message to cancel their task too
        :return:
        """
        super(AssembleTaskDecision, self).end_task(notify_others)

        rospy.logerr("AssembleTaskDecision(%s):: Ending task (notify=%s) Task: %s", self.agent_name,
                     str(notify_others), str(self.value))

        if self.value is None:
            self._product_provider.stop_assembly()

    def start_task(self, task):
        """
        Saves the current assembly task.
        Adds the expected items into the assembly goal
        :param task:
        :return:
        """
        super(AssembleTaskDecision, self).start_task(task)
        self._product_provider.update_assembly_goal(task_string=task.task)


class DeliveryTaskDecision(CurrentTaskDecision):
    """
    Decision mechanism that keeps track of the current delivery task and returns it in calc
    """

    def __init__(self, agent_name, task_type):
        super(DeliveryTaskDecision, self).__init__(agent_name, task_type)

        self._product_provider = ProductProvider(agent_name=agent_name)

    def end_task(self, notify_others=True):
        """
        Stops the current delivery task.
        Also clears the delivery goal
        :param notify_others: If set to True, all other agents with the same receive a message to cancel their task too
        :return:
        """
        rospy.logerr("CurrentTaskDecision(%s)::stopping task: %s notify: %r", self.task_type, str(self.value),
                     notify_others)
        if self.value is not None:
            self._product_provider.stop_delivery(job_id=self.value.task,
                                                 storage=self.value.destination_name)
        super(DeliveryTaskDecision, self).end_task(notify_others)

    def start_task(self, task):
        """
        Saves the current delivery task.
        Adds the expected items into the delivery goal
        :param task:
        :return:
        """
        super(DeliveryTaskDecision, self).start_task(task)
        self._product_provider.update_delivery_goal(item_list=self.value.items, job_id=self.value.task,
                                                    storage=self.value.destination_name)


class WellTaskDecision(CurrentTaskDecision):

    def destination_not_found(self):
        """
        Is called when destination is unreachable. Just end task
        :return:
        """
        self.end_task()
