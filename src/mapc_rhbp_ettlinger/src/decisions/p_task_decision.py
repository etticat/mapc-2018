import rospy
from mapc_rhbp_ettlinger.msg import TaskStop

from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from rospy.my_publish_subscribe import MyPublisher
from so_data.patterns import DecisionPattern


class CurrentTaskDecision(DecisionPattern):

    TYPE_ASSEMBLE = "assemble"
    TYPE_DELIVER = "deliver"
    TYPE_BUILD_WELL = "build_well"

    def __init__(self, agent_name, task_type):

        self.agent_name = agent_name
        self.task_type = task_type

        super(CurrentTaskDecision, self).__init__(buffer=None, frame=None, requres_pos=False)
        self.current_task = None

        topic = AgentUtils.get_coordination_topic()
        self._pub_task_stop = MyPublisher(topic, message_type="stop", task_type=task_type, queue_size=10)


    def calc_value(self):
        return [self.current_task, self.state]

    def start_task(self, task):
        assert self.current_task is None
        rospy.logerr("CurrentTaskDecision(%s)::starting task %s current state: %s", self.task_type, task.id, self.current_task)
        self.current_task = task

    def end_task(self):
        rospy.logerr("CurrentTaskDecision(%s)::stopping task", self.task_type)
        if self.current_task is not None:
            topic = AgentUtils.get_coordination_topic()
            self._pub_task_stop.publish(TaskStop(
                id=self.current_task.id,
                reason='stopped by client'))


        self.current_task = None


class AssembleTaskDecision(CurrentTaskDecision):

    def __init__(self, agent_name, task_type):
        super(AssembleTaskDecision, self).__init__(agent_name, task_type)
        self._product_provider = ProductProvider(agent_name=agent_name)

    def end_task(self):
        super(AssembleTaskDecision, self).end_task()

        if self.current_task is None:
            self._product_provider.stop_assembly()


class DeliveryTaskDecision(CurrentTaskDecision):

    def __init__(self, agent_name, task_type):
        super(DeliveryTaskDecision, self).__init__(agent_name, task_type)
        self._product_provider = ProductProvider(agent_name=agent_name)

    def end_task(self):

        if self.current_task is not None:
            self._product_provider.stop_delivery(job_id=self.current_task.task, storage=self.current_task.destination_name)
        super(DeliveryTaskDecision, self).end_task()


    def start_task(self, task):
        super(DeliveryTaskDecision, self).start_task(task)
        self._product_provider.update_delivery_goal(item_list=self.current_task.items, job_id=self.current_task.task, storage=self.current_task.destination_name)