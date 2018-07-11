import time
from abc import abstractmethod

from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAcknowledgement, Task

import utils.rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from rospy.my_publish_subscribe import MySubscriber, MyPublisher

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.contractor')


class ContractNetContractorBehaviour(object):

    def __init__(self, agent_name, task_type, mechanism, ready_for_bid_condition):

        self.mechanism = mechanism
        self._agent_name = agent_name
        self._current_task = None
        self._task_type = task_type
        self.ready_for_bid_condition = ready_for_bid_condition

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        prefix = AgentUtils.get_coordination_topic()

        MySubscriber(prefix, message_type="request", task_type=self._task_type, callback=self._callback_request)
        self._pub_bid = MyPublisher(prefix, message_type="bid", task_type=self._task_type, queue_size=10)
        MySubscriber(prefix, message_type="assignment", task_type=self._task_type, callback=self._callback_assign)
        self._pub_acknowledge = MyPublisher(prefix, message_type="acknowledgement", task_type=self._task_type, queue_size=10)
        MySubscriber(prefix, message_type="stop", task_type=self._task_type, callback=self._on_task_finished)

    def _callback_request(self, request):
        """

        :param request:
        :type request: TaskRequest
        :return:
        """

        current_time = time.time()
        if request.deadline < current_time:
            ettilog.logerr("Contractor(%s-%s):: Deadline over %f - %f", self._agent_name, self._task_type, request.deadline, current_time)
            return

        self.ready_for_bid_condition.sync()
        self.ready_for_bid_condition.updateComputation()
        if self.ready_for_bid_condition.satisfaction > 0.8:
            self.send_bid(request)

    def send_bid(self, request):

        bid = self.generate_bid(request)

        if bid is None:
            return

        ettilog.loginfo("Contractor(%s-%s):: Sending bid: %s", self._agent_name, self._task_type, str(bid is not None))
        self._pub_bid.publish(bid)

        self._current_task = request.id

    def _callback_assign(self, assignment):

        if assignment.bid.agent_name != self._agent_name:
            return

        if self._current_task != assignment.bid.id:
            return

        ettilog.loginfo("Contractor(%s-%s):: Received assignment for %s", self._agent_name, self._task_type,
                        assignment.bid.id)

        is_still_possible = self.bid_possible(assignment.bid)

        if is_still_possible:

            task = Task(
                id=assignment.bid.id,
                type=self._task_type,
                agent_name=self._agent_name,
                pos=assignment.bid.request.destination,
                task=assignment.tasks
            )
            self.mechanism.start_task(task)
            self._on_assignment_confirmed(assignment)
            acknowledgement = TaskAcknowledgement(
                id=assignment.id,
                assignment=assignment
            )
            self._pub_acknowledge.publish(acknowledgement)

    @abstractmethod
    def generate_bid(self, request):
        """

        :param request:
        :type request: TaskRequest
        :return:
        """
        pass

    @abstractmethod
    def _on_assignment_confirmed(self, assignment):
        pass

    def _on_task_finished(self, finish):
        if self.mechanism.current_task is not None and self.mechanism.current_task.id == finish.id:
            self.mechanism.end_task()

    @abstractmethod
    def bid_possible(self, bid):
        pass
