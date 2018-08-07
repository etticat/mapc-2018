import time
from abc import abstractmethod

import rospy
from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAcknowledgement, Task, TaskAssignment, TaskBid, TaskStop

import utils.rhbp_logging
from common_utils.agent_utils import AgentUtils
from my_subscriber import MySubscriber, MyPublisher
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.contractor')


class ContractNetContractorBehaviour(object):
    """
    Base class for all contractors in contract net.
    """

    def __init__(self, agent_name, task_type, current_task_mechanism):

        # Save constructor variables
        self._current_task_mechanism = current_task_mechanism
        self._agent_name = agent_name
        self._task_type = task_type

        # Init runtime variables
        self._current_task_id = None

        # Init providers
        self._product_provider = ProductProvider(agent_name=self._agent_name)

        # Init subscribers and publishers
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

        should_bid = self.should_bid_for_request(request)
        if should_bid:
            self.send_bid(request)

    @abstractmethod
    def should_bid_for_request(self, request):
        """
        Method decides if an agent should bid for a task request.
        Has to be overwritten
        :param request:
        :return:
        """

    def send_bid(self, request):
        """
        Sending a bid for a request
        :param request:
        :return:
        """

        # Generate bid
        bid = self.generate_bid(request)

        if bid is None:
            return

        #publish bid
        ettilog.loginfo("Contractor(%s-%s):: Publishing bid: %s", self._agent_name, self._task_type, str(bid is not None))
        self._pub_bid.publish(bid)

        # Saving the id to make sure we only accept assignemnts for tasks that we bid for
        self._current_task_id = request.id

    def _callback_assign(self, assignment):
        """
        Callback for assignment from the manager
        :param assignment:
        :return:
        """

        # Only accept assignments, that are directed to the agent itself
        if assignment.bid.agent_name != self._agent_name:
            return

        # Only accept assignments that match the task id, which the contractor bid on
        if self._current_task_id != assignment.bid.id:
            return

        ettilog.loginfo("Contractor(%s-%s):: Received assignment for %s", self._agent_name, self._task_type,
                        assignment.bid.id)

        # Checks if the bid is still possible after assignment
        is_still_possible = self.bid_possible(assignment.bid)

        if is_still_possible:

            # Create a task
            task = Task(
                id=assignment.bid.id,
                type=self._task_type,
                agent_name=self._agent_name,
                items=assignment.items,
                pos=assignment.bid.request.destination,
                destination_name=assignment.bid.request.destination_name,
                task=assignment.tasks
            )

            # notify the current task mechanism
            self._current_task_mechanism.start_task(task)

            # allow the overrwirding manager to execute additional code after assignemnt
            self._on_assignment_confirmed(assignment)

            # Publish acknowledgement for manager
            acknowledgement = TaskAcknowledgement(
                id=assignment.id,
                assignment=assignment
            )
            self._pub_acknowledge.publish(acknowledgement)

    @abstractmethod
    def generate_bid(self, request):
        """
        Returns the bid for a request.
        Has to be overwritten
        :param request:
        :type request: TaskRequest
        :return:
        """
        pass

    def _on_assignment_confirmed(self, assignment):
        """
        Is called after the task is successfully saved.
        May be overwritten to execute code on assignment.
        :param assignment:
        :return:
        """
        pass

    def _on_task_finished(self, finish):
        """
        callback method for when a task ends. This can happen when an error occurs, or when all parts of the task
        are done.
        :param finish:
        :return:
        """
        if self._current_task_mechanism.value is not None and self._current_task_mechanism.value.id == finish.id:
            self._current_task_mechanism.end_task()

    @abstractmethod
    def bid_possible(self, bid):
        """
        Method returns if the bid is possible.
        Has to be overwritten
        :param bid:
        :return:
        """
        pass
