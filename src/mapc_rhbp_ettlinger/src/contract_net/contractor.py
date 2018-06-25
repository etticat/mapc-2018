import time
from abc import abstractmethod

import rospy
from mapc_rhbp_ettlinger.msg import TaskRequest, TaskBid, TaskAcknowledgement, TaskAssignment, \
    TaskStop, Task

import utils.rhbp_logging
from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.contractor')


class ContractNetContractorBehaviour(BehaviourBase):

    def __init__(self, agent_name, name, task_type, **kwargs):

        super(ContractNetContractorBehaviour, self).__init__(name, **kwargs)
        self._task_knowledge_base = TaskKnowledgeBase()
        self._agent_name = agent_name
        self._current_task = None
        self._task_type = task_type

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        prefix = AgentUtils.get_coordination_prefix(self._task_type)

        rospy.Subscriber(prefix + "request", TaskRequest, self._callback_request)
        self._pub_bid = rospy.Publisher(prefix + "bid", TaskBid, queue_size=10)
        rospy.Subscriber(prefix + "assign", TaskAssignment, self._callback_assign)
        self._pub_acknowledge = rospy.Publisher(prefix + "acknowledge", TaskAcknowledgement, queue_size=10)
        rospy.Subscriber(prefix + "stop", TaskStop, self._on_task_finished)

    def _callback_request(self, request):
        """

        :param request:
        :type request: TaskRequest
        :return:
        """
        ettilog.loginfo("Contractor(%s-%s):: Got request: active:  %s", self._agent_name, self._task_type, str(self._active))
        if self.computeSatisfaction() < 0.8: # TODO: #86 Workaround: I can't seem to get multiple behaviours running
            return

        current_time = time.time()
        if request.deadline < current_time:
            ettilog.logerr("Contractor(%s-%s):: Deadline over %f - %f", self._agent_name, self._task_type, request.deadline, current_time)
            return

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

            accepted = self._task_knowledge_base.create_task(Task(
                id=assignment.bid.id,
                type=self._task_type,
                agent_name=self._agent_name,
                pos=assignment.bid.request.destination,
                task=assignment.tasks
            ))
            if accepted:
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

    @abstractmethod
    def _on_task_finished(self, finish):
        pass

    @abstractmethod
    def bid_possible(self, bid):
        pass
