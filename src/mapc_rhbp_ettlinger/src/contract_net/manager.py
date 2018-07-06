import random
import time
from abc import abstractmethod

from mapc_rhbp_ettlinger.msg import TaskStop

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from rospy.my_publish_subscribe import MyPublisher, MySubscriber

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.coordination.manager')


class ContractNetManager(object):
    DEADLINE_BIDS = 4
    DEADLINE_ACKNOWLEDGEMENT = 4

    def __init__(self, task_type):

        self._task_type = task_type

        self._id = random.randint(1,1000000)
        self._bids = None
        self._acknowledgements = None
        self._assignments = None

        self.reset_manager()

        self._facility_provider = FacilityProvider()

        topic = AgentUtils.get_coordination_topic()

        self._pub_task_request = MyPublisher(topic, message_type="request", task_type=self._task_type, queue_size=10)
        MySubscriber(topic, message_type="bid", task_type=self._task_type, callback=self._callback_bid)
        self._pub_task_assignment = MyPublisher(topic, message_type="assignment", task_type=self._task_type, queue_size=10)
        MySubscriber(topic, message_type="acknowledgement", task_type=self._task_type, callback=self._callback_acknowledgement)
        self._pub_task_stop = MyPublisher(topic, message_type="stop", task_type=self._task_type, queue_size=10)
        MySubscriber(topic, message_type="stop", task_type=self._task_type, callback=self._on_task_finished)

    def reset_manager(self):
        self._bids = []
        self._acknowledgements = []
        self._assignments = []
        self._id = self._id + 1

    def request_help(self, request):

        ettilog.loginfo("ContractNetManager(%s):: ---------------------------Manager start---------------------------", self._task_type)
        self.reset_manager()

        request.id = self._id
        request.deadline = time.time() + ContractNetManager.DEADLINE_BIDS

        self._pub_task_request.publish(request)

        sleep_time = request.deadline - time.time()
        time.sleep(sleep_time)
        self.process_bids()

    def _callback_bid(self, bid):

        if bid.request.deadline < time.time():
            ettilog.loginfo("ContractNetManager(%s):: Bid with expired deadline from %s (id:%d, bid-id:%d)", self._task_type, bid.agent_name, self._id, bid.id)
        elif bid.id != self._id:
            ettilog.loginfo("ContractNetManager(%s):: Received wrong id by %s (id:%d, bid-id:%d)", self._task_type, bid.agent_name, self._id, bid.id)
        else:
            ettilog.loginfo("ContractNetManager(%s):: Received bid from %s (id:%d)", self._task_type, bid.agent_name, self._id)
            self._bids.append(bid)

    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        assignments = self.get_assignments(self._bids)

        if assignments is None or len(assignments) == 0:
            ettilog.loginfo("ContractNetManager(%s):: No useful bid combination found in %d bids", self._task_type, len(self._bids))
            return

        ettilog.loginfo("ContractNetManager(%s):: Bids processed: Accepted bids from %s", self._task_type,
                        ", ".join([assignment.bid.agent_name for assignment in assignments]))

        acknowledgement_deadline = time.time() + ContractNetManager.DEADLINE_ACKNOWLEDGEMENT

        for assignment in assignments:
            assignment.deadline = acknowledgement_deadline
            self._assignments.append(assignment)

            ettilog.loginfo("ContractNetManager(%s):: Publishing assignment for %s", self._task_type, assignment.bid.agent_name)

            self._pub_task_assignment.publish(assignment)

        time.sleep(acknowledgement_deadline - time.time())

        self._process_bid_acknowledgements()

    def _callback_acknowledgement(self, acknowledgement):

        if acknowledgement.id != self._id:
            ettilog.loginfo("ContractNetManager(%s):: Received Acknowledgement with wrong id by %s (id:%d, bid-id:%d)", self._task_type, acknowledgement.assignment.bid.agent_name, self._id, acknowledgement.id)
            return
        else:
            ettilog.loginfo("ContractNetManager(%s):: Received Acknowledgement from %s", self._task_type,
                            acknowledgement.assignment.bid.agent_name)
            self._acknowledgements.append(acknowledgement)

    def _process_bid_acknowledgements(self):
        if len(self._acknowledgements) == len(self._assignments):
            ettilog.logerr("ContractNetManager(%s):: Task (%d) starting with %s", self._task_type,
                           self._id, str([assignment.bid.agent_name for assignment in self._assignments]))

            self._on_task_acknowledged()
        else:
            ettilog.loginfo("ContractNetManager(%s):: coordination unsuccessful. cancelling... Received %d/%d from %s", self._task_type,
                            len(self._acknowledgements), len(self._assignments),
                            str([acknowledgement.assignment.bid.agent_name for acknowledgement in self._acknowledgements]))

            self._pub_task_stop.publish(TaskStop(
                id=self._id,
                reason='acknowledgements failed'))

        ettilog.loginfo("ContractNetManager(%s):: ---------------------------Manager stop---------------------------", self._task_type)

    @abstractmethod
    def get_assignments(self, _bids):
        return []

    def _on_task_acknowledged(self):
        pass

    def _on_task_finished(self, task_stop):
        pass
