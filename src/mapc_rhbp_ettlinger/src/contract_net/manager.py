import random
import time
from abc import abstractmethod

import rospy
from mac_ros_bridge.msg import SimStart
from mapc_rhbp_ettlinger.msg import TaskStop, TaskRequest, TaskBid, TaskAssignment, TaskAcknowledgement

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from my_subscriber import MySubscriber, MyPublisher
from provider.facility_provider import FacilityProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.coordination.manager')


class ContractNetManager(object):
    """
    Contract Net manager base class
    """
    DEADLINE_BIDS = 4
    DEADLINE_ACKNOWLEDGEMENT = 4

    def __init__(self, task_type, agent_name):

        self._task_type = task_type

        self._init_config()

        # Initialise runtime variables
        self._id = 0  # Id of the current coordinated task
        self._bids = None
        self._acknowledgements = None
        self._assignments = None
        self.reset_manager()
        self.enabled = True

        # Initialise providers
        self._facility_provider = FacilityProvider(agent_name=agent_name)

        # Initialise subscribers
        topic = AgentUtils.get_coordination_topic()
        self._pub_task_request = MyPublisher(topic, message_type="request", task_type=self._task_type, queue_size=10)
        MySubscriber(topic, message_type="bid", task_type=self._task_type, callback=self._callback_bid)
        self._pub_task_assignment = MyPublisher(topic, message_type="assignment", task_type=self._task_type,queue_size=10)
        MySubscriber(topic, message_type="acknowledgement", task_type=self._task_type,
                     callback=self._callback_acknowledgement)
        self._pub_task_stop = MyPublisher(topic, message_type="stop", task_type=self._task_type, queue_size=10)
        MySubscriber(topic, message_type="stop", task_type=self._task_type, callback=self._on_task_finished)

        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):
        """
        Initialises the config parameters from the rospy config
        :return:
        """
        ContractNetManager.DEADLINE_BIDS = rospy.get_param('ContractNetManager.DEADLINE_BIDS',
                                                           ContractNetManager.DEADLINE_BIDS)
        ContractNetManager.DEADLINE_ACKNOWLEDGEMENT = rospy.get_param('ContractNetManager.DEADLINE_ACKNOWLEDGEMENT',
                                                                      ContractNetManager.DEADLINE_ACKNOWLEDGEMENT)

    def reset_manager(self):
        """
        Resets all values, so the manager can start a new task
        :return:
        """
        self._bids = []
        self._acknowledgements = []
        self._assignments = []
        self._id = self._id + 1

    def request_help(self, request):
        """
        Starts the coordination of a task.
        This method blocks until the coordination is done or has failed
        :param request:
        :type request: TaskRequest
        :return:
        """

        # Return if the manager got disabled
        if not self.enabled:
            return False

        ettilog.logerr("ContractNetManager(%s):: ---------------------------Manager start---------------------------",
                        self._task_type)

        self.start_time = rospy.get_rostime()

        self.reset_manager()

        # Inject a new Id and the deadline for the request
        request.id = self._id
        request.deadline = time.time() + ContractNetManager.DEADLINE_BIDS

        # Publish the request
        self._pub_task_request.publish(request)

        # Wait until deadline to process all bids
        sleep_time = request.deadline - time.time()
        time.sleep(sleep_time)
        return self.process_bids()

    def _callback_bid(self, bid):
        """
        Takes a bid from an agent and saves it into
        :param bid:
        :return:
        """

        if bid.request.deadline < time.time():
            # Ignore bids after deadline
            ettilog.logerr("ContractNetManager(%s):: Bid with expired deadline from %s (id:%d, bid-id:%d)",
                           self._task_type, bid.agent_name, self._id, bid.id)
            return
        if bid.id != self._id:
            # Ignore bids, that do not match the current id. This could happen if the agent took soo long to send bid,
            # that the manager already started coordinating the next task when the bid arrives.
            ettilog.logerr("ContractNetManager(%s):: Received invalid id by %s (id:%d, bid-id:%d)", self._task_type,
                           bid.agent_name, self._id, bid.id)
            return

        # Add bid to bid array
        ettilog.loginfo("ContractNetManager(%s):: Received bid from %s (id:%d)", self._task_type, bid.agent_name,
                        self._id)
        self._bids.append(bid)

    def process_bids(self):
        """
        Processes all bids, finds the best agents and sends acknowledgements to those agents.
        :return:
        """

        # Return if the manager got disabled in the meantime
        if not self.enabled:
            return False
        ettilog.logerr("ContractNetManager(%s):: Processing %d bids", self._task_type, len(self._bids))

        # Retrive the assignments. This method has to be implemented by an overriding class.
        assignments = self.get_assignments(self._bids)

        # If no assignments could be created, stop coordination
        if assignments is None or len(assignments) == 0:
            ettilog.loginfo("ContractNetManager(%s):: No useful bid combination found in %d bids", self._task_type,
                            len(self._bids))
            time_passed = (rospy.get_rostime() - self.start_time).to_sec()
            ettilog.logerr(
                "ContractNetManager(%s):: ---------------------------Manager failed: %.2f seconds---------------------------",
                self._task_type, time_passed)
            return False

        ettilog.loginfo("ContractNetManager(%s):: Bids processed: Accepted bids from %s", self._task_type,
                        ", ".join([assignment.bid.agent_name for assignment in assignments]))

        acknowledgement_deadline = time.time() + ContractNetManager.DEADLINE_ACKNOWLEDGEMENT

        # publish all assignments
        for assignment in assignments:
            assignment.deadline = acknowledgement_deadline
            self._assignments.append(assignment)

            ettilog.loginfo("ContractNetManager(%s):: Publishing assignment for %s", self._task_type,
                            assignment.bid.agent_name)
            self._pub_task_assignment.publish(assignment)

        # Wait until deadline and process the acknowledgements
        time.sleep(acknowledgement_deadline - time.time())
        return self._process_acknowledgements()

    def _callback_acknowledgement(self, acknowledgement):
        """
        Receives an acknolwedgement from an agent and saves it into an array.
        :param acknowledgement:
        :return:
        """

        if acknowledgement.assignment.deadline < time.time():
            # Ignore acknowledgements after deadline
            ettilog.logerr("ContractNetManager(%s):: Acknowledgement with expired deadline from %s (id:%d, bid-id:%d)",
                           self._task_type, acknowledgement.assignment.bid.agent_name, self._id, acknowledgement.id)
            return
        if acknowledgement.id != self._id:
            # Ignore acknowledgements, that do not match the current id. This could happen if the agent took soo long to send acknowledgement,
            # that the manager already started coordinating the next task when the bid arrives.
            ettilog.logerr("ContractNetManager(%s):: Received invalid acknowledgement by %s (id:%d, bid-id:%d)",
                           self._task_type,
                           acknowledgement.assignment.bid.agent_name, self._id, acknowledgement.id)
            return

        ettilog.loginfo("ContractNetManager(%s):: Received Acknowledgement from %s", self._task_type,
                        acknowledgement.assignment.bid.agent_name)
        self._acknowledgements.append(acknowledgement)

    def _process_acknowledgements(self):
        """
        Processes all acknowledgements. Checks if all were received succesfully. If not, it cancells the task again.
        :return:
        """

        # Return if the manager got disabled in the meantime
        if not self.enabled:
            return False

        if len(self._acknowledgements) == len(self._assignments):
            # If all agents acknowledged the assignments, end coordination
            ettilog.logerr("ContractNetManager(%s):: Task (%d) starting with %s", self._task_type,
                           self._id, str([assignment.bid.agent_name for assignment in self._assignments]))

            self._on_task_acknowledged(self._acknowledgements[0].id)
            time_passed = (rospy.get_rostime() - self.start_time).to_sec()
            ettilog.logerr(
                "ContractNetManager(%s):: ---------------------------Manager stop: %.2f seconds---------------------------",
                self._task_type, time_passed)
            return True
        else:
            # If not all assignments got acknowledged, Stop the whole task and end coordination.
            ettilog.logerr("ContractNetManager(%s):: coordination unsuccessful. cancelling... Received %d/%d from %s",
                           self._task_type,
                           len(self._acknowledgements), len(self._assignments),
                           str([acknowledgement.assignment.bid.agent_name for acknowledgement in
                                self._acknowledgements]))

            self._pub_task_stop.publish(TaskStop(id=self._id, reason='acknowledgements failed'))

            time_passed = (rospy.get_rostime() - self.start_time).to_sec()
            ettilog.logerr(
                "ContractNetManager(%s):: ---------------------------Manager stop: %.2f seconds---------------------------",
                self._task_type, time_passed)
            return False

    @abstractmethod
    def get_assignments(self, _bids):
        """
        Returns assignments from bids. Has to be overwritten.
        :param _bids:
        :return:
        """
        return []

    def _on_task_acknowledged(self, task_id):
        """
        callback method, when a task was successfully coordinated
        :param task_id:
        :return:
        """
        pass

    def _on_task_finished(self, task_id):
        """
        Callback method, for when a task is finished. either through error or by completing it.
        :param task_id:
        :type task_id: TaskStop
        :return:
        """
        pass
