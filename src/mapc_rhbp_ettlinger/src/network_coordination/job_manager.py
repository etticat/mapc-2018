import random
import time

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAssignment, JobAcknowledgement

import utils.rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_manager')


class JobManager(object):

    DEADLINE_BIDS = 2
    DEADLINE_ACKNOLEDGEMENT = 2

    def __init__(self, agent_name):

        self._agent_name = agent_name

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        self.id = self.job_id(new_id=True)

        self.bids = []
        self.accepted_bids = []
        self.acknowledgements = []

        self._pub_job_request = rospy.Publisher(AgentUtils.get_job_prefix() + "request", JobRequest,
                                                     queue_size=10)
        rospy.Subscriber(AgentUtils.get_job_prefix() + "bid", JobBid, self._callback_bid)
        self._pub_job_assignment = rospy.Publisher(AgentUtils.get_job_prefix() + "assign", JobAssignment,
                                                        queue_size=10)
        rospy.Subscriber(AgentUtils.get_job_prefix() + "acknowledge", JobAcknowledgement,
                         self._callback_acknowledgement)

    def job_request(self):
        """
        The first step of the protocol
        :return:
        """

        self.bids = []
        self.accepted_bids = []
        self.acknowledgements = []

        ettilog.logerr("JobManager(%s):: ---------------------------Manager start---------------------------",
                       self._agent_name)

        request = JobRequest(
            id=self.job_id(),
            deadline =time.time() + JobManager.DEADLINE_BIDS,
            items=[] # TODO
        )
        self._pub_job_request.publish(request)

        sleep_time = request.deadline - time.time()
        ettilog.loginfo("JobManager(%s):: sleeping for %s", self._agent_name, str(sleep_time))
        time.sleep(sleep_time)
        self.process_bids()

    def job_id(self, new_id=False):
        if new_id:
            self.id = self._agent_name + "-" + str(time.time() %512)
        return self.id

    def _callback_bid(self, bid):
        ettilog.loginfo("JobManager(%s): Received bid from %s", self._agent_name, bid.agent_name)
        generated_id = self.job_id()
        if bid.id != generated_id:
            ettilog.logerr("JobManager(%s): wrong id")
            return

        self.bids.append(bid)

    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        ettilog.loginfo("JobManager(%s, %s): Processing %s bids", self._agent_name, self.job_id(), str(len(self.bids)))

        # self.accepted_bids = self._product_provider.choose_best_job_bid_combination(self.bids) # TODO
        self.accepted_bids = self.bids # TODO: Temp: Accept everything

        if len(self.accepted_bids) == 0:
            ettilog.logerr("JobManager(%s): No useful bid combination found in %d bids", self._agent_name, len(self.bids))
            bids, roles = self._product_provider.get_items_and_roles_from_bids(self.bids)
            ettilog.logerr("JobManager(%s): ------ Items: %s", self._agent_name, str(bids))
            ettilog.logerr("JobManager(%s): ------ Agents: %s", self._agent_name, str([bid.agent_name for bid in self.bids]))
            self.accepted_bids = []
        else:
            ettilog.logerr("JobManager(%s): Bids processed: Accepted bids from %s ", self._agent_name, ", ".join([bid.agent_name for bid in self.accepted_bids]))

        rejected_bids = []

        for bid in self.bids:
            if bid not in self.accepted_bids:
                rejected_bids.append(bid)

        ettilog.loginfo("JobManager(%s): Rejecting %s bid(s)", self._agent_name, str(len(rejected_bids)))

        deadline = time.time() + JobManager.DEADLINE_ACKNOLEDGEMENT

        for bid in self.bids:
            if bid in self.accepted_bids:
                assignment = JobAssignment(
                    id = bid.id,
                    agent_name = bid.agent_name,
                    assigned = True,
                    deadline=deadline,
                    items=bid.items # Currently let them bring all items from bid. TODO: only bring neccessary items
                )
            else:
                assignment = JobAssignment(
                    assigned = False,
                    deadline=deadline,
                    bid = bid,
                    jobs = ""
                )

            ettilog.loginfo("JobManager(%s): Publishing assignment for %s (%s)", self._agent_name, bid.agent_name, str(assignment.assigned))
            self._pub_job_assignment.publish(assignment)

        time.sleep(deadline - time.time())

        self._process_bid_acknoledgements()

    def _callback_acknowledgement(self, acknowledgement):
        ettilog.loginfo("JobManager(%s): Received Acknowledgement from %s", self._agent_name, acknowledgement.agent_name)
        if acknowledgement.id != self.job_id():
            ettilog.loginfo("wrong id")
            return
        if acknowledgement.acknowledged == False:
            ettilog.loginfo("Contractor rejected acknowledgement")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        ettilog.loginfo("JobManager(%s): Processing Acknowledgements. Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

        if len(self.acknowledgements) == len(self.accepted_bids):
            ettilog.logerr("JobManager(%s): coordination successful. work can start with %d agents", self._agent_name, len(self.accepted_bids))
        else:
            ettilog.logerr("JobManager(%s): coordination unsuccessful. cancelling... Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

            # TODO: Delete from db
            # self._job_knowledgebase.cancel_job_requests(self.job_id())
            # for bid in self.bids:
            #         assignment = JobAssignment(
            #             assigned = False,
            #             deadline=0,
            #             bid = bid,
            #         )
            #         self._pub_job_assignment.publish(assignment)


        self.id = self.job_id(new_id=True)

        ettilog.logerr("JobManager(%s):: ---------------------------Manager stop---------------------------",
                   self._agent_name)