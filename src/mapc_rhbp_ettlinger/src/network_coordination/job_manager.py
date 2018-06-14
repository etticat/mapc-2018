import random
import time

import rospy
from mac_ros_bridge.msg import Position, Job
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAssignment, JobAcknowledgement

import utils.rhbp_logging
from agent_knowledge.tasks import JobKnowledgebase
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_manager')


class JobManager(object):

    DEADLINE_BIDS = 2
    DEADLINE_ACKNOLEDGEMENT = 2

    def __init__(self):


        self._product_provider = ProductProvider(agent_name="agentA1") # temporarily just use any name
        # TODO: Make productProvider independent from agent

        self._job_knowledgebase = JobKnowledgebase()
        self._facility_provider = FacilityProvider()

        self.id = self.job_id(new_id=True)

        self.bids = []
        self.acknowledgements = []

        self._pub_job_request = rospy.Publisher(AgentUtils.get_job_prefix() + "request", JobRequest,
                                                     queue_size=10)
        rospy.Subscriber(AgentUtils.get_job_prefix() + "bid", JobBid, self._callback_bid)
        self._pub_job_assignment = rospy.Publisher(AgentUtils.get_job_prefix() + "assign", JobAssignment,
                                                        queue_size=10)
        rospy.Subscriber(AgentUtils.get_job_prefix() + "acknowledge", JobAcknowledgement,
                         self._callback_acknowledgement)

    def job_request(self, job):
        """
        The first step of the protocol
        :param job:
        :type job: Job
        :return:
        """
        self.bids = []
        self.acknowledgements = []

        ettilog.logerr("JobManager:: ---------------------------Manager start---------------------------",)

        request = JobRequest(
            id=self.job_id(),
            deadline =time.time() + JobManager.DEADLINE_BIDS,
            items=job.items
        )
        self._pub_job_request.publish(request)
        self._job = job

        sleep_time = request.deadline - time.time()
        ettilog.loginfo("JobManager:: sleeping for %s", str(sleep_time))
        time.sleep(sleep_time)
        self.process_bids()

    def job_id(self, new_id=False):
        if new_id:
            self.id = "task-" + str(time.time() %512)
        return self.id

    def _callback_bid(self, bid):
        ettilog.loginfo("JobManager(%s): Received bid from %s", bid.agent_name)
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

        ettilog.loginfo("JobManager(%s, %s): Processing %s bids", self.job_id(), str(len(self.bids)))

        self.assignments = self._product_provider.choose_best_job_bid_combination(self._job, self.bids) # TODO

        if self.assignments == None:
            ettilog.logerr("JobManager:: No useful bid combination found in %d bids", len(self.bids))
            bids, roles = self._product_provider.get_items_and_roles_from_bids(self.bids)
            ettilog.logerr("JobManager:: ------ Items: %s", str(bids))
            ettilog.logerr("JobManager:: ------ Agents: %s", str([bid.agent_name for bid in self.bids]))
            return
        else:
            ettilog.logerr("JobManager:: Bids processed: Accepted bids from %s  assignments: %s", ", ".join([bid.agent_name for bid in self.assignments]), str(self.assignments))


        deadline = time.time() + JobManager.DEADLINE_ACKNOLEDGEMENT

        for assignment in self.assignments:
            assignment.pos = self._facility_provider.get_storage_by_name(self._job.storage_name).pos
            ettilog.loginfo("JobManager:: Publishing assignment for %s (%s)", assignment.agent_name, str(assignment.assigned))
            self._pub_job_assignment.publish(assignment)

        time.sleep(deadline - time.time())

        self._process_bid_acknoledgements()

    def _callback_acknowledgement(self, acknowledgement):
        ettilog.loginfo("JobManager:: Received Acknowledgement from %s", acknowledgement.agent_name)
        if acknowledgement.id != self.job_id():
            ettilog.loginfo("wrong id")
            return
        if acknowledgement.acknowledged == False:
            ettilog.loginfo("Contractor rejected acknowledgement")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        ettilog.loginfo("JobManager:: Processing Acknowledgements. Received %d/%d from %s", len(self.acknowledgements), len(self.assignments), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

        if len(self.acknowledgements) == len(self.assignments):
            ettilog.logerr("JobManager:: coordination successful. work can start with %d agents", len(self.assignments))
        else:
            ettilog.logerr("JobManager:: coordination unsuccessful. cancelling... Received %d/%d from %s", len(self.acknowledgements), len(self.assignments), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

            # TODO: Delete from db
            self._job_knowledgebase.end_job_task(
                job_id=self._job.id,
                agent_name="*")
            for bid in self.bids:
                    assignment = JobAssignment(
                        assigned = False,
                        deadline=0,
                        bid = bid,
                        job_id=self._job.id
                    )
                    self._pub_job_assignment.publish(assignment)

        self.id = self.job_id(new_id=True)

        ettilog.logerr("JobManager:: ---------------------------Manager stop---------------------------")