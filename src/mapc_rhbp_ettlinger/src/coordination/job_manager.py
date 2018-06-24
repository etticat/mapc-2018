import time

import rospy
from mac_ros_bridge.msg import Job
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAssignment, JobAcknowledgement

from agent_knowledge.task import TaskKnowledgebase
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.job_combination import ChooseBestJobCombination
from provider.facility_provider import FacilityProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.coordination.job_manager')


class JobManager(object):

    DEADLINE_BIDS = 2.5
    DEADLINE_ACKNOLEDGEMENT = 2.5

    def __init__(self):


        self._job_combination = ChooseBestJobCombination()

        self._facility_provider = FacilityProvider()

        self.id = self.job_id(new_id=True)

        self.bids = []
        self.acknowledgements = []
        self.busy = False

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
        self.busy = True
        self.bids = []
        self.acknowledgements = []

        ettilog.loginfo("JobManager:: ---------------------------Manager start---------------------------",)
        ettilog.loginfo("JobManager:: looking for %s to be brought to %s for %d",
                       [item.name + " (" + str(item.amount) + ")" for item in job.items], job.storage_name, job.reward + job.fine)



        request = JobRequest(
            id=self.job_id(),
            deadline =time.time() + JobManager.DEADLINE_BIDS,
            items=CalcUtil.get_list_from_items(job.items)
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
        ettilog.loginfo("JobManager: Received bid from %s", bid.agent_name)
        generated_id = self.job_id()
        if bid.id != generated_id:
            ettilog.loginfo("JobManager: JobManager: wrong id")
            return

        self.bids.append(bid)

    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        ettilog.loginfo("JobManager: Processing %s bids", str(len(self.bids)))

        self.selected_bids = self._job_combination.choose_best_agent_combination(self._job, self.bids) # TODO

        if self.selected_bids == None:
            ettilog.loginfo("JobManager:: No useful bid combination found in %d bids", len(self.bids))
            ettilog.loginfo("JobManager:: ------ Items: %s", str([bid.items for bid in self.bids]))
            ettilog.loginfo("JobManager:: ------ Agents: %s", str([bid.agent_name for bid in self.bids]))

            self.id = self.job_id(new_id=True)
            self.busy = False
            return
        else:
            ettilog.loginfo("JobManager:: Bids processed: Accepted bids from %s  assignments: %s",
                           ", ".join([bid.agent_name for bid in self.bids]),
                           ", ".join([str(bid.items) for bid in self.bids]))

        deadline = time.time() + JobManager.DEADLINE_ACKNOLEDGEMENT

        self.assignments =[]
        for bid in self.selected_bids:
            assignement = JobAssignment(
                id=bid.id,
                agent_name=bid.agent_name,
                assigned=True,
                deadline=0,
                type=TaskKnowledgebase.TYPE_DELIVER,
                pos = self._facility_provider.get_storage_by_name(self._job.storage_name).pos,
                job_id = self._job.id)
            self.assignments.append(assignement)
            ettilog.loginfo("JobManager:: Publishing assignment for %s (%s)", assignement.agent_name, str(assignement.assigned))
            self._pub_job_assignment.publish(assignement)

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
            ettilog.logerr("JobManager:: Job (%s) starting with %s to deliver %s",
                           self._job.id,
                           str([bid.agent_name for bid in self.selected_bids]),
                           self._job.storage_name)
        else:
            ettilog.loginfo("JobManager:: coordination unsuccessful. cancelling... Received %d/%d from %s", len(self.acknowledgements), len(self.assignments), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

            for bid in self.selected_bids:
                assignement = JobAssignment(
                    id=bid.id,
                    agent_name=bid.agent_name,
                    assigned=False,
                    deadline=0,
                    type=TaskKnowledgebase.TYPE_DELIVER,
                    pos=self._facility_provider.get_storage_by_name(self._job.storage_name).pos,
                    job_id=self._job.id)

                self.assignments.append(assignement)
                ettilog.loginfo("JobManager:: Publishing assignment for %s (%s)", assignement.agent_name,
                                str(assignement.assigned))
                self._pub_job_assignment.publish(assignement)

        self.id = self.job_id(new_id=True)
        self.busy = False
        ettilog.loginfo("JobManager:: ---------------------------Manager stop---------------------------")