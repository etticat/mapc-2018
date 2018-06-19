import time

from mac_ros_bridge.msg import Job
from mapc_rhbp_ettlinger.msg import JobRequest, JobAssignment

import utils.rhbp_logging
from agent_knowledge.well import WellTaskKnowledgebase
from coordination.job_manager import JobManager
from decisions.well_chooser import ChooseWellToBuild

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_manager')


class BuildWellManager(JobManager):

    def __init__(self):
        super(BuildWellManager, self).__init__()

        self._well_task_knowledgebase = WellTaskKnowledgebase()
        self._well_chooser = ChooseWellToBuild()

    def well_request(self, well_type, pos):
        """
        The first step of the protocol
        :param job:
        :type job: Job
        :return:
        """
        self.busy = True
        self.bids = []
        self.acknowledgements = []

        ettilog.logerr("BuildWellManager:: ---------------------------BuildWell start---------------------------",)

        request = JobRequest(
            id=self.job_id(),
            deadline =time.time() + JobManager.DEADLINE_BIDS,
            items=[],
            pos=pos

        )

        self._pub_job_request.publish(request)
        self.well_type = well_type
        self.pos = pos

        sleep_time = request.deadline - time.time()
        time.sleep(sleep_time)
        self.process_bids()

    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        accepted_bids = self._well_chooser.choose_agent_for_building(self.bids)

        if len(accepted_bids) == 0:
            ettilog.logerr("BuildWellManager:: No useful bid found in %d bids", len(self.bids))
            self.busy = False

            self.id = self.job_id(new_id=True)
            return
        else:
            ettilog.logerr("BuildWellManager:: Bids processed: Accepted bids from %s",
                           ", ".join([bid.agent_name for bid in accepted_bids]))

        deadline = time.time() + JobManager.DEADLINE_ACKNOLEDGEMENT

        self.assignments = []
        for bid in accepted_bids:
            job_assignement = JobAssignment(
                id = self.job_id(),
                job_id = self.well_type,
                agent_name = bid.agent_name,
                assigned = True,
                type = "build_well",
                deadline = deadline,
                items = [],
                pos = self.pos
            )
            self.assignments.append(job_assignement)
            ettilog.logerr("BuildWellManager:: Publishing assignment for %s", bid.agent_name)
            self._pub_job_assignment.publish(job_assignement)

        time.sleep(deadline - time.time())

        self._process_bid_acknoledgements()

    def _process_bid_acknoledgements(self):

        if len(self.acknowledgements) == len(self.assignments):
            ettilog.logerr("BuildWellManager:: coordination successful. work can start with %d agents", len(self.assignments))
        else:
            ettilog.logerr("BuildWellManager:: coordination unsuccessful. cancelling... Received %d/%d from %s", len(self.acknowledgements), len(self.assignments), str([acknowledgement.agent_name for acknowledgement in self.acknowledgements]))

            # TODO: Let the contractor delete the task, so they can cleanup the goals

        self.id = self.job_id(new_id=True)
        self.busy = False
        ettilog.logerr("BuildWellManager:: ---------------------------Manager stop---------------------------")