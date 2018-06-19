import time

import rospy
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAcknowledgement, JobAssignment, \
    JobTask, WellTask

import utils.rhbp_logging
from agent_knowledge.tasks import JobKnowledgebase
from agent_knowledge.well import WellTaskKnowledgebase
from common_utils.agent_utils import AgentUtils
from decisions.job_bid import JobBidDecider
from provider.product_provider import ProductProvider

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_contractor')

class JobContractor(object):


    def __init__(self, agent_name, product_provider=None):

        self._agent_name = agent_name
        self._job_knowledgebase = JobKnowledgebase()
        self._well_task_knowledgebase = WellTaskKnowledgebase()
        self.job_bid_decider = JobBidDecider(self._agent_name)


        if product_provider == None:
            self._product_provider = ProductProvider(agent_name=self._agent_name)
        else:
            # TODO: This is only for testing
            self._product_provider = product_provider

        prefix = AgentUtils.get_job_prefix()

        rospy.Subscriber(prefix + "request", JobRequest, self._callback_request)
        self._pub_job_bid = rospy.Publisher(prefix + "bid", JobBid, queue_size=10)
        rospy.Subscriber(prefix + "assign", JobAssignment, self._callback_assign)
        self._pub_job_acknowledge = rospy.Publisher(prefix + "acknowledge", JobAcknowledgement, queue_size=10)


    def _callback_request(self, request):
        """

        :param request:
        :type request: JobRequest
        :return:
        """

        current_time = time.time()
        # if request.deadline < current_time:
        #     rospy.loginfo("Deadline over")
        #     return
        #
        current_job = self._job_knowledgebase.get_task(agent_name=self._agent_name)
        current_well_job = self._well_task_knowledgebase.get_task(agent_name=self._agent_name)

        if current_job is None and current_well_job is None:
            self.send_bid(request)

    def send_bid(self, request):
        # Items which are requested and can be provided by agent

        bid = self.job_bid_decider.generate_bid(request)

        if bid != None:
            rhbplog.loginfo("JobContractor(%s):: bidding on %s", self._agent_name, request.id)
            self._pub_job_bid.publish(bid)


    def _callback_assign(self, job_assignment):


        if job_assignment.agent_name != self._agent_name:
            return
        if job_assignment.assigned == False:
            rhbplog.loginfo("JobContractor(%s):: Cancelled assignment for %s", self._agent_name, job_assignment.id)
            return

        rhbplog.loginfo("JobContractor(%s):: Received assignment for %s", self._agent_name, job_assignment.id)

        is_still_possible = True # TODO check if agent is still idle

        if is_still_possible:

            acknoledgement = JobAcknowledgement(
                acknowledged=True,
                agent_name=job_assignment.agent_name,
                id=job_assignment.id,
                items=job_assignment.items
            )

            if job_assignment.type == "deliver":

                self._job_knowledgebase.save_task(JobTask(
                    job_id = job_assignment.job_id,
                    agent_name = self._agent_name,
                    pos = job_assignment.pos,
                    items = job_assignment.items
                ))
            elif job_assignment.type == "build_well":
                self._well_task_knowledgebase.save_task(WellTask(
                    agent_name = self._agent_name,
                    pos = job_assignment.pos,
                    well_type = job_assignment.job_id,
                    built = False
                ))
            self._pub_job_acknowledge.publish(acknoledgement)