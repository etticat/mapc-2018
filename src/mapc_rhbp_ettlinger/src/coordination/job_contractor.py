import random
import time

import rospy
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAcknowledgement, JobAssignment, \
    JobTask

from common_utils.agent_utils import AgentUtils

import utils.rhbp_logging
from provider.product_provider import ProductProvider

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_contractor')

class JobContractor(object):


    def __init__(self, agent_name, role, product_provider=None):

        self._agent_name = agent_name
        self.role = role
        self.current_job = None
        # self._job_knowledgebase = JobKnowledgebase()


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
        job_job = None # TODO: self._job_knowledgebase.get_job_job(self._agent_name)

        current_time = time.time()
        if request.deadline < current_time:
            rospy.logerr("Deadline over")
            return

        if job_job is None:
            self.send_bid(request)

    def send_bid(self, request):

        bid = JobBid(
            id=request.id,
            bid = random.randint(0, 7),
            expected_steps=random.randint(3, 10),
            agent_name = self._agent_name,
            items = self._product_provider.get_items(),
        )

        rhbplog.loginfo("JobContractor(%s):: bidding on %s: %s", self._agent_name, request.id, bid.bid)
        self._pub_job_bid.publish(bid)

        self.current_job = request.id

    def _callback_assign(self, job_assignment):


        if job_assignment.agent_name != self._agent_name or self.current_job != job_assignment.id:
            return
        if job_assignment.assigned == False:
            rhbplog.loginfo("JobContractor(%s):: Cancelled assignment for %s", self._agent_name, job_assignment.id)
            return

        rhbplog.logerr("JobContractor(%s):: Received assignment for %s", self._agent_name, job_assignment.id)

        is_still_possible = True # TODO check if agent is still idle

        if is_still_possible:

            # accepted = self._job_knowledgebase.save_job(JobTask(
            #     id=job_assignment.id,
            #     agent_name=self._agent_name,
            #     pos=job_assignment.bid.request.destination,
            #     jobs=job_assignment.jobs,
            #     active=True
            # ))
            acknoledgement = JobAcknowledgement(
                acknowledged=True, # TODO check
                agent_name=job_assignment.agent_name,
                id=job_assignment.id,
                items=job_assignment.items
            )
            self._pub_job_acknowledge.publish(acknoledgement)