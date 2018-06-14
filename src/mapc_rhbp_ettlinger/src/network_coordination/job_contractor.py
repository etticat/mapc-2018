import random
import time

import rospy
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAcknowledgement, JobAssignment, \
    JobTask

from agent_knowledge.tasks import JobKnowledgebase
from common_utils.agent_utils import AgentUtils

import utils.rhbp_logging
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.job_contractor')

class JobContractor(object):


    def __init__(self, agent_name, role, product_provider=None):

        self._agent_name = agent_name
        self.role = role
        self._job_knowledgebase = JobKnowledgebase()


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
        if request.deadline < current_time:
            rospy.logerr("Deadline over")
            return

        job_job = self._job_knowledgebase.get_tasks(agent_name=self._agent_name)

        if job_job is None:
            self.send_bid(request)

    def send_bid(self, request):

        item_intersect = CalcUtil.items_intersect(request.items, self._product_provider.get_items())

        if len(item_intersect) > 0:
            bid = JobBid(
                id=request.id,
                bid = random.randint(0, 7), # TODO
                expected_steps=random.randint(3, 10), # TODO
                agent_name = self._agent_name,
                items = item_intersect,
            )

            rhbplog.loginfo("JobContractor(%s):: bidding on %s: %s", self._agent_name, request.id, bid.bid)
            self._pub_job_bid.publish(bid)


    def _callback_assign(self, job_assignment):


        if job_assignment.agent_name != self._agent_name:
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