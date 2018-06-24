import rospy
from mapc_rhbp_ettlinger.msg import JobRequest, JobBid, JobAcknowledgement, JobAssignment, \
    Task

from agent_knowledge.task import TaskKnowledgebase
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from decisions.job_bid import JobBidDecider
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.coordination.job_contractor')


class JobContractor(object):

    def __init__(self, agent_name, product_provider=None):

        self._agent_name = agent_name
        self._task_knowledgebase = TaskKnowledgebase()
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

        if not self._task_knowledgebase.has_priority_task(agent_name=self._agent_name):
            self.send_bid(request)

    def send_bid(self, request):
        # Items which are requested and can be provided by agent

        bid = self.job_bid_decider.generate_bid(request)

        if bid != None:
            ettilog.loginfo("JobContractor(%s):: bidding on %s", self._agent_name, request.id)
            self._pub_job_bid.publish(bid)

    def _callback_assign(self, job_assignment):

        if job_assignment.agent_name != self._agent_name:
            return

        if job_assignment.assigned == True:
            ettilog.loginfo("JobContractor(%s):: Received assignment for %s", self._agent_name, job_assignment.id)
            self.save_job(job_assignment)
        if job_assignment.assigned == False:
            ettilog.loginfo("JobContractor(%s):: Cancelled assignment for %s", self._agent_name, job_assignment.id)
            self.cancel_job(job_assignment)

    def save_job(self, job_assignment):
        is_still_possible = True  # TODO check if agent is still idle
        if is_still_possible:

            acknoledgement = JobAcknowledgement(
                acknowledged=True,
                agent_name=job_assignment.agent_name,
                id=job_assignment.id,
                items=job_assignment.items
            )

            successful = self._task_knowledgebase.create_task(Task(
                agent_name=self._agent_name,
                pos=job_assignment.pos,
                task=job_assignment.job_id,
                type=job_assignment.type
            ))

            if successful:
                self._pub_job_acknowledge.publish(acknoledgement)

    def cancel_job(self, job_assignment):

        successful = self._task_knowledgebase.finish_task(
            agent_name=self._agent_name,
            task=job_assignment.job_id,
            type=job_assignment.type
        )
