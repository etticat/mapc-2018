import time

import rospy
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAcknowledgement, AssembleAssignment, \
    AssembleStop, Task

import utils.rhbp_logging
from agent_knowledge.task import TaskKnowledgebase
from common_utils.agent_utils import AgentUtils
from decisions.assembly_bid import ShouldBidForAssembly
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_contractor')

class AssembleContractor(object):


    def __init__(self, agent_name, role, product_provider=None):

        self._assembly_bid_chooser = ShouldBidForAssembly(agent_name=agent_name, role=role)
        self._agent_name = agent_name
        self.role = role
        self.current_task = None
        self._task_knowledge_base = TaskKnowledgebase()
        self.enabled = True


        if product_provider == None:
            self._product_provider = ProductProvider(agent_name=self._agent_name)
        else:
            # TODO: This is only for testing
            self._product_provider = product_provider

        prefix = AgentUtils.get_assemble_prefix()

        rospy.Subscriber(prefix + "request", AssembleRequest, self._callback_request)
        self._pub_assemble_bid = rospy.Publisher(prefix + "bid", AssembleBid, queue_size=10)
        rospy.Subscriber(prefix + "assign", AssembleAssignment, self._callback_assign)
        self._pub_assemble_acknowledge = rospy.Publisher(prefix + "acknowledge", AssembleAcknowledgement, queue_size=10)
        rospy.Subscriber(prefix + "stop", AssembleStop, self._callback_stop)


    def _callback_request(self, request):
        """

        :param request:
        :type request: AssembleRequest
        :return:
        """

        if self.enabled == False:
            return

        current_time = time.time()
        if request.deadline < current_time:
            ettilog.loginfo("Deadline over %f - %f", request.deadline, current_time)
            return

        if not self._task_knowledge_base.has_priority_task(agent_name=self._agent_name):
            self.send_bid(request)

    def send_bid(self, request):

        bid = self._assembly_bid_chooser.choose(request)

        ettilog.loginfo("AssembleContractor(%s):: Sending bid: %s", self._agent_name, str(bid is not None))
        if bid is None:
            return

        ettilog.loginfo("AssembleContractor(%s):: bidding on %s: %s", self._agent_name, request.id, bid.bid)
        self._pub_assemble_bid.publish(bid)

        self.current_task = request.id

    def _callback_assign(self, assembleAssignment):


        if assembleAssignment.bid.agent_name != self._agent_name or self.current_task != assembleAssignment.bid.id:
            return
        if assembleAssignment.assigned == False:
            ettilog.loginfo("AssembleContractor(%s):: Cancelled assignment for %s", self._agent_name, assembleAssignment.bid.id)
            return

        ettilog.loginfo("AssembleContractor(%s):: Received assignment for %s", self._agent_name, assembleAssignment.bid.id)

        is_still_possible = True # TODO check if agent is still idle

        if is_still_possible:

            accepted = self._task_knowledge_base.create_task(Task(
                id=assembleAssignment.bid.id,
                type=TaskKnowledgebase.TYPE_ASSEMBLE,
                agent_name=self._agent_name,
                pos=assembleAssignment.bid.request.destination,
                task=assembleAssignment.tasks
            ))
            items_to_assemble = {}
            for task in assembleAssignment.tasks.split(","):
                task_split = task.split(":")
                if task_split[0] == "assemble":
                    items_to_assemble[task_split[1]] = items_to_assemble.get(task_split[1], 0) + 1

            self._product_provider.start_assembly(items_to_assemble=items_to_assemble)
            acknoledgement = AssembleAcknowledgement(
                acknowledged=accepted,
                bid=assembleAssignment.bid
            )
            self._pub_assemble_acknowledge.publish(acknoledgement)

    def _callback_stop(self, assemble_stop):
        """

        :param assemble_stop:
        :type assemble_stop: AssembleStop
        :return:
        """
        current_assemble_task = self._task_knowledge_base.get_task(self._agent_name, type=TaskKnowledgebase.TYPE_ASSEMBLE)
        if current_assemble_task is not None and current_assemble_task.id == assemble_stop.id:
            self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=TaskKnowledgebase.TYPE_ASSEMBLE)
            self._product_provider.stop_assembly()

            ettilog.logerr("AssembleContractor(%s):: Stopping task %s because %s", self._agent_name, assemble_stop.id, assemble_stop.reason)
