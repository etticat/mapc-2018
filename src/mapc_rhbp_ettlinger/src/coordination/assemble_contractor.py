import random
import time

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAcknowledgement, AssembleAssignment, \
    AssembleTask

from agent_knowledge.assemble_task import AssembleKnowledgebase
from common_utils.agent_utils import AgentUtils

import utils.rhbp_logging
from provider.product_provider import ProductProvider

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_contractor')

class AssembleContractor(object):


    def __init__(self, agent_name, role, product_provider=None):

        self._agent_name = agent_name
        self.role = role
        self.current_task = None
        self._assemble_knowledgebase = AssembleKnowledgebase()


        if product_provider == None:
            self._product_provider = ProductProvider(agent_name=self._agent_name)
        else:
            # TODO: This is only for testing
            self._product_provider = product_provider

        self.busy = self._assemble_knowledgebase.get_assemble_task(self._agent_name) != None

        prefix = AgentUtils.get_assemble_prefix()

        rospy.Subscriber(prefix + "request", AssembleRequest, self._callback_request)
        self._pub_assemble_bid = rospy.Publisher(prefix + "bid", AssembleBid, queue_size=10)
        rospy.Subscriber(prefix + "assign", AssembleAssignment, self._callback_assign)
        self._pub_assemble_acknowledge = rospy.Publisher(prefix + "acknowledge", AssembleAcknowledgement, queue_size=10)


    def _callback_request(self, request):
        """

        :param request:
        :type request: AssembleRequest
        :return:
        """
        assemble_task = self._assemble_knowledgebase.get_assemble_task(self._agent_name)

        current_time = time.time()
        if request.deadline < current_time:
            rospy.logerr("Deadline over")
            return

        if self.busy is False and assemble_task is None:
            self.send_bid(request)

    def send_bid(self, request):

        self.busy = True
        bid = AssembleBid(
            id=request.id,
            bid = random.randint(0,7),
            agent_name = self._agent_name,
            items = self._product_provider.get_items(), # TODO: Read from db
            role = self.role,
            request = request
        )

        rhbplog.logerr("AssembleContractor(%s):: bidding on %s: %s", self._agent_name, request.id, bid.bid)
        self._pub_assemble_bid.publish(bid)

        self.current_task = request.id

        time.sleep(4)
        self.busy = False



    def _callback_assign(self, assembleAssignment):


        if assembleAssignment.bid.agent_name != self._agent_name or self.current_task != assembleAssignment.bid.id:
            return
        if assembleAssignment.assigned == False:
            rhbplog.logerr("AssembleContractor(%s):: Cancelled assignment for %s", self._agent_name, assembleAssignment.bid.id)
            self.busy = False
            return
        rhbplog.logerr("AssembleContractor(%s):: Received assignment for %s", self._agent_name, assembleAssignment.bid.id)

        is_still_possible = True # TODO check if agent is still idle

        if is_still_possible:

            accepted = self._assemble_knowledgebase.save_assemble(AssembleTask(
                id=assembleAssignment.bid.id,
                agent_name=self._agent_name,
                pos=assembleAssignment.bid.request.destination,
                tasks=assembleAssignment.tasks,
                active=True
            ))
            acknoledgement = AssembleAcknowledgement(
                acknowledged=accepted,
                bid=assembleAssignment.bid
            )
            self._pub_assemble_acknowledge.publish(acknoledgement)

        self.busy = False
