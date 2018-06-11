import time

import rospy
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAssignment, AssembleAcknowledgement

from common_utils.agent_utils import AgentUtils
from coordination.product_info import ProductValueInfo

import utils.rhbp_logging
from provider.product_provider import ProductProvider

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_manager')

class AssembleManager(object):

    DEADLINE_BIDS = 0.5
    DEADLINE_ACKNOLEDGEMENT = 0.5

    def __init__(self, agent_name):

        self.items = []
        self._agent_name = agent_name

        self._product_value_info = ProductValueInfo()
        self._product_provider = ProductProvider(agent_name=self._agent_name)

        self.id = time.time() %512

        self.bids = []
        self.accepted_bids = []
        self.acknowledgements = []

        self._pub_assemble_request = rospy.Publisher(AgentUtils.get_assemble_prefix() + "request", AssembleRequest,
                                                     queue_size=10)
        rospy.Subscriber(AgentUtils.get_assemble_prefix() + "bid", AssembleBid, self._callback_bid)
        self._pub_assemble_assignment = rospy.Publisher(AgentUtils.get_assemble_prefix() + "assign", AssembleAssignment,
                                                        queue_size=10)
        rospy.Subscriber(AgentUtils.get_assemble_prefix() + "acknowledge", AssembleAcknowledgement,
                         self._callback_acknowledgement)

    def request_assist(self, workshop):
        """
        The first step of the protocol
        :return:
        """
        request = AssembleRequest(
            deadline=time.time() + AssembleManager.DEADLINE_BIDS,
            destination = workshop,
            agent_name = self._agent_name,
            id = self.generate_assemble_id(new_id=True)
        )
        self.bids = []
        self._pub_assemble_request.publish(request)

        sleep_time = request.deadline - time.time()
        rhbplog.loginfo("AssembleManager(%s):: sleeping for %s", self._agent_name, str(sleep_time))
        time.sleep(sleep_time)
        self.process_bids()

    def generate_assemble_id(self, new_id=False):
        if new_id:
            self.id = self.id + 1
        id_ = self._agent_name + "-" + str(self.id)
        return id_

    def _callback_bid(self, bid):
        rhbplog.loginfo("AssembleManager(%s): Received bid from %s", self._agent_name, bid.agent_name)
        if bid.id != self.generate_assemble_id():
            rhbplog.loginfo("wrong id")
            return
        self.bids.append(bid)



    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """
        max_bid = 0
        combination = []

        rhbplog.loginfo("AssembleManager(%s): Processing %s bids", self._agent_name, str(len(self.bids)))

        # for bid in self.bids:

        self.accepted_bids, finished_products = self._product_value_info.choose_best_bid_combination(self.bids, self.items)
        rejected_bids = []

        for bid in self.bids:
            if bid not in self.accepted_bids:
                rejected_bids.append(bid)

        rhbplog.logerr("AssembleManager(%s): Assembling %s", self._agent_name, str(finished_products))

        rhbplog.loginfo("AssembleManager(%s): Accepting %s bid(s)", self._agent_name, str(len(self.accepted_bids)))
        rhbplog.loginfo("AssembleManager(%s): Rejecting %s bid(s)", self._agent_name, str(len(rejected_bids)))

        deadline = time.time() + AssembleManager.DEADLINE_ACKNOLEDGEMENT

        for bid in self.bids:
            assignment = AssembleAssignment(
                assigned = (bid in self.accepted_bids),
                deadline=deadline,
                bid = bid
            )

            rhbplog.loginfo("AssembleManager(%s): Publishing assignment for %s (%s)", self._agent_name, bid.agent_name, str(assignment.assigned))
            self._pub_assemble_assignment.publish(assignment)

        time.sleep(deadline - time.time())

        self._process_bid_acknoledgements()


    def assign_bids(self, bids, combination):
        """
        Assignes assist tasks to assemble all items from the combinationarray
        :param bids:
        :param combination:
        :return:
        """

    def _callback_acknowledgement(self, acknowledgement):
        rhbplog.loginfo("AssembleManager(%s): Received Acknowledgement from %s", self._agent_name, acknowledgement.bid.agent_name)
        if acknowledgement.bid.id != self.generate_assemble_id():
            rhbplog.loginfo("wrong id")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        rhbplog.logerr("AssembleManager(%s): Processing Acknoledgements. Received %d/%d", self._agent_name, len(self.acknowledgements), len(self.accepted_bids))

        if len(self.acknowledgements) == len(self.accepted_bids):
            rhbplog.loginfo("AssembleManager(%s): coordination successful. work can start", self._agent_name)
        else:
            rhbplog.loginfo("AssembleManager(%s): coordination unsuccessful. cancelling...", self._agent_name)
            for bid in self.bids:
                    assignment = AssembleAssignment(
                        assigned = False,
                        deadline=0,
                        bid = bid
                    )
                    self._pub_assemble_assignment.publish(assignment)