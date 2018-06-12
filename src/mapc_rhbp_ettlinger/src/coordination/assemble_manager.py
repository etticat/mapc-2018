import time

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAssignment, AssembleAcknowledgement, \
    AssembleTask

from agent_knowledge.assemble_task import AssembleKnowledgebase
from common_utils.agent_utils import AgentUtils
from coordination.product_info import ProductValueInfo

import utils.rhbp_logging
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_manager')

class AssembleManager(object):

    DEADLINE_BIDS = 0.5
    DEADLINE_ACKNOLEDGEMENT = 0.5

    def __init__(self, agent_name, role, product_provider = None):

        self._agent_name = agent_name
        self._role = role
        self.busy = False
        self.workshop_position = None

        self._product_value_info = ProductValueInfo()
        if product_provider == None:
            self._product_provider = ProductProvider(agent_name=self._agent_name)
        else:
            # TODO: This is only for testing
            self._product_provider = product_provider
        self._assemble_knowledgebase = AssembleKnowledgebase()

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
        ettilog.logerr("AssembleManager(%s):: requesting assist", self._agent_name)
        request = AssembleRequest(
            deadline=time.time() + AssembleManager.DEADLINE_BIDS,
            destination = workshop,
            agent_name = self._agent_name,
            id = self.generate_assemble_id(new_id=True)
        )
        self.bids = []
        self.busy = True
        self.workshop_position = workshop
        self._pub_assemble_request.publish(request)

        sleep_time = request.deadline - time.time()
        ettilog.loginfo("AssembleManager(%s):: sleeping for %s", self._agent_name, str(sleep_time))
        time.sleep(sleep_time)
        self.process_bids()

    def generate_assemble_id(self, new_id=False):
        if new_id:
            self.id = self.id + 1
        id_ = self._agent_name + "-" + str(self.id)
        return id_

    def _callback_bid(self, bid):
        ettilog.loginfo("AssembleManager(%s): Received bid from %s", self._agent_name, bid.agent_name)
        if bid.id != self.generate_assemble_id():
            ettilog.loginfo("wrong id")
            return
        self.bids.append(bid)



    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        ettilog.logerr("AssembleManager(%s): Processing %s bids", self._agent_name, str(len(self.bids)))

        self.accepted_bids, finished_products = self._product_value_info.choose_best_bid_combination(self.bids, self._product_provider.get_items(), self._role)

        if len(finished_products.keys()) == 0:
            ettilog.logerr("No useful bid combination found")
            self.busy = False
            return

        rejected_bids = []

        for bid in self.bids:
            if bid not in self.accepted_bids:
                rejected_bids.append(bid)


        ettilog.logerr("AssembleManager(%s): Assembling %s with %s", self._agent_name, str(finished_products), str([bid.agent_name for bid in self.accepted_bids]))

        ettilog.loginfo("AssembleManager(%s): Accepting %s bid(s)", self._agent_name, str(len(self.accepted_bids)))
        ettilog.loginfo("AssembleManager(%s): Rejecting %s bid(s)", self._agent_name, str(len(rejected_bids)))

        products_to_assemble = []
        products_to_assemble_others = []
        # Distributing tasks: TODO: currently manager builds everything. in future others may build
        for item, count in finished_products.iteritems():
            for i in range(count):
                products_to_assemble.append("assemble:" + item)
                products_to_assemble_others.append("assist:" + self._agent_name)


        accepted = self._assemble_knowledgebase.save_assemble(AssembleTask(
            id=self.generate_assemble_id(),
            agent_name=self._agent_name,
            pos=self.workshop_position,
            tasks=",".join(products_to_assemble),
            active=True
        ))

        assert accepted == True # The manager should not have gotten a task in the meantime

        deadline = time.time() + AssembleManager.DEADLINE_ACKNOLEDGEMENT

        for bid in self.bids:
            assignment = AssembleAssignment(
                assigned = (bid in self.accepted_bids),
                deadline=deadline,
                bid = bid,
                tasks = ",".join(products_to_assemble_others)
            )

            ettilog.loginfo("AssembleManager(%s): Publishing assignment for %s (%s)", self._agent_name, bid.agent_name, str(assignment.assigned))
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
        ettilog.loginfo("AssembleManager(%s): Received Acknowledgement from %s", self._agent_name, acknowledgement.bid.agent_name)
        if acknowledgement.bid.id != self.generate_assemble_id():
            ettilog.loginfo("wrong id")
            return
        if acknowledgement.acknowledged == False:
            ettilog.logerr("Contractor rejected acknowledgement")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        ettilog.logerr("AssembleManager(%s): Processing Acknoledgements. Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.bid.agent_name for acknowledgement in self.acknowledgements]))

        if len(self.acknowledgements) == len(self.accepted_bids):
            ettilog.loginfo("AssembleManager(%s): coordination successful. work can start", self._agent_name)
        else:
            ettilog.loginfo("AssembleManager(%s): coordination unsuccessful. cancelling...", self._agent_name)

            self._assemble_knowledgebase.cancel_assemble_requests(self.generate_assemble_id())
            for bid in self.bids:
                    assignment = AssembleAssignment(
                        assigned = False,
                        deadline=0,
                        bid = bid,
                    )
                    self._pub_assemble_assignment.publish(assignment)

        # rospy.signal_shutdown("end of test")
        self.busy = False