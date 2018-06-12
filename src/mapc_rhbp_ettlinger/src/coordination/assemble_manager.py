import random
import time

import rospy
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAssignment, AssembleAcknowledgement, \
    AssembleTask

import utils.rhbp_logging
from agent_knowledge.assemble_task import AssembleKnowledgebase
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_manager')


class AssembleManager(object):

    DEADLINE_BIDS = 2
    DEADLINE_ACKNOLEDGEMENT = 2

    def __init__(self, agent_name, role):

        self._agent_name = agent_name
        self._role = role
        self.workshop_position = None
        self.busy = False

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        self._assemble_knowledgebase = AssembleKnowledgebase()

        self.id = 0

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
        self.id = time.time() %512
        self.bids = []

        self.busy = True

        ettilog.loginfo("AssembleManager(%s):: requesting assist", self._agent_name)

        request = AssembleRequest(
            deadline=time.time() + AssembleManager.DEADLINE_BIDS,
            destination = workshop,
            agent_name = self._agent_name,
            id = self.generate_assemble_id(new_id=True)
        )
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
        generated_id = self.generate_assemble_id()
        if bid.id != generated_id:
            ettilog.loginfo("wrong id")
            return
        self.bids.append(bid)



    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        ettilog.loginfo("AssembleManager(%s): Processing %s bids", self._agent_name, str(len(self.bids)))

        self.accepted_bids, finished_products = self._product_provider.choose_best_bid_combination(self.bids)

        if len(finished_products.keys()) == 0:
            ettilog.logerr("AssembleManager(%s): No useful bid combination found in %d bids", self._agent_name, len(self.bids))
            self.busy = False
            self.accepted_bids = []

        rejected_bids = []

        for bid in self.bids:
            if bid not in self.accepted_bids:
                rejected_bids.append(bid)


        ettilog.loginfo("AssembleManager(%s): Assembling %s with %s", self._agent_name, str(finished_products), str([bid.agent_name for bid in self.accepted_bids]))

        ettilog.loginfo("AssembleManager(%s): Accepting %s bid(s)", self._agent_name, str(len(self.accepted_bids)))
        ettilog.loginfo("AssembleManager(%s): Rejecting %s bid(s)", self._agent_name, str(len(rejected_bids)))

        assembly_instructions = self.generate_assembly_instructions(self.accepted_bids, finished_products)

        deadline = time.time() + AssembleManager.DEADLINE_ACKNOLEDGEMENT

        for bid in self.bids:
            if bid in self.accepted_bids:
                assignment = AssembleAssignment(
                    assigned = True,
                    deadline=deadline,
                    bid = bid,
                    tasks = assembly_instructions[bid.agent_name]
                )
            else:
                assignment = AssembleAssignment(
                    assigned = False,
                    deadline=deadline,
                    bid = bid,
                    tasks = ""
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
            ettilog.loginfo("Contractor rejected acknowledgement")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        ettilog.loginfo("AssembleManager(%s): Processing Acknoledgements. Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.bid.agent_name for acknowledgement in self.acknowledgements]))

        if len(self.acknowledgements) == len(self.accepted_bids):
            ettilog.logerr("AssembleManager(%s): coordination successful. work can start with %d agents", self._agent_name, len(self.accepted_bids))
        else:
            ettilog.logerr("AssembleManager(%s): coordination unsuccessful. cancelling...", self._agent_name)

            self._assemble_knowledgebase.cancel_assemble_requests(self.generate_assemble_id())
            for bid in self.bids:
                    assignment = AssembleAssignment(
                        assigned = False,
                        deadline=0,
                        bid = bid,
                    )
                    self._pub_assemble_assignment.publish(assignment)

        self.busy = False
        # rospy.signal_shutdown("end of test")

    def generate_assembly_instructions(self, accepted_bids, finished_products):
        res = {}
        agents = []

        for bid in accepted_bids:
            res[bid.agent_name] = ""
            agents.append(bid.agent_name)

        # Distributing tasks: TODO: currently manager builds everything. in future others may build
        for item, count in finished_products.iteritems():
            for i in range(count):
                selected_agent = random.choice(agents)
                # TODO: Every agent could be selected at different points
                # For now let this be done randomly. In future this has to be selected better
                for agent in agents:
                    if res[agent] is not "":
                        res[agent] = res[agent]  + ","

                    if agent == selected_agent:
                        res[agent] = res[agent] + "assemble:" + item
                    else:
                        res[agent] = res[agent] + "assist:" + selected_agent

        return res
