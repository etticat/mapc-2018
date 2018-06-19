import random
import time

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAssignment, AssembleAcknowledgement, \
    AssembleTask, AssembleManagerStatus, AssembleStop

import utils.rhbp_logging
from agent_knowledge.assemble_task import AssembleKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.product_util import ProductUtil
from common_utils.rhbp_logging import LOGGER_DEFAULT_NAME
from decisions.assembly_combination import ChooseBestAssemblyCombination
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = utils.rhbp_logging.LogManager(logger_name=LOGGER_DEFAULT_NAME + '.assemble_manager')


class AssembleManager(object):

    DEADLINE_BIDS = 2.0
    DEADLINE_ACKNOLEDGEMENT = 2.0

    def __init__(self, agent_name):

        self._facility_provider = FacilityProvider()
        self._assembly_combination = ChooseBestAssemblyCombination()
        self._agent_name = agent_name

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        self._assemble_knowledgebase = AssembleKnowledgebase()

        self.id = self.assemble_id(new_id=True)
        self.current_running_id = None

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
        self._pub_assemble_stop = rospy.Publisher(AgentUtils.get_assemble_prefix() + "stop", AssembleStop,
                                                        queue_size=10)



        rospy.Subscriber(AgentUtils.get_assemble_prefix() + "request_start", AssembleManagerStatus, self._callback_request_start)
        self._pub_assemble_request_start = rospy.Publisher(AgentUtils.get_assemble_prefix() + "request_start", AssembleManagerStatus,
                                                        queue_size=10)
        rospy.Subscriber(AgentUtils.get_assemble_prefix() + "request_over", AssembleManagerStatus, self._callback_request_over)
        self._pub_assemble_request_over = rospy.Publisher(AgentUtils.get_assemble_prefix() + "request_over", AssembleManagerStatus,
                                                        queue_size=10)

    def _callback_request_start(self, request):
        if self.current_running_id == None:
            self.current_running_id = request.id

            if self.current_running_id == self.id:
                self.bids = []
                self.accepted_bids = []
                self.acknowledgements = []

                ettilog.logerr("AssembleManager(%s):: ---------------------------Manager start---------------------------", self._agent_name)

                request = AssembleRequest(
                    deadline=(time.time() + AssembleManager.DEADLINE_BIDS),
                    destination=self._facility_provider.get_random_storage(),
                    agent_name=self._agent_name,
                    id=self.assemble_id()
                )
                self._pub_assemble_request.publish(request)

                sleep_time = request.deadline - time.time()
                ettilog.logdebug("AssembleManager(%s):: sleeping for %s", self._agent_name, str(sleep_time))
                time.sleep(sleep_time)
                self.process_bids()

    def _callback_request_over(self, request):
        self.current_running_id = None

    def request_assist(self):
        """
        The first step of the protocol
        :return:
        """
        if self._agent_name != "agentA1":
            return # For debugging only allow A1 to manage

        if self.current_running_id == None:
            self._pub_assemble_request_start.publish(AssembleManagerStatus(id=self.assemble_id()))


    def assemble_id(self, new_id=False):
        if new_id:
            self.id = self._agent_name + "-" + str(time.time() %512)
        return self.id

    def _callback_bid(self, bid):
        ettilog.logdebug("AssembleManager(%s): Received bid from %s", self._agent_name, bid.agent_name)
        generated_id = self.assemble_id()
        if bid.id != generated_id:
            ettilog.logdebug("wrong id")
            return
        self.bids.append(bid)



    def process_bids(self):
        """

        :param bids:
        :type bids: dict[]
        :return:
        """

        ettilog.logdebug("AssembleManager(%s, %s): Processing %s bids", self._agent_name, self.assemble_id(), str(len(self.bids)))

        self.accepted_bids, finished_products = self._assembly_combination.choose(self.bids)

        if len(finished_products.keys()) == 0:
            ettilog.logerr("AssembleManager(%s): No useful bid combination found in %d bids", self._agent_name, len(self.bids))
            bids, roles = ProductUtil.get_items_and_roles_from_bids(self.bids)
            ettilog.logerr("AssembleManager(%s): ------ Items: %s", self._agent_name, str(bids))
            ettilog.logerr("AssembleManager(%s): ------ Agents: %s", self._agent_name, str([bid.agent_name for bid in self.bids]))
            ettilog.logerr("AssembleManager(%s): ------ roles: %s", self._agent_name, str(roles))
            self.accepted_bids = []
        else:
            ettilog.logerr("AssembleManager(%s): Bids processed: %s building %s", self._agent_name, ", ".join([bid.agent_name for bid in self.accepted_bids]), str(finished_products.keys()))
            bids, roles = ProductUtil.get_items_and_roles_from_bids(self.bids)
            ettilog.logerr("AssembleManager(%s): ------ Items: %s", self._agent_name, str(bids))
            ettilog.logerr("AssembleManager(%s): ------ Agents: %s", self._agent_name, str([bid.agent_name for bid in self.bids]))
            ettilog.logerr("AssembleManager(%s): ------ roles: %s", self._agent_name, str(roles))
        rejected_bids = []

        for bid in self.bids:
            if bid not in self.accepted_bids:
                rejected_bids.append(bid)


        ettilog.logdebug("AssembleManager(%s): Assembling %s with %s", self._agent_name, str(finished_products), str([bid.agent_name for bid in self.accepted_bids]))

        ettilog.logdebug("AssembleManager(%s): Accepting %s bid(s)", self._agent_name, str(len(self.accepted_bids)))
        ettilog.logdebug("AssembleManager(%s): Rejecting %s bid(s)", self._agent_name, str(len(rejected_bids)))

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

            ettilog.logdebug("AssembleManager(%s): Publishing assignment for %s (%s)", self._agent_name, bid.agent_name, str(assignment.assigned))
            self._pub_assemble_assignment.publish(assignment)

        time.sleep(deadline - time.time())

        self._process_bid_acknoledgements()

    def _callback_acknowledgement(self, acknowledgement):
        ettilog.logdebug("AssembleManager(%s): Received Acknowledgement from %s", self._agent_name, acknowledgement.bid.agent_name)
        if acknowledgement.bid.id != self.assemble_id():
            ettilog.logdebug("wrong id")
            return
        if acknowledgement.acknowledged == False:
            ettilog.logdebug("Contractor rejected acknowledgement")
            return
        self.acknowledgements.append(acknowledgement)

    def _process_bid_acknoledgements(self):
        ettilog.logdebug("AssembleManager(%s): Processing Acknoledgements. Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.bid.agent_name for acknowledgement in self.acknowledgements]))

        if len(self.acknowledgements) == len(self.accepted_bids):
            ettilog.logerr("AssembleManager(%s): coordination successful. work can start with %d agents", self._agent_name, len(self.accepted_bids))
        else:
            ettilog.logerr("AssembleManager(%s): coordination unsuccessful. cancelling... Received %d/%d from %s", self._agent_name, len(self.acknowledgements), len(self.accepted_bids), str([acknowledgement.bid.agent_name for acknowledgement in self.acknowledgements]))

            self._pub_assemble_stop.publish(AssembleStop(id=self.assemble_id(), reason="coordinatino unsuscessful"))

        self._pub_assemble_request_over.publish(AssembleManagerStatus(id=self.id))
        self.id = self.assemble_id(new_id=True)

        ettilog.logerr("AssembleManager(%s):: ---------------------------Manager stop---------------------------",
                   self._agent_name)

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
