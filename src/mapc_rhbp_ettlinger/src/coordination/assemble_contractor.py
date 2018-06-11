import random
from time import time

import rospy
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAcknowledgement, AssembleAssignment

from common_utils.agent_utils import AgentUtils

import utils.rhbp_logging

rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.assemble_contractor')

class AssembleContractor:


    def __init__(self, agent_name):

        self.agent_name = agent_name
        self.current_task = None
        self.items = []

        prefix = AgentUtils.get_assemble_prefix()

        rospy.Subscriber(prefix + "request", AssembleRequest, self._callback_request)
        self._pub_assemble_bid = rospy.Publisher(prefix + "bid", AssembleBid, queue_size=10)
        rospy.Subscriber(prefix + "assign", AssembleAssignment, self._callback_assign)
        self._pub_assemble_acknowledge = rospy.Publisher(prefix + "acknowledge", AssembleAcknowledgement, queue_size=10)


    def _callback_request(self, request):
        if self.current_task == None:
            self.send_bid(request)




    def send_bid(self, request):

        bid = AssembleBid(
            id=request.id,
            bid = random.randint(0,7),
            agent_name = self.agent_name,
            items = self.items
        )

        rhbplog.loginfo("AssembleContractor(%s):: bidding on %s: %s",self.agent_name, request.id, bid.bid)
        self._pub_assemble_bid.publish(bid)

        self.current_task = request.id


        # This can't really be calculated into a number. The recipient has to combine multiple agents
        # bid = {
        #     "ingredients": {
        #         "item1": 2
        #         "item4": 12
        #     },
        #     "load_free" : 10,  # The less load the agent has available, the better it is for them to go assembling.
        #     "role": "car",
        #     "duration_to_destination": 4 # steps
        #
        # }
    # def generate_bid_value(self):


    # def process_bid_assignage(self):
    #     still_available_for_task_assembly = True
    #     if  still_available_for_task_assembly:
    #         self.send_acknowledgement()

    # def send_acknowledgement(self):
    #     pass

    def _callback_assign(self, request):


        if request.bid.agent_name != self.agent_name or self.current_task != request.bid.id:
            return
        rhbplog.loginfo("AssembleContractor(%s):: Received assignage for %s",self.agent_name, request.bid.id)

        if request.assigned == False:
            self.idle = True
            return

        is_still_possible = True # TODO check if agent is still idle

        if is_still_possible:
            acknoledgement = AssembleAcknowledgement(
                acknowledged=True,
                bid=request.bid
            )
            self._pub_assemble_acknowledge.publish(acknoledgement)
