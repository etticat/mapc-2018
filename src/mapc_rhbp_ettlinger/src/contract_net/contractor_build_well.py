import random

from mapc_rhbp_ettlinger.msg import TaskBid

from contract_net.contractor import ContractNetContractorBehaviour
from decisions.generate_bid_from_request import GenerateBidFromRequestDecision
from decisions.current_task import CurrentTaskDecision


class BuildWellContractor(ContractNetContractorBehaviour):
    """
    Contractor class for accepting build well coordination requests
    """

    def __init__(self, agent_name, current_task_mechanism, ready_for_bid_condition):
        super(BuildWellContractor, self).__init__(
            agent_name, task_type=CurrentTaskDecision.TYPE_BUILD_WELL, current_task_mechanism=current_task_mechanism)
        self.job_bid_decider = GenerateBidFromRequestDecision(self._agent_name)
        self.ready_for_bid_condition = ready_for_bid_condition

    def bid_possible(self, bid):
        """
        Method decides if bid is still possible after assignment.
        Just check if the agent is still not busy.
        :param bid:
        :return:
        """
        return self.should_bid_for_request(bid.request)

    def should_bid_for_request(self, request):
        """
        Check if the preconditions are met to send a request.
        :param request:
        :return:
        """
        self.ready_for_bid_condition.sync()
        self.ready_for_bid_condition.updateComputation()
        should_bid = self.ready_for_bid_condition.satisfaction > 0.8
        return should_bid

    def generate_bid(self, request):
        """
        Generates a bid for well assembly
        :param request:
        :return:
        """
        return TaskBid(
            id=request.id,
            agent_name=self._agent_name,
            expected_steps=random.randint(3, 10),  # TODO
            request=request
        )

    def _on_assignment_confirmed(self, assignment):
        # TODO: Save, that massim is reserved
        pass
