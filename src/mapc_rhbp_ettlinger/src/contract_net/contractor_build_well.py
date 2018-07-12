import random

from mapc_rhbp_ettlinger.msg import TaskBid

from contract_net.contractor import ContractNetContractorBehaviour
from decisions.job_bid import JobBidDecider
from decisions.p_task_decision import CurrentTaskDecision


class BuildWellContractor(ContractNetContractorBehaviour):

    def _on_assignment_confirmed(self, assignment):
        # TODO: Save in DB, that items are reserved
        pass

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, mechanism, ready_for_bid_condition):
        super(BuildWellContractor, self).__init__(
            agent_name, task_type=CurrentTaskDecision.TYPE_BUILD_WELL, mechanism=mechanism)
        self.job_bid_decider = JobBidDecider(self._agent_name)
        self.ready_for_bid_condition = ready_for_bid_condition

    def should_bid_for_request(self, request):
        self.ready_for_bid_condition.sync()
        self.ready_for_bid_condition.updateComputation()
        should_bid = self.ready_for_bid_condition.satisfaction > 0.8
        return should_bid

    def generate_bid(self, request):
        return TaskBid(
            id=request.id,
            agent_name=self._agent_name,
            expected_steps=random.randint(3, 10),  # TODO
            request=request
        )
