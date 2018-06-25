import random

from mapc_rhbp_ettlinger.msg import TaskBid

from agent_knowledge.task import TaskKnowledgeBase
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.job_bid import JobBidDecider


class BuildWellContractorBehaviour(ContractNetContractorBehaviour):

    def _on_task_finished(self, finish):
        successful = self._task_knowledge_base.finish_task(
            agent_name=self._agent_name,
            task=finish.job_id,
            type=TaskKnowledgeBase.TYPE_BUILD_WELL
        )

    def _on_assignment_confirmed(self, assignment):
        # TODO: Save in DB, that items are reserved
        pass

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, name, **kwargs):
        super(BuildWellContractorBehaviour, self).__init__(
            agent_name, name, task_type=TaskKnowledgeBase.TYPE_BUILD_WELL, **kwargs)
        self.job_bid_decider = JobBidDecider(self._agent_name)

    def generate_bid(self, request):
        return TaskBid(
            id=request.id,
            agent_name=self._agent_name,
            expected_steps=random.randint(3, 10),  # TODO
            request=request
        )
