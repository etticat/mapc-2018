from agent_knowledge.task import TaskKnowledgeBase
from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.job_bid import JobBidDecider


ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.deliver')

class DeliverContractorBehaviour(ContractNetContractorBehaviour):

    def _on_task_finished(self, finish):
        self._task_knowledge_base.finish_task(
            agent_name=self._agent_name,
            task=finish.job_id,
            type=TaskKnowledgeBase.TYPE_DELIVER
        )

    def _on_assignment_confirmed(self, assignment):
        # TODO: Save in DB, that items are reserved
        pass

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, name, **kwargs):
        super(DeliverContractorBehaviour, self).__init__(
            agent_name, name, task_type=TaskKnowledgeBase.TYPE_DELIVER, **kwargs)
        self.job_bid_decider = JobBidDecider(self._agent_name)

    def generate_bid(self, request):
        return self.job_bid_decider.generate_bid(request)
