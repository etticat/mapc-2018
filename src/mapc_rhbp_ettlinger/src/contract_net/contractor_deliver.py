from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.job_bid import JobBidDecider
from decisions.p_task_decision import CurrentTaskDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.deliver')

class DeliverContractor(ContractNetContractorBehaviour):

    def _on_assignment_confirmed(self, assignment):
        # TODO: Save in DB, that items are reserved
        pass

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, mechanism, ready_for_bid_condition):
        super(DeliverContractor, self).__init__(
            agent_name, task_type=CurrentTaskDecision.TYPE_DELIVER, mechanism=mechanism)
        self.job_bid_decider = JobBidDecider(self._agent_name)
        self.ready_for_bid_condition = ready_for_bid_condition

    def should_bid_for_request(self, request):
        self.ready_for_bid_condition.sync()
        self.ready_for_bid_condition.updateComputation()
        should_bid = self.ready_for_bid_condition.satisfaction > 0.8
        return should_bid


    def generate_bid(self, request):
        return self.job_bid_decider.generate_bid(request)


    def _on_task_finished(self, finish):
        if self.mechanism.current_task is not None and self.mechanism.current_task.task == finish.job_id:
            self.mechanism.end_task()