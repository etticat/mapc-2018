from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.assembly_bid import ShouldBidForAssembly
from decisions.p_task_decision import CurrentTaskDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.assemble')


class AssembleContractor(ContractNetContractorBehaviour):

    def _on_assignment_confirmed(self, assignment):
        self._product_provider.update_assembly_goal(task_string=assignment.tasks)

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, role, mechanism, ready_for_bid_condition):
        super(AssembleContractor, self).__init__(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_ASSEMBLE, mechanism=mechanism)
        self._assembly_bid_chooser = ShouldBidForAssembly(agent_name=agent_name, role=role)
        self.ready_for_bid_condition = ready_for_bid_condition

    def should_bid_for_request(self, request):
        self.ready_for_bid_condition.sync()
        self.ready_for_bid_condition.updateComputation()
        should_bid = self.ready_for_bid_condition.satisfaction > 0.8
        return should_bid


    def generate_bid(self, request):
        return self._assembly_bid_chooser.choose(request)
