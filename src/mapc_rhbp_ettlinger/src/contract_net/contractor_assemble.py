from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.assembly_bid import ShouldBidForAssembly
from decisions.p_task_decision import CurrentTaskDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.assemble')


class AssembleContractor(ContractNetContractorBehaviour):

    def _on_assignment_confirmed(self, assignment):
        items_to_assemble = {}
        for task in assignment.tasks.split(","):
            task_split = task.split(":")
            if task_split[0] == "assemble":
                items_to_assemble[task_split[1]] = items_to_assemble.get(task_split[1], 0) + 1
        self._product_provider.start_assembly(items_to_assemble=items_to_assemble)

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, role, mechanism, ready_for_bid_condition):
        super(AssembleContractor, self).__init__(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_ASSEMBLE, mechanism=mechanism, ready_for_bid_condition=ready_for_bid_condition)
        self._assembly_bid_chooser = ShouldBidForAssembly(agent_name=agent_name, role=role)

    def generate_bid(self, request):
        return self._assembly_bid_chooser.choose(request)
