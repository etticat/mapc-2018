import copy

from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from common_utils import etti_logging
from contract_net.manager import ContractNetManager
from decisions.main_assemble_agent import MainAssembleAgentDecision
from decisions.current_task import CurrentTaskDecision
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.coordination.manager.assemble')


class AssembleManager(ContractNetManager):
    """
    Manager for assembly coordination
    """

    def __init__(self, agent_name, assembly_combination_decision):
        super(AssembleManager, self).__init__(task_type=CurrentTaskDecision.TYPE_ASSEMBLE, agent_name=agent_name)

        self._assembly_combination_decision = assembly_combination_decision

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)

    def request_assembly(self):
        """
        Creates a request object
        :return:
        """

        request = TaskRequest(
            items=[]
        )

        # Start coordination here
        return self.request_help(request)

    def get_assignments(self, bids):
        """
        Takes bids and finds the best assignments to assemble items
        :param bids:
        :return:
        """

        # Get best combinations possible from the bids in order of priority
        res = self._assembly_combination_decision.choose_best_combinations(bids)
        if res is None:
            return

        bids, products, activation, destination, assembly_instructions = res

        finished_products = products
        accepted_bids = bids
        best_destination = destination

        ettilog.logerr("AssembleManager:: Assembling %s activation: %f", str(finished_products), activation)


        # Create assignments for all accepted bids
        assignments = []
        agents = [bid.agent_name for bid in accepted_bids]
        for bid in accepted_bids:
            bid.request.destination = best_destination.pos
            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                agents=agents,
                tasks=assembly_instructions[bid.agent_name]
            )
            assignments.append(assignment)

        return assignments
