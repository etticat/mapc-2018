import copy

from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from common_utils import etti_logging
from common_utils.calc import CalcUtil
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
        super(AssembleManager, self).__init__(task_type=CurrentTaskDecision.TYPE_ASSEMBLE)

        self._assembly_combination_decision = assembly_combination_decision

        self._facility_provider = FacilityProvider()
        self._product_provider = ProductProvider(agent_name=agent_name)

        self._assembly_agent_chooser = MainAssembleAgentDecision()

    def request_assembly(self):
        """
        Creates a request object
        :return:
        """

        # TODO: Currently we pick random assembly facility. Maybe we should decide this differently. After coordination?
        workshop = self._facility_provider.get_random_workshop()
        if workshop is not None:
            request = TaskRequest(
                destination=workshop.pos,
                items=[]
            )

            # Start coordination here
            return self.request_help(request)
        else:
            # If workshops are not initialised yet, end coordination right away
            return False

    def get_assignments(self, bids):
        """
        Takes bids and finds the best assignments to assemble items
        :param bids:
        :return:
        """
        accepted_bids = None
        finished_products = None
        assembly_instructions = None

        # Get best combinations possible from the bids in order of priority
        bids_products_array = self._assembly_combination_decision.choose_best_combinations(bids)

        if len(bids_products_array) < 1:
            # We can't make anything from those bids.
            return None

        # Try possible combinations until we find one, where the capacity of all involved agents allows to start assembly
        for bids, products, activation in bids_products_array:

            # Generate assemly instructions for all involved agents
            assembly_instructions = self._assembly_agent_chooser.generate_assembly_instructions(bids, products)

            # If we found a combination that we can assemble, stop the loop
            if assembly_instructions is not None:
                finished_products = products
                accepted_bids = bids
                break
            else:
                ettilog.loginfo("AssembleManager:: Cannot assemble %s, no capacity", str(finished_products))

        if assembly_instructions is None:
            # None of the cominations that we found is possible due to capacity
            ettilog.logerr("AssembleManager::No one has enough capacity to assemble")
            return None

        ettilog.logerr("AssembleManager:: Assembling %s", str(finished_products))

        # Create assignments for all accepted bids
        assignments = []
        for bid in accepted_bids:
            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                tasks=assembly_instructions[bid.agent_name]
            )
            assignments.append(assignment)

        return assignments
