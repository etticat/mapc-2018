from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.should_bid_for_assembly import ShouldBidForAssemblyDecision
from decisions.current_task import CurrentTaskDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.assemble')


class AssembleContractor(ContractNetContractorBehaviour):
    """
    Contractor class for accepting assembly coordination requests
    """

    def __init__(self, agent_name, role, current_task_decision, ready_for_bid_condition):
        self.ready_for_bid_condition = ready_for_bid_condition
        self._assembly_bid_chooser = ShouldBidForAssemblyDecision(agent_name=agent_name, role=role)

        super(AssembleContractor, self).__init__(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_ASSEMBLE,
                                                 current_task_decision=current_task_decision)

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
        Generates a bid. Use a decision mechanism for this
        :param request:
        :return:
        """
        return self._assembly_bid_chooser.generate_assembly_bid(request)
