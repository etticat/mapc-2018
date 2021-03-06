from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.current_task import CurrentTaskDecision
from decisions.generate_bid_from_request import GenerateBidFromRequestDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.deliver')


class DeliverContractor(ContractNetContractorBehaviour):

    def __init__(self, agent_name, current_task_decision, ready_for_bid_condition, assemble_task_decision):
        """

        :param agent_name:
        :param current_task_decision:
        :param ready_for_bid_condition:
        :param assemble_task_decision: AssembleTaskDecision
        """
        super(DeliverContractor, self).__init__(
            agent_name, task_type=CurrentTaskDecision.TYPE_DELIVER, current_task_decision=current_task_decision)

        self.assemble_task_decision = assemble_task_decision
        self.ready_for_bid_condition = ready_for_bid_condition

        self.job_bid_decider = GenerateBidFromRequestDecision(self._agent_name)

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
        Generates a bid from a request using the mechanism.
        :param request:
        :return:
        """
        return self.job_bid_decider.generate_bid(request, self.assemble_task_decision.has_task())

    def _on_task_finished(self, finish):
        """
        When a task is finished not only check the task id itself, but also check the job id. This allows to send
        TaskStop messages to job performers with job_id as target. :param finish: :return:
        """
        super(DeliverContractor, self)._on_task_finished(finish)
        if self._current_task_decision.value is not None and self._current_task_decision.value.task == finish.job_id:
            ettilog.logerr("DeliverContractor:: delivery finished")
            self._current_task_decision.end_task()

    def _on_assignment_confirmed(self, assignment):
        """
        After accepting a delivery task, cancel the assembly task if there is one, so the partner agents don't have
        to wait for the delivery and can do something else instead
        :param assignment:
        :return:
        """
        if self.assemble_task_decision.has_task():
            ettilog.logerr("DeliverContractor(%S):: ending assembly task")
        self.assemble_task_decision.end_task(notify_others=True)
