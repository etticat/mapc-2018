from agent_knowledge.task import TaskKnowledgeBase
from common_utils import etti_logging
from contract_net.contractor import ContractNetContractorBehaviour
from decisions.assembly_bid import ShouldBidForAssembly

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.contractor.assemble')


class AssembleContractorBehaviour(ContractNetContractorBehaviour):

    def _on_task_finished(self, finish):

        current_assemble_task = self._task_knowledge_base.get_task(self._agent_name,
                                                                   type=TaskKnowledgeBase.TYPE_ASSEMBLE)
        if current_assemble_task is not None and current_assemble_task.id == finish.id:
            self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=TaskKnowledgeBase.TYPE_ASSEMBLE)
            self._product_provider.stop_assembly()

            ettilog.logerr("AssembleContractor(%s):: Stopping task %s because %s", self._agent_name, finish.id,
                           finish.reason)

    def _on_assignment_confirmed(self, assignment):
        items_to_assemble = {}
        for task in assignment.tasks.split(","):
            task_split = task.split(":")
            if task_split[0] == "assemble":
                items_to_assemble[task_split[1]] = items_to_assemble.get(task_split[1], 0) + 1
        self._product_provider.start_assembly(items_to_assemble=items_to_assemble)

    def bid_possible(self, bid):
        return True

    def __init__(self, agent_name, role, name, **kwargs):

        super(AssembleContractorBehaviour, self).__init__(
            agent_name=agent_name, name=name, task_type=TaskKnowledgeBase.TYPE_ASSEMBLE, **kwargs)

        self._assembly_bid_chooser = ShouldBidForAssembly(agent_name=agent_name, role=role)

    def generate_bid(self, request):
        return self._assembly_bid_chooser.choose(request)
