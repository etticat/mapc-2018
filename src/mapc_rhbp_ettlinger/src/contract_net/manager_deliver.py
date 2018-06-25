from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from agent_knowledge.task import TaskKnowledgeBase
from common_utils.calc import CalcUtil
from contract_net.manager import ContractNetManager
from decisions.job_combination import ChooseBestJobCombination


class DeliverManager(ContractNetManager):

    def __init__(self):
        super(DeliverManager, self).__init__(task_type=TaskKnowledgeBase.TYPE_DELIVER)

        self._job_combination = ChooseBestJobCombination()
        self._job = None

    def reset_manager(self):
        super(DeliverManager, self).reset_manager()

    def request_job(self, job):
        self._job = job

        request = TaskRequest(
            items=CalcUtil.get_list_from_items(job.items),
            destination=self._facility_provider.get_storage_by_name(self._job.storage_name).pos
        )
        self.request_help(request)

    def get_assignments(self, bids):
        bids = self._job_combination.choose_best_agent_combination(self._job, bids=bids)  # TODO

        if bids is None:
            return None

        assignments = []
        for bid in bids:
            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                items=bid.items,  # TODO: Filter out the ones we don't need
                tasks=self._job.id
            )
            assignments.append(assignment)

        return assignments
