from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from common_utils.calc import CalcUtil
from contract_net.manager import ContractNetManager
from decisions.job_combination import ChooseBestJobCombination
from decisions.p_task_decision import CurrentTaskDecision


class DeliverManager(ContractNetManager):

    def __init__(self):
        super(DeliverManager, self).__init__(task_type=CurrentTaskDecision.TYPE_DELIVER)

        self._job_combination = ChooseBestJobCombination()
        self._job = None

    def reset_manager(self):
        super(DeliverManager, self).reset_manager()

    def request_job(self, job):
        self._job = job

        request = TaskRequest(
            items=CalcUtil.get_list_from_items(job.items),
            destination=self._facility_provider.get_storage_by_name(self._job.storage_name).pos,
            destination_name=job.storage_name,
        )
        self.request_help(request)

    def get_assignments(self, bids):
        bids, item_needed_from_storage = self._job_combination.choose_best_agent_combination(self._job, bids=bids)  # TODO

        if bids is None or len(bids) == 0:
            return None

        assignments = []
        from operator import attrgetter


        # bid_attach_storage_retrival = min(bids, key=attrgetter('expected_steps'))
        agentA1InList = False
        for bid in bids:
            # if bid_attach_storage_retrival == bid:
            if bid.agent_name == "agentA1":
                agentA1InList = True
                items = item_needed_from_storage
            else:
                items =[]

            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                items=items,
                tasks=self._job.id
            )
            assignments.append(assignment)

        if not agentA1InList:
            return None
        else:
            return assignments
