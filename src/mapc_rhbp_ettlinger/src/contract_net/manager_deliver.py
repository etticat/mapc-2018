from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from contract_net.manager import ContractNetManager
from decisions.job_combination import ChooseBestJobCombination
from decisions.p_task_decision import CurrentTaskDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.coordination.manager.deliver')


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
        accepted_bids, item_needed_from_storage = self._job_combination.choose_best_agent_combination(self._job, bids=bids)  # TODO

        if accepted_bids is None or len(accepted_bids) == 0:
            ettilog.logerr("DeliverManager:: Unable to find suitable combination")
            ettilog.logerr("DeliverManager:: Job items: %s", str(CalcUtil.get_dict_from_items(self._job.items)))
            ettilog.logerr("DeliverManager:: Still needed items: %s", str(item_needed_from_storage))
            ettilog.logerr("DeliverManager:: Bids:")
            for bid in bids:
                ettilog.logerr("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items) ))
            return None

        assignments = []
        from operator import attrgetter

        needed_job_items = CalcUtil.get_list_from_items(self._job.items)

        # Pick the agent that is closest to the storage for picking up the stored items
        bid_attach_storage_retrival = min(accepted_bids, key=attrgetter('expected_steps'))

        for bid in accepted_bids:

            useful_items = CalcUtil.list_intersect(bid.items, needed_job_items)

            if bid_attach_storage_retrival == bid:
                useful_items += item_needed_from_storage

            needed_job_items = CalcUtil.list_diff(needed_job_items, useful_items)


            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                items=useful_items,
                tasks=self._job.id
            )
            assignments.append(assignment)

        # If the chooser chose the correct storage items, the distribution of items should always result in no more items
        # still needed to be distributed
        # TODO: Check why this happens
        # assert len(needed_job_items) == 0
        if len(needed_job_items) != 0:
            ettilog.logerr("DeliverManager:: Cant find all needed items. Maybe someone list item in meantime?")
            ettilog.logerr("DeliverManager:: Job items: %s", str(CalcUtil.get_dict_from_items(self._job.items)))
            ettilog.logerr("DeliverManager:: Still needed items: %s", str(needed_job_items))
            ettilog.logerr("DeliverManager:: Items from storage: %s", str(item_needed_from_storage))
            ettilog.logerr("DeliverManager:: Accepted bids:")
            for bid in accepted_bids:
                ettilog.logerr("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items) ))
            ettilog.logerr("DeliverManager:: Bids:")
            for bid in bids:
                ettilog.logerr("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items) ))
            return None
        return assignments
