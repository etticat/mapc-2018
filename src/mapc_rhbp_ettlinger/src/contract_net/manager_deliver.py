from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from contract_net.manager import ContractNetManager
from decisions.choose_best_job_agent_combination import ChooseBestJobAgentCombinationDecision
from decisions.current_task import CurrentTaskDecision
from operator import attrgetter

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.coordination.manager.deliver')


class DeliverManager(ContractNetManager):
    """
    Manager for job deliver coordination
    """

    def __init__(self, agent_name):
        super(DeliverManager, self).__init__(task_type=CurrentTaskDecision.TYPE_DELIVER, agent_name=agent_name)

        self._job_combination = ChooseBestJobAgentCombinationDecision(agent_name=agent_name)
        self._job = None

    def request_job(self, job):
        """
        Requests help for a job
        :param job:
        :return:
        """

        # Keep an instance of the job itself for later use in assignments
        self._job = job

        request = TaskRequest(
            items=CalcUtil.get_list_from_items(job.items),
            destination=self._facility_provider.get_storage_by_name(self._job.storage_name).pos,
            destination_name=job.storage_name,
        )
        self.request_help(request)

    def get_assignments(self, bids):
        """
        Generates assignments from bids. finds the combination, that makes most sense.
        :param bids:
        :return:
        """
        accepted_bids, item_needed_from_storage = self._job_combination.choose_best_agent_combination(self._job,
                                                                                                      bids=bids)

        if accepted_bids is None or len(accepted_bids) == 0:
            ettilog.logerr("DeliverManager:: Unable to find suitable combination for %s", self._job.id)
            ettilog.loginfo("DeliverManager:: Job items: %s", str(CalcUtil.get_dict_from_items(self._job.items)))
            ettilog.loginfo("DeliverManager:: Still needed items: %s", str(item_needed_from_storage))
            ettilog.loginfo("DeliverManager:: Bids:")
            for bid in bids:
                ettilog.loginfo("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items)))
            return None

        assignments = []

        # Get all the items that are needed to perform the job
        needed_job_items = CalcUtil.get_list_from_items(self._job.items)

        # Select the agent that is closest to the storage for picking up the stored items
        bid_attach_storage_retrieval = min(accepted_bids, key=attrgetter('expected_steps'))

        for bid in accepted_bids:

            # Get all the items from the agent, that are useful for the job
            useful_items = CalcUtil.list_intersect(bid.items, needed_job_items)

            # If this bid is responsible for picking up stored items, add those to the items list
            if bid_attach_storage_retrieval == bid:
                useful_items += item_needed_from_storage

            # remove the assigned items from the still needed items array
            needed_job_items = CalcUtil.list_diff(needed_job_items, useful_items)

            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                items=useful_items,
                tasks=self._job.id
            )
            assignments.append(assignment)

        # If the chooser chose the correct storage items, the distribution of items should always result in no more
        # items still needed to be distributed. Therefore, this should never happen.
        if len(needed_job_items) != 0:
            ettilog.logerr("DeliverManager:: Cant find all needed items. Maybe someone list item in meantime?")
            ettilog.logerr("DeliverManager:: Job items: %s", str(CalcUtil.get_dict_from_items(self._job.items)))
            ettilog.logerr("DeliverManager:: Still needed items: %s", str(needed_job_items))
            ettilog.logerr("DeliverManager:: Items from storage: %s", str(item_needed_from_storage))
            ettilog.logerr("DeliverManager:: Accepted bids:")
            for bid in accepted_bids:
                ettilog.logerr("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items)))
            ettilog.logerr("DeliverManager:: Bids:")
            for bid in bids:
                ettilog.logerr("DeliverManager:: --- %s", str(bid.agent_name + ":" + str(bid.items)))
            return None
        return assignments

    def reset_history(self):
        pass
