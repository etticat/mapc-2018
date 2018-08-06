from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from contract_net.manager import ContractNetManager
from decisions.current_task import CurrentTaskDecision
from decisions.well_chooser import ChooseWellToBuildDecision


class BuildWellManager(ContractNetManager):
    """
    Manager for build well coordination.
    """

    def __init__(self, well_chooser, agent_name):
        super(BuildWellManager, self).__init__(task_type=CurrentTaskDecision.TYPE_BUILD_WELL, agent_name=agent_name)

        self.well_type = None
        self.pos = None

        self._well_chooser = well_chooser

    def build_well(self, well_type, pos):
        """
        Start building well at a defined position
        :param well_type:
        :param pos:
        :return:
        """

        self.well_type = well_type
        self.pos = pos

        request = TaskRequest(
            items=[],
            destination=pos
        )
        self.request_help(request)

    def get_assignments(self, bids):
        """
        Choose the best bids to start building. Curently only 1 bid is chosen.
        :param bids:
        :return:
        """
        bids = self._well_chooser.choose_agent_for_building(bids)

        if bids is None:
            return None

        assignments = []
        for bid in bids:
            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                tasks=self.well_type
            )
            assignments.append(assignment)

        return assignments

    def _on_task_acknowledged(self, task_id):
        """
        When the task is confirmed, save the cost into an array
        :param task_id:
        :return:
        """
        self._well_chooser.start_task(self._id, self.well_type)

    def _on_task_finished(self, task_stop):
        """
        When task is finished, delete it from well_chooser
        :param task_stop:
        :return:
        """
        self._well_chooser.end_task(task_stop.id)