from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from contract_net.manager import ContractNetManager
from decisions.p_task_decision import CurrentTaskDecision
from decisions.well_chooser import ChooseWellToBuild


class BuildWellManager(ContractNetManager):

    def __init__(self):
        super(BuildWellManager, self).__init__(task_type=CurrentTaskDecision.TYPE_BUILD_WELL)

        self._well_chooser = ChooseWellToBuild()

        self.well_type = None
        self.pos = None

    def reset_manager(self):
        super(BuildWellManager, self).reset_manager()

    def build_well(self, well_type, pos):
        self.well_type = well_type
        self.pos = pos

        request = TaskRequest(
            items=[],
            destination=pos
        )
        self.request_help(request)

    def get_assignments(self, bids):
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

