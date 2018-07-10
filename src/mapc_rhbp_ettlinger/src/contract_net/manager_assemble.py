import random

from mapc_rhbp_ettlinger.msg import TaskRequest, TaskAssignment

from contract_net.manager import ContractNetManager
from decisions.assembly_combination import AssemblyCombinationDecision
from decisions.p_task_decision import CurrentTaskDecision
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider


class AssembleManager(ContractNetManager):

    def __init__(self, agent_name):
        super(AssembleManager, self).__init__(task_type=CurrentTaskDecision.TYPE_ASSEMBLE)

        self._facility_provider = FacilityProvider()
        self._assembly_combination = AssemblyCombinationDecision()
        self._product_provider = ProductProvider(agent_name=agent_name)

    def reset_manager(self):

        super(AssembleManager, self).reset_manager()

    def request_assembly(self):

        request = TaskRequest(
            destination=self._facility_provider.get_random_workshop().pos,
            items=[]
        )

        self.request_help(request)

    def get_assignments(self, bids):
        accepted_bids, finished_products = self._assembly_combination.choose(bids)

        if finished_products is None:
            return None

        assembly_instructions = self.generate_assembly_instructions(accepted_bids, finished_products)

        assignments = []
        for bid in accepted_bids:
            assignment = TaskAssignment(
                id=bid.id,
                bid=bid,
                tasks=assembly_instructions[bid.agent_name]
            )
            assignments.append(assignment)

        return assignments

    def generate_assembly_instructions(self, accepted_bids, finished_products):
        res = {}
        agents = []

        for bid in accepted_bids:
            res[bid.agent_name] = ""
            agents.append(bid.agent_name)

        # Distributing tasks: TODO: currently manager builds everything. in future others may build
        for item in finished_products:
            selected_agent = random.choice(agents)
            # TODO: Every agent could be selected at different points
            # For now let this be done randomly. In future this has to be selected better
            for agent in agents:
                if res[agent] is not "":
                    res[agent] = res[agent] + ","

                if agent == selected_agent:
                    res[agent] = res[agent] + "assemble:" + item
                else:
                    res[agent] = res[agent] + "assist:" + selected_agent

        return res
