#!/usr/bin/env python2
from behaviour_components.conditions import Negation
from common_utils import etti_logging
from contract_net.contractor_assemble import AssembleContractor
from contract_net.contractor_build_well import BuildWellContractor
from contract_net.contractor_deliver import DeliverContractor
from global_rhbp_components import GlobalRhbpComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.coordination')


class AgentCoordinationManager(object):
    """
    Manager, that handles all the coordination on agent level.
    The word Manager refers to Manager in the context of Manager in rhbp components an not in Context of Contract Net
    """
    def __init__(self, agent_name, global_rhbp_components, role):
        """

        :param agent_name:
        :param global_rhbp_components:
        :type global_rhbp_components: GlobalRhbpComponents
        """

        self._agent_name = agent_name
        self.global_rhbp_components = global_rhbp_components

        self.assemble_contractor_behaviour = AssembleContractor(agent_name=self._agent_name, role=role,
                                                                current_task_mechanism=self.global_rhbp_components.assemble_task_mechanism,
                                                                ready_for_bid_condition=self.global_rhbp_components.has_no_task_assigned_cond)

        self.deliver_contractor_behaviour = DeliverContractor(agent_name=self._agent_name,
                                                              current_task_mechanism=self.global_rhbp_components.deliver_task_mechanism,
                                                              ready_for_bid_condition=Negation(
                                                                  self.global_rhbp_components.has_deliver_job_task_assigned_cond))

        self.build_well_contractor_behaviour = BuildWellContractor(agent_name=self._agent_name,
                                                                   current_task_mechanism=self.global_rhbp_components.well_task_mechanism,
                                                                   ready_for_bid_condition=self.global_rhbp_components.has_no_task_assigned_cond)
