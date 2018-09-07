#!/usr/bin/env python2
from behaviour_components.conditions import Negation
from common_utils import etti_logging
from contract_net.contractor_assemble import AssembleContractor
from contract_net.contractor_deliver import DeliverContractor
from agent_components.shared_components import SharedComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.coordination')


class CoordinationComponent(object):
    """
    Manager, that handles all the coordination on agent level.
    The word Manager refers to Manager in the context of Manager in rhbp components an not in Context of Contract Net
    """
    def __init__(self, agent_name, shared_components, role):
        """

        :param agent_name:
        :param shared_components:
        :type shared_components: SharedComponents
        """

        self._agent_name = agent_name
        self.shared_components = shared_components

        self.assemble_contractor_behaviour = AssembleContractor(agent_name=self._agent_name, role=role,
                                                                current_task_mechanism=self.shared_components.assemble_task_mechanism,
                                                                ready_for_bid_condition=self.shared_components.has_no_task_assigned_cond)

        self.deliver_contractor_behaviour = DeliverContractor(agent_name=self._agent_name,
                                                              current_task_mechanism=self.shared_components.deliver_task_mechanism,
                                                              assemble_task_mechanism=self.shared_components.assemble_task_mechanism,
                                                              ready_for_bid_condition=Negation(
                                                                  self.shared_components.has_deliver_job_task_assigned_cond))

