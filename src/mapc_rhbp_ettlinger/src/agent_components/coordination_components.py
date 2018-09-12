#!/usr/bin/env python2
from behaviour_components.conditions import Negation
from common_utils import etti_logging
from contract_net.contractor_assemble import AssembleContractor
from contract_net.contractor_deliver import DeliverContractor
from agent_components.shared_components import SharedComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.coordination')


class CoordinationComponent(object):
    """
    Component, that handles all the coordination on agent side. It instantiates the contract net contractors and keeps 
    track of them
    """
    def __init__(self, agent_name, shared_components, role):
        """

        :param agent_name:
        :param shared_components:
        :type shared_components: SharedComponents
        """

        self._assemble_contractor = AssembleContractor(
            agent_name=agent_name, role=role,
            current_task_decision=shared_components.assemble_task_decision,
            ready_for_bid_condition=shared_components.has_no_task_assigned_cond)

        self.deliver_contractor = DeliverContractor(
            agent_name=agent_name,
            current_task_decision=shared_components.deliver_task_decision,
            assemble_task_decision=shared_components.assemble_task_decision,
            ready_for_bid_condition=Negation(shared_components.has_priority_job_task_assigned_cond))
