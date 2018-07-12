#!/usr/bin/env python2
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from common_utils import etti_logging
from contract_net.contractor_assemble import AssembleContractor
from contract_net.contractor_build_well import BuildWellContractor
from contract_net.contractor_deliver import DeliverContractor
from sensor.sensor_map import SensorAndConditionMap

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.coordination')


class CoordinationManager(object):
    def __init__(self, agent_name, sensor_map, role):
        """

        :param agent_name:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        """

        self.planner_prefix = agent_name + "_coordination"
        self._agent_name = agent_name
        self.sensor_map = sensor_map

        self.init_assemble_contractor(role)
        self.init_build_well_contractor()
        self.init_deliver_contractor()

    def init_assemble_contractor(self, role):
        self.assemble_contractor_behaviour = AssembleContractor(agent_name=self._agent_name, role=role, mechanism=self.sensor_map.assemble_task_mechanism,
                                                                ready_for_bid_condition=self.sensor_map.has_no_task_assigned_cond)

    def init_deliver_contractor(self):
        self.deliver_contractor_behaviour = DeliverContractor(agent_name=self._agent_name, mechanism=self.sensor_map.deliver_task_mechanism,
                                                                ready_for_bid_condition=Negation(self.sensor_map.has_deliver_job_task_assigned_cond))

    def init_build_well_contractor(self):
        self.build_well_contractor_behaviour = BuildWellContractor(agent_name=self._agent_name, mechanism=self.sensor_map.well_task_mechanism,
                                                                ready_for_bid_condition=self.sensor_map.has_no_task_assigned_cond)

    def step(self):
        pass
