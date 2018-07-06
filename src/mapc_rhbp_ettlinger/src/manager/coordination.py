#!/usr/bin/env python2
from behaviour_components.condition_elements import Effect
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from common_utils import etti_logging
from contract_net.contractor_assemble import AssembleContractorBehaviour
from contract_net.contractor_build_well import BuildWellContractorBehaviour
from contract_net.contractor_deliver import DeliverContractorBehaviour
from sensor.sensor_map import SensorAndConditionMap

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.coordination')


class CoordinationManager(Manager):
    def __init__(self, agent_name, sensor_map, role):
        """

        :param agent_name:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        """

        self.planner_prefix = agent_name + "_coordination"
        self._agent_name = agent_name
        self.sensor_map = sensor_map
        super(CoordinationManager, self).__init__(prefix=self.planner_prefix, max_parallel_behaviours=100)


        self.init_assemble_contractor(role)
        self.init_build_well_contractor()
        self.init_deliver_contractor()

    def init_assemble_contractor(self, role):
        assemble_contractor_behaviour = AssembleContractorBehaviour(
            name="assemble_contractor_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.planner_prefix,
            role=role
        )
        assemble_contractor_behaviour.add_precondition(
            precondition=self.sensor_map.has_no_task_assigned_cond
        )
        assemble_contractor_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.sensor_map.has_assemble_task_sensor.name,
                sensor_type=bool,
                indicator=1.0
            )
        )
        assemble_coordination_goal = GoalBase(
            name='task_coordination_goal',
            permanent=True,
            plannerPrefix=self.planner_prefix,
            conditions=[self.sensor_map.has_assemble_task_assigned_cond])

    def init_deliver_contractor(self):
        deliver_contractor_behaviour = DeliverContractorBehaviour(
            name="deliver_contractor_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.planner_prefix
        )
        deliver_contractor_behaviour.add_precondition(
            precondition=self.sensor_map.has_no_task_assigned_cond
        )
        deliver_contractor_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.sensor_map.has_deliver_task_sensor.name,
                sensor_type=bool,
                indicator=1.0
            )
        )
        deliver_coordination_goal = GoalBase(
            name='deliver_coordination_goal',
            permanent=True,
            plannerPrefix=self.planner_prefix,
            conditions=[self.sensor_map.has_deliver_job_task_assigned_cond])

    def init_build_well_contractor(self):
        build_well_contractor_behaviour = BuildWellContractorBehaviour(
            name="build_well_contractor_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.planner_prefix
        )
        build_well_contractor_behaviour.add_precondition(
            precondition=self.sensor_map.has_no_task_assigned_cond
        )
        build_well_contractor_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.sensor_map.has_well_task_sensor.name,
                sensor_type=bool,
                indicator=1.0
            )
        )
        build_well_coordination_goal = GoalBase(
            name='build_well_coordination_goal',
            permanent=True,
            plannerPrefix=self.planner_prefix,
            conditions=[self.sensor_map.has_build_well_task_assigned_cond])
