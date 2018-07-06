#!/usr/bin/env python2
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from common_utils import etti_logging
from network_behaviours.assemble import AssembleNetworkBehaviour
from network_behaviours.build_well import BuildWellNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from network_behaviours.hoarding import HoardingNetworkBehaviour
from network_behaviours.job_execution import DeliverJobNetworkBehaviour
from sensor.sensor_map import SensorAndConditionMap

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.action')

class ActionManager(Manager):
    def __init__(self, agent_name, sensor_map):
        """

        :param agent_name:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        """

        self._agent_name = agent_name
        super(ActionManager, self).__init__(prefix=self._agent_name, max_parallel_behaviours=1)

        self.sensor_map = sensor_map
        self.init_behaviour_network()
        self.init_goals()
        # TODO: Add goals for assembly, massim, score, battery

    def init_behaviour_network(self):
        ####################### Exploration Network Behaviour ########################
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            priority=1,
            agent_name=self._agent_name,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)


        self.exploration_network.add_precondition(self.sensor_map.has_no_task_assigned_cond)

        self.exploration_network.add_precondition(
            Disjunction(
                Negation(self.sensor_map.resources_of_all_items_discovered_condition),
                Conjunction(
                    Negation(self.sensor_map.can_fit_more_ingredients_cond),
                    Negation(self.sensor_map.has_finished_products_cond)
                )
            )
        )

        self.exploration_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )

        ######################## Gathering Network Behaviour ########################
        self._gathering_network = GatheringNetworkBehaviour(
            name=self._agent_name + '/gathering',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=2,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        # Gather when there is space left in load
        # We want to gather till its full. so this doesn't really make sense
        # Maybe add some other condition to encourage it more when there is still a lot to gather
        # self._gathering_network.add_precondition(Negation(self.sensor_map.load_fullness_condition))

        # Gather when we know some of the resource nodes already
        self._gathering_network.add_precondition(self.sensor_map.resources_of_all_items_discovered_condition)

        # Gather only when next item fits in storage
        self._gathering_network.add_precondition(self.sensor_map.can_fit_more_ingredients_cond)

        self._gathering_network.add_precondition(self.sensor_map.has_no_task_assigned_cond)

        self._gathering_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))

        ######################## HOARDING Network Behaviour ########################
        self._hoarding_network = HoardingNetworkBehaviour(
            name=self._agent_name + '/hoarding',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=2,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        # Only hoard when stock is full
        self._hoarding_network.add_precondition(Negation(self.sensor_map.can_fit_more_ingredients_cond))

        self._hoarding_network.add_precondition(self.sensor_map.has_finished_products_cond)

        self._hoarding_network.add_precondition(self.sensor_map.has_no_task_assigned_cond)


        self._hoarding_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.load_factor_sensor.name, # TODO: Take a proper one
                indicator=1.0,
                sensor_type=float
            ))

        ####################### Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self._agent_name + '/assemble',
            plannerPrefix=self._agent_name,
            sensor_map=self.sensor_map,
            priority=4,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        self._assembly_network.add_precondition(
            self.sensor_map.has_assemble_task_assigned_cond
        )

        self._assembly_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_assemble_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )
        ######################## Job Network Behaviour ########################
        self._job_execution_network = DeliverJobNetworkBehaviour(
            name=self._agent_name + '/job',
            plannerPrefix=self._agent_name,
            sensor_map=self.sensor_map,
            priority=5,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        self._job_execution_network.add_precondition(
            self.sensor_map.has_deliver_job_task_assigned_cond
        )

        self._job_execution_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_deliver_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        ####################### Build Well Behaviour ########################
        self.build_well_network = BuildWellNetworkBehaviour(
            name=self._agent_name + '/well',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=3,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        self.build_well_network.add_precondition(
            self.sensor_map.has_build_well_task_assigned_cond
        )

        self.build_well_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_well_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

    def init_goals(self):
        self.task_fulfillment_goal = GoalBase(
            name='task_fulfillment_goal',
            permanent=True,
            priority=200,
            plannerPrefix=self._agent_name,
            conditions=[self.sensor_map.has_no_task_assigned_cond])

        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            priority=50,
            plannerPrefix=self._agent_name,
            conditions=[self.sensor_map.load_fullness_condition])
