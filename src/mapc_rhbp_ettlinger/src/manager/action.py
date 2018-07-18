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
from global_rhbp_components import GlobalRhbpComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.action')


class ActionManager(Manager):

    def __init__(self, agent_name, global_rhbp_components):
        """
        Manager that is responsible for all actions on the massim simulation
        :param agent_name:
        :param global_rhbp_components:
        :type global_rhbp_components: GlobalRhbpComponents
        """
        super(ActionManager, self).__init__(prefix=agent_name, max_parallel_behaviours=1)

        self._agent_name = agent_name
        self._global_rhbp_components = global_rhbp_components

        self._init_behaviour_network()
        self._init_goals()

    def _init_behaviour_network(self):
        """
        Initialise all behaviour networks
        :return:
        """
        ####################### Exploration Network Behaviour ########################
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            priority=1,
            agent_name=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.exploration_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)

        self.exploration_network.add_precondition(
            Disjunction(
                Negation(self._global_rhbp_components.resources_of_all_items_discovered_condition),
                Conjunction(
                    Negation(self._global_rhbp_components.can_fit_more_ingredients_cond),
                    Negation(self._global_rhbp_components.has_finished_products_to_store)
                )
            )
        )

        self.exploration_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.resource_discovery_progress_sensor.name,
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
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        # Gather when we know all of the resource nodes already
        # TODO: We could let certain agents already start earlier, which are not good at exploration (trucks, motorcycles, ..)
        self._gathering_network.add_precondition(
            self._global_rhbp_components.resources_of_all_items_discovered_condition)

        # Gather only when storage can fit more items
        self._gathering_network.add_precondition(self._global_rhbp_components.can_fit_more_ingredients_cond)

        # Only gather when agent has no tasks assigned
        self._gathering_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)

        self._gathering_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))

        ######################## HOARDING Network Behaviour ########################
        self._hoarding_network = HoardingNetworkBehaviour(
            name=self._agent_name + '/hoarding',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=2,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        # Only hoard when stock is full
        self._hoarding_network.add_precondition(Negation(self._global_rhbp_components.can_fit_more_ingredients_cond))

        # Only hoard when there is no task assigned
        self._hoarding_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)

        # Only hoard if there are finished products, that need to be stored
        self._hoarding_network.add_precondition(self._global_rhbp_components.has_finished_products_to_store)

        # TODO: This effect is false. With the correct effect its really hard to make the planner do exactly what we want
        self._hoarding_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))

        ####################### Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self._agent_name + '/assemble',
            plannerPrefix=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            priority=4,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        # Only assemble when we have a task assigned
        self._assembly_network.add_precondition(
            self._global_rhbp_components.has_assemble_task_assigned_cond
        )

        # Only assemble if we don't have a delivery task assigned. they have priority
        self._assembly_network.add_precondition(
            Negation(self._global_rhbp_components.has_deliver_job_task_assigned_cond)
        )

        self._assembly_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.has_assemble_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )
        ######################## Job Network Behaviour ########################
        self._job_execution_network = DeliverJobNetworkBehaviour(
            name=self._agent_name + '/job',
            plannerPrefix=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            priority=5,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        # Only perform delivery, when there is a task assigned
        self._job_execution_network.add_precondition(
            self._global_rhbp_components.has_deliver_job_task_assigned_cond
        )

        self._job_execution_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.has_deliver_task_sensor.name,
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
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.build_well_network.add_precondition(
            self._global_rhbp_components.has_build_well_task_assigned_cond
        )

        self.build_well_network.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.has_well_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

    def _init_goals(self):
        """
        Initialises goals on action manager level
        :return:
        """

        # Tasks need to be fulfilled
        self.task_fulfillment_goal = GoalBase(
            name='task_fulfillment_goal',
            permanent=True,
            priority=200,
            plannerPrefix=self._agent_name,
            conditions=[self._global_rhbp_components.has_no_task_assigned_cond])

        # We want to gather items otherwise
        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            priority=50,
            plannerPrefix=self._agent_name,
            conditions=[self._global_rhbp_components.load_fullness_condition])

        # TODO: Proper working hoarding goal