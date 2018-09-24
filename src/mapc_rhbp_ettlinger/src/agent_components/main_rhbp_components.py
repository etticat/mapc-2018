#!/usr/bin/env python2

from agent_components.shared_components import SharedComponents
from behaviour_components.activators import GreedyActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction, Condition
from behaviour_components.goals import GoalBase
from common_utils import etti_logging
from decisions.exploration_target import ExplorationDecision, ExploreCornersDecision
from network_behaviours.assemble import AssembleNetworkBehaviour
from network_behaviours.build_well import BuildWellNetworkBehaviour
from network_behaviours.deliver_job import DeliverJobNetworkBehaviour
from network_behaviours.dismantle import DismantleNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from provider.self_organisation_provider import SelfOrganisationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.action')


class MainRhbpComponent(object):
    """
    MainRhbpComponent keeps track of the first level of Rhbp components, which includes instances of all
    Network Behaviours
    """

    def __init__(self, agent_name, shared_components, manager):
        """
        :param agent_name:
        :param shared_components:
        :type shared_components: SharedComponents
        """
        self._agent_name = agent_name
        self._shared_components = shared_components
        self._manager = manager

        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self._init_behaviour_network()
        self._init_goals()

    def _init_behaviour_network(self):
        """
        Initialise all behaviour networks
        :return:
        """
        ####################### Exploration Network Behaviour ########################
        # Responsible for finding resource nodes in the beginning of a simulation

        # ExplorationDecision picks the next destination for exploration
        exploration_decision = ExplorationDecision(
            self._self_organisation_provider.so_buffer,
            agent_name=self._agent_name)

        self._exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            exploration_decision=exploration_decision,
            priority=1,
            agent_name=self._agent_name,
            shared_components=self._shared_components,
            max_parallel_behaviours=1)

        # Only explore if no task is assigned
        # It can happen, that during initial exploration, a well position is found and a task is created.
        # In this case, we stop exploring
        self._exploration_network.add_precondition(self._shared_components.has_no_task_assigned_cond)

        # Only explore until all resources are discovered
        self._exploration_network.add_precondition(
            Negation(self._shared_components.exploration_phase_finished_condition))

        self._exploration_network.add_precondition(
            Negation(self._shared_components.opponent_well_exists_cond)
        )
        # Exploration increases the percentage of resource nodes, that have been discovered
        self._exploration_network.add_effect(
            Effect(
                sensor_name=self._shared_components.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )

        ######################## Gathering Network Behaviour ########################
        # Gather ingredients
        self._gathering_network = GatheringNetworkBehaviour(
            name=self._agent_name + '/gathering',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=2,
            shared_components=self._shared_components,
            max_parallel_behaviours=1)

        # Gather when we know all of the resource nodes already
        self._gathering_network.add_precondition(self._shared_components.exploration_phase_finished_condition)

        # Gather only when storage can fit more items
        self._gathering_network.add_precondition(self._shared_components.can_fit_more_ingredients_cond)

        # Only gather when agent has no tasks assigned
        self._gathering_network.add_precondition(self._shared_components.has_no_task_assigned_cond)

        self._gathering_network.add_precondition(Disjunction(
            Negation(self._shared_components.opponent_well_exists_cond),
            self._shared_components.at_resource_node_cond
        ))

        # Gathering increases the load of items in stock
        self._gathering_network.add_effect(
            Effect(
                sensor_name=self._shared_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))
        self._gathering_network.add_precondition(
            Negation(self._shared_components.is_forever_exploring_agent_cond))


        ####################### Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self._agent_name + '/assemble',
            plannerPrefix=self._agent_name,
            shared_components=self._shared_components,
            priority=4,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        # Only assemble when we have a task assigned
        self._assembly_network.add_precondition(self._shared_components.has_assemble_task_assigned_cond)

        # Only assemble if we don't have a priority task assigned (delivery or build well)
        self._assembly_network.add_precondition(
            Negation(self._shared_components.has_priority_job_task_assigned_cond)
        )

        # Assembly has the effect of finishing the assembly task.
        self._assembly_network.add_effect(
            Effect(
                sensor_name=self._shared_components.has_assemble_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        ######################## Job Network Behaviour ########################
        self._deliver_job_network = DeliverJobNetworkBehaviour(
            name=self._agent_name + '/job',
            plannerPrefix=self._agent_name,
            shared_components=self._shared_components,
            priority=5,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        # Only perform delivery, when there is a task assigned
        self._deliver_job_network.add_precondition(self._shared_components.has_deliver_job_task_assigned_cond)

        # Job delivery has the effect of finishing the delivery task
        self._deliver_job_network.add_effect(
            Effect(
                sensor_name=self._shared_components.has_deliver_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        ####################### Build Well Behaviour ########################
        self._build_well_network = BuildWellNetworkBehaviour(
            name=self._agent_name + '/well',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=3,
            shared_components=self._shared_components,
            max_parallel_behaviours=1)

        self._build_well_network.add_precondition(Negation(self._shared_components.is_forever_exploring_agent_cond))

        # Building wells has the effect of finishing a well task
        self._build_well_network.add_effect(
            Effect(
                sensor_name=self._shared_components.has_well_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        ####################### Dismantle Network Behaviour ########################
        self.dismantle_network = DismantleNetworkBehaviour(
            name=self._agent_name + '/dismantle',
            plannerPrefix=self._agent_name,
            priority=10,
            agent_name=self._agent_name,
            shared_components=self._shared_components,
            max_parallel_behaviours=1)

        # Only start dismantling if there are opponent wells
        self.dismantle_network.add_precondition(self._shared_components.opponent_well_exists_cond)
        self.dismantle_network.add_precondition(Negation(self._shared_components.is_forever_exploring_agent_cond))

        self.dismantle_network.add_precondition(Negation(self._shared_components.at_resource_node_cond))

        # Dismantling has the effect of reducing the opponent wells.
        self.dismantle_network.add_effect(
            effect=Effect(
                sensor_name=self._shared_components.opponent_wells_sensor.name,
                sensor_type=bool,
                indicator=-1.0
            )
        )


        ####################### Find Well Location Network Behaviour ########################
        if self._agent_name in ["TUBDAI1", "TUBDAI2", "TUBDAI3", "TUBDAI4"]:
            find_well_exploration_decision =  ExploreCornersDecision(
                self._self_organisation_provider.so_buffer, agent_name=self._agent_name)
        else:
            find_well_exploration_decision = self._exploration_network.exploration_decision

        self._find_well_location_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/welllocation',
            plannerPrefix=self._agent_name,
            exploration_decision=find_well_exploration_decision,
            priority=1,
            agent_name=self._agent_name,
            shared_components=self._shared_components,
            max_parallel_behaviours=1)

        if self._agent_name not in ["agentA1", "agentB1", "TUBDAI1"]:
            # Try to find opponent wells only when no tasks are assigned
            self._find_well_location_network.add_precondition(
                self._shared_components.has_no_task_assigned_cond)
            # Only do it when stock is full. (nothing else can be done)
            self._find_well_location_network.add_precondition(
                Negation(self._shared_components.can_fit_more_ingredients_cond)
            )
            # Only do it after exploration phase is over
            self._find_well_location_network.add_precondition(
                self._shared_components.exploration_phase_finished_condition)

        # The main effect is to increase the number of known opponent wells. It is hard to create proper goals for this.
        # Therefore use a fake effect: resource discovery, which is a side effect of this behaviour too.
        self._find_well_location_network.add_effect(
            Effect(
                sensor_name=self._shared_components.resource_discovery_progress_sensor.name,
                indicator=2.0,
                sensor_type=float
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
            priority=3,
            planner_prefix=self._agent_name,
            conditions=[self._shared_components.has_no_task_assigned_cond])

        # Otherwise we want to gather items
        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            priority=2,
            planner_prefix=self._agent_name,
            conditions=[self._shared_components.load_factor_condition])

        # If there are opponent wells, we want to destroy them
        self._dismantle_goal = GoalBase(
            name='dismantle_goal',
            permanent=True,
            priority=4,
            planner_prefix=self._agent_name,
            conditions=[Negation(self._shared_components.opponent_well_exists_cond)])

        # If there are opponent wells, we want to destroy them
        self._exploration_goal = GoalBase(
            name='exploration_goal',
            permanent=True,
            priority=1,
            planner_prefix=self._agent_name,
            conditions=[Condition(
                sensor=self._shared_components.resource_discovery_progress_sensor,
                activator=GreedyActivator()
            )])

    def step(self, guarantee_decision=True):
        """
        For each step in the simulation, a manager step is initiated.
        :param guarantee_decision:
        :return:
        """
        self._manager.step(guarantee_decision=guarantee_decision)
