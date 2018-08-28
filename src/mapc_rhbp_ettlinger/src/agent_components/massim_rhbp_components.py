#!/usr/bin/env python2
from rospy.impl.registration import RegManager

from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from common_utils import etti_logging
from decisions.exploration_target import ExplorationDecision, ExploreCornersDecision
from network_behaviours.assemble import AssembleNetworkBehaviour
from network_behaviours.build_well import BuildWellNetworkBehaviour
from network_behaviours.dismantle import DismantleNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from network_behaviours.hoarding import HoardingNetworkBehaviour
from network_behaviours.job_execution import DeliverJobNetworkBehaviour
from global_rhbp_components import GlobalRhbpComponents
from provider.self_organisation_provider import SelfOrganisationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.manager.action')


class MassimRhbpComponents(object):

    def __init__(self, agent_name, global_rhbp_components):
        """
        Manager that is responsible for all actions on the massim simulation
        :param agent_name:
        :param global_rhbp_components:
        :type global_rhbp_components: GlobalRhbpComponents
        """
        self._agent_name = agent_name
        self._global_rhbp_components = global_rhbp_components

        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self._init_behaviour_network()
        self._init_goals()

    def _init_behaviour_network(self):
        """
        Initialise all behaviour networks
        :return:
        """
        ####################### Exploration Network Behaviour ########################
        exploration_decision = ExplorationDecision(self._self_organisation_provider.so_buffer, agent_name=self._agent_name)
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            exploration_mechanism=exploration_decision,
            priority=1,
            agent_name=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.exploration_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)

        self.exploration_network.add_precondition(Negation(self._global_rhbp_components.resources_of_all_items_discovered_condition))

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
        self._gathering_network.add_precondition(
            Negation(self._global_rhbp_components.is_forever_exploring_agent_cond))

        ######################## HOARDING Network Behaviour ########################
        # self._hoarding_network = HoardingNetworkBehaviour(
        #     name=self._agent_name + '/hoarding',
        #     plannerPrefix=self._agent_name,
        #     agent_name=self._agent_name,
        #     priority=2,
        #     global_rhbp_components=self._global_rhbp_components,
        #     max_parallel_behaviours=1)
        #
        # # Only hoard when stock is full
        # self._hoarding_network.add_precondition(Negation(self._global_rhbp_components.can_fit_more_ingredients_cond))
        #
        # # Only hoard when there is no task assigned
        # self._hoarding_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)
        #
        # # Only hoard if there are finished products, that need to be stored
        # self._hoarding_network.add_precondition(self._global_rhbp_components.has_finished_products_to_store)
        #
        # # TODO: This effect is false. With the correct effect its really hard to make the planner do exactly what we want
        # self._hoarding_network.add_effect(
        #     Effect(
        #         sensor_name=self._global_rhbp_components.load_factor_sensor.name,
        #         indicator=1.0,
        #         sensor_type=float
        #     ))

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

        ####################### Explore Corners Network Behaviour ########################
        # TODO: If massium > min well type
        # id %5 == 0 (only some do that)
        # move from corner to corer (ExplorationNetwork with ExploreCornerDecision)

        ####################### Dismantle Network Behaviour ########################
        self.dismantle_network = DismantleNetworkBehaviour(
            name=self._agent_name + '/dismantle',
            plannerPrefix=self._agent_name,
            priority=10,
            agent_name=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.dismantle_network.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)
        self.dismantle_network.add_precondition(Negation(self._global_rhbp_components.can_fit_more_ingredients_cond))
        self.dismantle_network.add_precondition(self._global_rhbp_components.opponent_well_exists_cond)
        self.dismantle_network.add_precondition(Negation(self._global_rhbp_components.is_forever_exploring_agent_cond))

        self.dismantle_network.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.opponent_wells_sensor.name,
                sensor_type=bool,
                indicator=-1.0
            )
        )

        ####################### Find Well Location Network Behaviour ########################
        if self._agent_name in ["agentA1", "agentB2", "agentB3", "agentB4"]:
            find_well_exploration_decision =  ExploreCornersDecision(
                self._self_organisation_provider.so_buffer, agent_name=self._agent_name)
        else:
            find_well_exploration_decision = self.exploration_network.exploration_mechanism

        self.find_well_location_network_behaviour = ExplorationNetworkBehaviour(
            name=self._agent_name + '/welllocation',
            plannerPrefix=self._agent_name,
            exploration_mechanism=find_well_exploration_decision,
            priority=1,
            agent_name=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.find_well_location_network_behaviour.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)
        self.find_well_location_network_behaviour.add_precondition(Negation(self._global_rhbp_components.can_fit_more_ingredients_cond))
        self.find_well_location_network_behaviour.add_precondition(self._global_rhbp_components.enough_massium_to_build_well_cond)
        self.find_well_location_network_behaviour.add_precondition(Disjunction(
            Negation(self._global_rhbp_components.can_fit_more_ingredients_cond),
            self._global_rhbp_components.is_forever_exploring_agent_cond
        ))

        self.find_well_location_network_behaviour.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.resource_discovery_progress_sensor.name,
                indicator=2.0,
                sensor_type=float
            )
        )

        ####################### Idle Network Behaviour ########################
        self.idle_network_behaviour = ExplorationNetworkBehaviour(
            name=self._agent_name + '/idle',
            plannerPrefix=self._agent_name,
            exploration_mechanism=exploration_decision,
            priority=1,
            agent_name=self._agent_name,
            global_rhbp_components=self._global_rhbp_components,
            max_parallel_behaviours=1)

        self.idle_network_behaviour.add_precondition(self._global_rhbp_components.has_no_task_assigned_cond)
        self.idle_network_behaviour.add_precondition(Negation(self._global_rhbp_components.can_fit_more_ingredients_cond))
        self.idle_network_behaviour.add_precondition(self._global_rhbp_components.resources_of_all_items_discovered_condition)

        self.idle_network_behaviour.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.resource_discovery_progress_sensor.name,
                indicator=1.0,
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
            priority=200,
            planner_prefix=self._agent_name,
            conditions=[self._global_rhbp_components.has_no_task_assigned_cond])

        # We want to gather items otherwise
        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            priority=50,
            planner_prefix=self._agent_name,
            conditions=[self._global_rhbp_components.load_fullness_condition])

        # We want to destroy all opposing wells
        self._dismantle_goal = GoalBase(
            name='dismantle_goal',
            permanent=True,
            planner_prefix=self._agent_name,
            conditions=[Negation(self._global_rhbp_components.opponent_well_exists_cond)])
