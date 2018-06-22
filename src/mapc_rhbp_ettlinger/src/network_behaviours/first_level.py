#!/usr/bin/env python2
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviour_components.sensors import TopicSensor
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from network_behaviours.sensor_map import SensorAndConditionMap


class FirstLevelBehaviours(object):
    def __init__(self, agent, msg):

        self._agent_name = agent._agent_name
        self._agent = agent

        self.massim_sensor = TopicSensor(
            topic="/team",
            name="massium_sensor",
            message_attr="massium")
        self.score_sensor = TopicSensor(
            topic="/team",
            name="score_sensor",
            message_attr="score")

        self.sensor_map = SensorAndConditionMap(agent_name=agent._agent_name)

        self.init_behaviour_network(msg)
        self.init_behaviour_network_connections()
        self.init_coordination()
        # TODO: Add goals for assembly, massim, score, battery

    def init_behaviour_network(self, msg):
        ######################## Battery Behaviours ########################
        self.battery_charging_behaviours = BatteryChargingNetworkBehaviour(
            agent=self._agent,
            sensor_map=self.sensor_map,
            msg=msg,)


        ######################## Gathering Network Behaviour ########################
        self._gathering_network = GatheringNetworkBehaviour(
            name=self._agent_name + '/gathering',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self._agent,
            readyThreshold = 0.01, # I want this behaviour to be executed whenever at least some activation is there
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        # Add charging behaviours to gathering Network
        # self.battery_charging_behaviours.init_charge_behaviours(
        #     planner_prefix=self._gathering_network.get_manager_prefix())


        ######################## Job Network Behaviour ########################
        # self._job_execution_network = JobExecutionNetworkBehaviour(
        #     name=self._agent_name + '/job',
        #     plannerPrefix=self._agent_name,
        #     msg=msg,
        #     agent=self._agent,
        #     max_parallel_behaviours=1)
        #
        ####################### Assembly Network Behaviour ########################
        # self._assembly_network = AssembleNetworkBehaviour(
        #     name=self._agent_name + '/assemble',
        #     plannerPrefix=self._agent_name,
        #     msg=msg,
        #     agent=self._agent,
        #     max_parallel_behaviours=1)
        #
        ####################### Exploration Network Behaviour ########################
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            agent=self._agent,
            sensor_map=self.sensor_map,
            charging_components=self.battery_charging_behaviours,
            msg=msg,
            max_parallel_behaviours=1)

        ####################### Build Well Behaviour ########################
        # self.build_well_network = BuildWellNetworkBehaviour(
        #     name=self._agent_name + '/well',
        #     plannerPrefix=self._agent_name,
        #     agent=self._agent,
        #     msg=msg,
        #     max_parallel_behaviours=1)

    def init_behaviour_network_connections(self):
        ######################## Job Network Behaviour ########################

        # Only perform jobs if there is enough charge left
        # self._job_execution_network.add_precondition(
        #     precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Is done implicitly already through goal QQQ: Do I really need this?
        # TODO: #86 Not needed anymore
        # self._job_execution_network.add_precondition(
        #     precondition=self._job_execution_network.has_tasks_assigned_condition)

        # TODO: #86 Not needed anymore
        # self._job_execution_network.add_precondition(
        #     Negation(self.build_well_network.requires_action_condition)
        # )

        # self._job_execution_network.add_effect(
        #     effect=Effect(
        #             sensor_name=self._job_execution_network.has_tasks_assigned_sensor.name,
        #             indicator=-1.0, # TODO #86 The goal is to not  have a task assigned. -1.0 doesnt seem to work
        #             sensor_type=bool
        #         )
        # )

        ######################## Gathering Network Behaviour ########################

        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            plannerPrefix=self._agent_name,
            conditions=[self.sensor_map.load_fullnes_condition])

        # Gather when there is space left in load
        self._gathering_network.add_precondition(Negation(self.sensor_map.load_fullnes_condition))

        # Gather when we know some of the resource nodes already
        self._gathering_network.add_precondition(self.exploration_network.discovery_completeness_condition)

        # Gather only when next item fits in storage
        self._gathering_network.add_precondition(self.sensor_map.can_fit_more_ingredients_cond)

        self._gathering_network.add_effects_and_goals([(
            self.sensor_map.load_factor_sensor,
            Effect(
                sensor_name=self.sensor_map.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )])



        ######################## Assembly Network Behaviour ########################
        # Only assemble if there is enough battery left
        # self._assembly_network.add_precondition(
        #     precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # We should not assemble before the trunk is full. (except when we are already assembling)
        # TODO: #86 Not needed anymore


        # self._assembly_network.add_precondition(
        #     self._assembly_network.has_assemble_task_assigned_cond
        # )

        # self._assembly_network.add_precondition(
        #     Negation(self.build_well_network.requires_action_condition)
        # )

        # self._assembly_network.add_effect(
        #     Effect(
        #         sensor_name=self._assembly_network.assemble_organized_sensor.name,
        #         indicator=-1.0,
        #         sensor_type=bool
        #     )
        # )

        ######################## Exploration Network Behaviour ########################

        self._exploration_goal = GoalBase(
            name='charge_goal',
            permanent=True,
            plannerPrefix=self._agent_name,
            conditions=[self.exploration_network.resources_of_all_items_discovered_condition])

        # Only do shop exploration when enough battery left
        # self.exploration_network.add_precondition(
        #     precondition=self.battery_charging_network_behaviour._enough_battery_cond)
        #
        # Only explore when there is not task assigned
        # self.exploration_network.add_precondition(
        #     precondition=Negation(self._job_execution_network.has_tasks_assigned_condition))

        # Only explore when not all resource nodes are discovered
        # self.exploration_network.add_precondition(
        #     precondition = Disjunction(
                # Negation(self.exploration_network.all_resources_discovered_condition),  # Explore if not all resources have been found
                # Negation(self._gathering_network.next_item_fits_in_storage_condition))  # Or when storage is full withouth having any tasks
                #     TODO: #86 Not needed anymore
        # )

        # self.exploration_network.add_precondition(
        #     Negation(self.build_well_network.requires_action_condition)
        # )

        self.exploration_network.add_effect(
            Effect(
                sensor_name=self.exploration_network.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )


        ######################## Build Well Behaviour ########################

        # Only do shop exploration when enough battery left
        # self.build_well_network.add_precondition(
        #     precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Only explore when there is not task assigned
        # self.build_well_network.add_precondition(
        #     precondition=self.build_well_network.requires_action_condition)

        # self.build_well_network.add_effects_and_goals(
        #     [(self.build_well_network.target_well_integrity_sensor,
        #     Effect(
        #         sensor_name=self.build_well_network.target_well_integrity_sensor.name,
        #         indicator=1.0,
        #         sensor_type=float
        #     ))]
        # )

    def init_coordination(self):
        # self.request_assembly = RequestAssemblyBehaviour(
        #     requires_execution_steps=False,
        #     name="request_assembly",
        #     plannerPrefix=self._agent_name)
        #
        # self.request_assembly.add_precondition(Negation(self._assembly_network.has_assemble_task_assigned_cond))
        #
        # self.request_assembly.add_effect(
        #     effect=Effect(
        #         sensor_name=self._assembly_network.assemble_organized_sensor.name,
        #         indicator=1.0,
        #         sensor_type=bool
        #     )
        # )
        pass


class RequestAssemblyBehaviour(BehaviourBase):
    pass