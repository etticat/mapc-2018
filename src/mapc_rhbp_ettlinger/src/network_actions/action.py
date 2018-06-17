#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye, sys, Agent

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.resource import ResourceKnowledgebase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import TopicSensor
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from network_actions.assemble import AssembleNetworkBehaviour
from network_actions.battery import BatteryChargingNetworkBehaviour
from network_actions.build_well import BuildWellNetworkBehaviour
from network_actions.exploration import ExplorationNetworkBehaviour
from network_actions.gather import GatheringNetworkBehaviour
from network_actions.job_execution import JobExecutionNetworkBehaviour


class ActionNetworkBehaviour(NetworkBehaviour):
    def __init__(self, agent, name, msg, coordination_network_behaviour,  **kwargs):

        super(ActionNetworkBehaviour, self).__init__(name, **kwargs)
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
        self.init_behaviour_network(msg, coordination_network_behaviour)
        self.init_behaviour_network_connections(coordination_network_behaviour)

    def init_behaviour_network(self, msg, coordination_network_behaviour):
        ######################## Battery Network Behaviour ########################
        self.battery_charging_network_behaviour = BatteryChargingNetworkBehaviour(
            name=self.get_manager_prefix() + '/battery',
            plannerPrefix=self.get_manager_prefix(),
            agent=self._agent,
            msg=msg,
            max_parallel_behaviours=1)

        ######################## Job Network Behaviour ########################
        self._job_execution_network = JobExecutionNetworkBehaviour(
            name=self.get_manager_prefix() + '/job',
            plannerPrefix=self.get_manager_prefix(),
            msg=msg,
            agent=self._agent,
            max_parallel_behaviours=1)

        ######################## Gathering Network Behaviour ########################
        self._gathering_network = GatheringNetworkBehaviour(
            name=self.get_manager_prefix() + '/gathering',
            plannerPrefix=self.get_manager_prefix(),
            msg=msg,
            agent=self._agent,
            max_parallel_behaviours=1)

        ######################## Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self.get_manager_prefix() + '/assemble',
            plannerPrefix=self.get_manager_prefix(),
            coordination_network_behaviour = coordination_network_behaviour,
            msg=msg,
            agent=self._agent,
            max_parallel_behaviours=1)

        ######################## Exploration Network Behaviour ########################
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self.get_manager_prefix() + '/explore',
            plannerPrefix=self.get_manager_prefix(),
            agent=self._agent,
            msg=msg,
            max_parallel_behaviours=1)
        ######################## Build Well Behaviour ########################
        self.build_well_network = BuildWellNetworkBehaviour(
            name=self.get_manager_prefix() + '/well',
            plannerPrefix=self.get_manager_prefix(),
            agent=self._agent,
            msg=msg,
            max_parallel_behaviours=1)

        rospy.logerr("behaviour initialized")

    def init_behaviour_network_connections(self, coordination_network_behaviour):
        ######################## Battery Network Behaviour ########################

        # Battery network has the effect of charging the battery
        self.battery_charging_network_behaviour.add_effects_and_goals([(
            self.battery_charging_network_behaviour._charge_sensor,
            Effect(
                sensor_name=self.battery_charging_network_behaviour._charge_sensor.name,
                indicator=1.0,
                sensor_type=float))])

        # Only charge if charge is required
        self.battery_charging_network_behaviour.add_precondition(
            precondition=self.battery_charging_network_behaviour._require_charging_cond)

        ######################## Job Network Behaviour ########################

        # Only perform jobs if there is enough charge left
        self._job_execution_network.add_precondition(
            precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Is done implicitly already through goal QQQ: Do I really need this?
        self._job_execution_network.add_precondition(
            precondition=self._job_execution_network.has_tasks_assigned_condition)

        self._job_execution_network.add_precondition(
            Negation(self.build_well_network.requires_action_condition)
        )

        self._job_execution_network.add_effects_and_goals(
            [(
                self._job_execution_network.has_tasks_assigned_sensor,
                Effect(
                    sensor_name=self._job_execution_network.has_tasks_assigned_sensor.name,
                    indicator=-1.0,
                    sensor_type=bool
                )),
            (
                self.massim_sensor,
                Effect(
                    sensor_name=self.massim_sensor.name,
                    indicator=1.0,
                    sensor_type=float
                )
            )]
        )
        ######################## Gathering Network Behaviour ########################

        # Only gather when there is enough battery left
        self._gathering_network.add_precondition(
            precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Only gather if there is enough storage left
        # TODO: This should be a linear activator or something
        # Currently the problem is when an agent almost fills up their stock with one item while still having place
        # for one item of a different kind, they may go to the oposite side of the map. This is quite inefficient
        self._gathering_network.add_precondition(
            precondition=self._gathering_network.next_item_fits_in_storage_condition
        )
        # Only gather when there is nothing to be assembled
        self._gathering_network.add_precondition(
            precondition=Negation(coordination_network_behaviour.has_assemble_task_assigned_cond))

        # Only gather if there is no task assigned
        # TODO: We might want to allow this in the case where the agent does not have all items for the task
        # I do not think this makes sense as only accepting tasks where the agent already has all items seems more
        # promising. Might want to reconsider this at a later stage when the job execution is more clear
        self._gathering_network.add_precondition(
            precondition=Negation(self._job_execution_network.has_tasks_assigned_condition))

        # Only gather if all resources are discovered
        self._gathering_network.add_precondition(
            precondition=self.exploration_network.all_resources_discovered_condition)

        self._gathering_network.add_precondition(
            Negation(self.build_well_network.requires_action_condition)
        )

        self._gathering_network.add_effects_and_goals([(
            self._gathering_network.storage_space_after_next_item_sensor,
            Effect(
                sensor_name=self._gathering_network.storage_space_after_next_item_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )])

        ######################## Assembly Network Behaviour ########################
        # Only assemble if there is enough battery left
        self._assembly_network.add_precondition(
            precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # We should not assemble before the trunk is full. (except when we are already assembling)
        self._assembly_network.add_precondition(
            # But also keep this network active when task is assigned
            coordination_network_behaviour.has_assemble_task_assigned_cond
        )

        self._assembly_network.add_precondition(
            Negation(self.build_well_network.requires_action_condition)
        )

        self._assembly_network.add_effects_and_goals(
            [(coordination_network_behaviour.assemble_organized_sensor,
            Effect(
                sensor_name=coordination_network_behaviour.assemble_organized_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            ))]
        )

        ######################## Exploration Network Behaviour ########################

        # Only do shop exploration when enough battery left
        self.exploration_network.add_precondition(
            precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Only explore when there is not task assigned
        self.exploration_network.add_precondition(
            precondition=Negation(self._job_execution_network.has_tasks_assigned_condition))

        # Only explore when not all resource nodes are discovered
        # TODO: Maybe it makes sense that some agents continue to explore
        self.exploration_network.add_precondition(
            precondition = Disjunction(
                Negation(self.exploration_network.all_resources_discovered_condition),  # Explore if not all resources have been found
                Negation(self._gathering_network.next_item_fits_in_storage_condition))  # Or when storage is full withouth having any tasks
        )

        self.exploration_network.add_precondition(
            Negation(self.build_well_network.requires_action_condition)
        )

        self.exploration_network.add_effects_and_goals(
            [(self.exploration_network.resource_discovery_progress_sensor,
            Effect(
                sensor_name=self.exploration_network.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))]
        )


        ######################## Build Well Behaviour ########################

        # Only do shop exploration when enough battery left
        self.build_well_network.add_precondition(
            precondition=self.battery_charging_network_behaviour._enough_battery_cond)

        # Only explore when there is not task assigned
        self.build_well_network.add_precondition(
            precondition=self.build_well_network.requires_action_condition)

        self.build_well_network.add_effects_and_goals(
            [(self.build_well_network.target_well_integrity_sensor,
            Effect(
                sensor_name=self.build_well_network.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))]
        )

        rospy.logerr("behaviour connections initialized")