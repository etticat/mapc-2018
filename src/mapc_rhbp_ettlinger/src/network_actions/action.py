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
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from network_actions.assemble import AssembleNetworkBehaviour
from network_actions.battery import BatteryChargingNetworkBehaviour
from network_actions.exploration import ExplorationNetworkBehaviour
from network_actions.gather import GatheringNetworkBehaviour
from network_actions.job_execution import JobExecutionNetworkBehaviour


class ActionNetworkBehaviour(NetworkBehaviour):
    def __init__(self, agent, name, msg, coordination_network_behaviour,  **kwargs):

        super(ActionNetworkBehaviour, self).__init__(name, **kwargs)
        self._agent_name = agent._agent_name
        self._agent = agent
        self.init_behaviour_network(msg, coordination_network_behaviour)
        self.init_behaviour_network_connections(coordination_network_behaviour)

    def init_behaviour_network(self, msg, coordination_network_behaviour):
        ######################## Battery Network Behaviour ########################
        self._battery_charging_network_behaviour = BatteryChargingNetworkBehaviour(
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
        self._exploration_network = ExplorationNetworkBehaviour(
            name=self.get_manager_prefix() + '/explore',
            plannerPrefix=self.get_manager_prefix(),
            agent=self._agent,
            msg=msg,
            max_parallel_behaviours=1)

        ######################## Build Well Network Behaviour ########################
        # self._build_wells_network_behaviour = BuildWellNetworkBehaviour(
        #     name=self.get_manager_prefix() + '/BuildWellsNetwork',
        #     plannerPrefix=self.get_manager_prefix(),
        #     agent=self,
        #     msg=msg,
        #     max_parallel_behaviours=1)

        rospy.logerr("behaviour initialized")

    def init_behaviour_network_connections(self, coordination_network_behaviour):
        ######################## Battery Network Behaviour ########################

        # Battery network has the effect of charging the battery
        self._battery_charging_network_behaviour.add_effects_and_goals([(
            self._battery_charging_network_behaviour._charge_sensor,
            Effect(
                sensor_name=self._battery_charging_network_behaviour._charge_sensor.name,
                indicator=1.0,
                sensor_type=float))])

        # Only charge if charge is required
        self._battery_charging_network_behaviour.add_precondition(
            precondition=self._battery_charging_network_behaviour._require_charging_cond)

        ######################## Job Network Behaviour ########################

        # Only perform jobs if there is enough charge left
        self._job_execution_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # Is done implicitly already through goal QQQ: Do I really need this?
        self._job_execution_network.add_precondition(
            precondition=self._job_execution_network.has_tasks_assigned_condition)

        ######################## Gathering Network Behaviour ########################

        # Only gather when there is enough battery left
        self._gathering_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

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
            precondition=self._exploration_network.all_resources_discovered_condition)

        ######################## Assembly Network Behaviour ########################
        # Only assemble if there is enough battery left
        self._assembly_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # We should not assemble before the trunk is full. (except when we are already assembling)
        self._assembly_network.add_precondition(
            # But also keep this network active when task is assigned
            coordination_network_behaviour.has_assemble_task_assigned_cond
        )

        ######################## Exploration Network Behaviour ########################

        # Only do shop exploration when enough battery left
        self._exploration_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # Only explore when there is not task assigned
        self._exploration_network.add_precondition(
            precondition=Negation(self._job_execution_network.has_tasks_assigned_condition))

        # Only explore when not all resource nodes are discovered
        # TODO: Maybe it makes sense that some agents continue to explore
        self._exploration_network.add_precondition(
            precondition=Negation(self._exploration_network.all_resources_discovered_condition)
        )

        # Seems to work without. With them I get errors but it continues to run normally
        self._job_performance_goal = GoalBase(
            name='score_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self._job_execution_network.has_tasks_assigned_condition)])

        rospy.logerr("behaviour connections initialized")