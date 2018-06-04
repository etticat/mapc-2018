#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye, sys

from agent_knowledge.resource import ResourceKnowledgebase
from common_utils.agent_utils import AgentUtils
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, Disjunction
from behaviour_components.managers import Manager
from network_behaviours.assist import AssistNetworkBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.hoarding import HoardingNetworkBehaviour
from network_behaviours.job_execution import JobExecutionNetworkBehaviour


class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    def __init__(self, agent_name):
        """

        :param agent_name:
        :type agent_name:  str
        """
        self.facilityKnowledgebase = ResourceKnowledgebase()

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)

        # Take the name from the constructor parameter in case it was started from a development environment
        if agent_name != None:
            self._agent_name = agent_name
        else:
            self._agent_name = rospy.get_param('~agent_name', "agentA1")  # default for debugging 'agentA1'

        rospy.logerr("RhbpAgent:: Starting %s", self._agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self._sim_started = False
        self._initialized = False

        # subscribe to MAC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._sim_started:  # init only once here

            self._sim_started = True
            rospy.loginfo(self._agent_name + " startet")

            # init only once, even when run restarts
            if not self._initialized:
                self.init_behaviour_network(msg)

            self._initialized = True

    def init_behaviour_network(self, msg):

        ######################## Battery Network Behaviour ########################
        self._battery_charging_network_behaviour = BatteryChargingNetworkBehaviour(
            name=self._agent_name + '/BatteryChargingNetwork',
            plannerPrefix=self._agent_name,
            agent=self,
            msg=msg,
            max_parallel_behaviours=1)

        # CONDITION: Vehicle has enough charge to function
        self.enough_battery_cond = Condition(
            sensor=self._battery_charging_network_behaviour._charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=self._battery_charging_network_behaviour.agent_charge_critical,
                isMinimum=True))

        self._battery_charging_network_behaviour.add_effects_and_goals([(
            self._battery_charging_network_behaviour._charge_sensor,
            Effect(sensor_name=self._battery_charging_network_behaviour._charge_sensor.name, indicator=1.0, sensor_type=float))])

        self._battery_charging_network_behaviour.add_precondition(self._battery_charging_network_behaviour._require_charging_cond)


        ######################## Job Network Behaviour ########################
        self._job_performance_network = JobExecutionNetworkBehaviour(
            name=self._agent_name + '/JobPerformanceNetwork',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self,
            max_parallel_behaviours=1)
        self._job_performance_network.add_precondition(
            precondition=self.enough_battery_cond)
        self._job_performance_network.add_precondition(
            precondition=self._job_performance_network.has_tasks__assigned_condition)

        ######################## Hoarding Network Behaviour ########################
        self._hoarding_network = HoardingNetworkBehaviour(
            name=self._agent_name + '/HoardingNetwork',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self,
            max_parallel_behaviours=1)

        self._hoarding_network.add_precondition(
            precondition=self.enough_battery_cond)

        self._hoarding_network.add_precondition(
            precondition=Negation(self._job_performance_network.has_tasks__assigned_condition))



        ######################## Assist Network Behaviour ########################
        self._assist_task_network = AssistNetworkBehaviour(
            agent_name=self._agent_name,
            name=self._agent_name + '/AssistNetwork',
            plannerPrefix=self._agent_name,
            msg = msg,
            max_parallel_behaviours=1
        )
        # Only assist when something is assigned
        # TODO: This should not be required. as the network should not be executed anyway when the goal is fulfilled.
        # self._assist_task_network.add_precondition(
        #     precondition=self._assist_task_network.assist_assigned_condition)

        # Only perform tasks or hoard when there is no assist required
        self._job_performance_network.add_precondition(
            precondition=Negation(self._assist_task_network.assist_assigned_condition))

        self._hoarding_network.add_precondition(
            precondition=Negation(self._assist_task_network.assist_assigned_condition))

        # TODO: Do I need these #33-1
        # Everything seems to work well without it. With it I run into errors
        # undeclared predicate has_task used in domain definition
        # [ERROR] [1527777061.931515]: PLANNER ERROR: Planner exited with failure.. Generating PDDL log files for step 11
        # self._job_performance_network.add_effects_and_goals([(
        #     self._job_performance_network.has_tasks_assigned_sensor,
        #     Effect(
        #         sensor_name=self._job_performance_network.has_tasks_assigned_sensor.name,
        #         indicator=-1.0,
        #         sensor_type=bool
        #            )
        #
        # )])

        ######################## Exploration Network Behaviour ########################
        self._shop_exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/ExplorationNetwork',
            plannerPrefix=self._agent_name,
            agent=self,
            msg=msg,
            max_parallel_behaviours=1)

        # self._shop_exploration_network.add_effects_and_goals([(
        #     self._shop_exploration_network.resource_discovery_progress_sensor,
        #     Effect(
        #         sensor_name=self._shop_exploration_network.resource_discovery_progress_sensor.name,
        #         indicator=1.0,
        #         sensor_type=bool))])

        # Only do shop exploration when enough battery left
        self._shop_exploration_network.add_precondition(
            precondition=self.enough_battery_cond)
        # Only explore when there is not task assigned
        self._shop_exploration_network.add_precondition(
            precondition=Negation(self._job_performance_network.has_tasks__assigned_condition))


        self._hoarding_network.add_precondition(
            precondition=self._shop_exploration_network.all_resources_discovered_condition)





        #only explore if we don't have a task assigned
        # self._shop_exploration_network.add_precondition(
        #     condition=Negation(self._job_performance_network.has_tasks__assigned_condition))

        # self._build_wells_network_behaviour = BuildWellNetworkBehaviour(
        #     name=self._agent_name + '/BuildWellsNetwork',
        #     plannerPrefix=self._agent_name,
        #     agent=self,
        #     msg=msg,
        #     max_parallel_behaviours=1)
        #
        # # Performing jobs produces money
        # self._shop_exploration_network.add_effect(
        #     effect=Effect(
        #         sensor_name=self._build_wells_network_behaviour.money_sensor,
        #         indicator=1.0,
        #         sensor_type=bool))

        # The overall goal is to gain score (For now to have wells)
        # TODO: Goal should be high well number. Will be changed once wells are implemented

        # TODO: Do I need these #33-1
        # Seems to work without. With them I get errors but it continues to run normally
        # self._job_performance_goal = GoalBase(
        #     name='score_goal',
        #     permanent=True,
        #     plannerPrefix=self._agent_name,
        #     conditions=[Negation(self._job_performance_network.has_tasks__assigned_condition)])



        rospy.logerr("behaviour initialized")

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + str(msg))
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Bye:" + str(msg))

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """

        start_time = rospy.get_rostime()
        rospy.logdebug("RhbpAgent::callback %s", str(msg))

        self.agent_info = msg.agent

        for resource in msg.resources:
            self.facilityKnowledgebase.add_new_resource(resource)

        self._received_action_response = False

        # self._received_action_response is set to True if a generic action response was received (send by any behaviour)
        while not self._received_action_response:
            self._manager.step()
            # action send is finally triggered by a selected behaviour

        duration = rospy.get_rostime() - start_time
        rospy.logdebug("%s: Decision-making duration %f", self._agent_name, duration.to_sec())





if __name__ == '__main__':

    agent_name = None

    if len(sys.argv) == 3 and sys.argv[1] == "--agent-name":
        agent_name = sys.argv[2]

    try:
        rhbp_agent = RhbpAgent(agent_name)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
