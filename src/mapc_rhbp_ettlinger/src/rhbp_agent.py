#!/usr/bin/env python2
import time

from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import AgentConfig

from threading import Thread

from mac_ros_bridge.msg import RequestAction, SimStart, SimEnd, Bye, sys, Agent

import rospy

from agent_components.coordination import AgentCoordinationManager
from agent_components.massim_rhbp_components import MassimRhbpComponents
from behaviour_components.managers import Manager
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.debug import DebugUtils
from decisions.choose_best_available_job import ChooseBestAvailableJobDecision
from decisions.well_chooser import ChooseWellToBuildDecision
from provider.action_provider import ActionProvider, Action
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.simulation_provider import SimulationProvider
from global_rhbp_components import GlobalRhbpComponents
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.node.agent')


class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    MAX_DECISION_MAKING_TIME = 7
    BUILD_WELL_ENABLED = False

    def __init__(self):

        self._sim_started = False

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)
        self._init_config()

        ettilog.loginfo("RhbpAgent(%s):: Starting", self._agent_name)

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # Subscribers are served in order they register. These subscribers are called BEFORE all the subscribers from rhbp components
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction,
                         self._callback_action_request_first)

        # initialise providers
        self._simulation_provider = SimulationProvider(agent_name=self._agent_name)
        self._well_provider = WellProvider(agent_name=self._agent_name)
        self._stats_provider = StatsProvider(agent_name=self._agent_name)
        self._action_provider = ActionProvider(agent_name=self._agent_name)
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=self._agent_name)
        self._self_organisation_provider.init_entity_listener()

        # initialise all components
        self.global_rhbp_components = GlobalRhbpComponents(agent_name=self._agent_name)
        ettilog.loginfo("RhbpAgent(%s):: GlobalRhbpComponents initialized", self._agent_name)
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)
        self._massim_rhbp_components = MassimRhbpComponents(agent_name=self._agent_name,
                                                            global_rhbp_components=self.global_rhbp_components)
        self._coordination_manager = None  # Will be initialised once simulation is started
        ettilog.loginfo("RhbpAgent(%s):: ActionManager initialized", self._agent_name)

        # One agent has the task of bidding for auctions
        if self._should_bid_for_auctions:
            self._job_decider = ChooseBestAvailableJobDecision(agent_name=self._agent_name)

        # Subscribers are served in order they register. These subscribers are called AFTER all the subscribers from rhbp components
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction,
                         self._callback_action_request_after_sensors)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)
        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)
        rospy.Subscriber("/agentConfig", AgentConfig, self._callback_agent_config)
        # rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        ettilog.logerr("RhbpAgent(%s):: Initialisation finished", self._agent_name)

    def _callback_agent_config(self, agent_conf):
        """

        :param agent_conf:
        :type agent_conf: AgentConfig
        :return:
        """

        ettilog.logerr("RhbpAgent(%s):: Received agent conf:  %s", self._agent_name, str(agent_conf))

        for config in agent_conf.configs:
            type, value = config.value.split(",")

            if "int" in type:
                value = int(value)
            elif "float" in type:
                value = float(value)
            elif "bool" in type:
                value = bool(value)
            else:
                ettilog.logerr("RhbpAgent(%s):: invalid config for %s", self._agent_name, config.key)

            rospy.set_param(config.key, value)

    def _init_config(self):
        """
        Initialises the config parameters from the rospy config
        :return:
        """

        # Inject the agent name from command line into rospy to allow for easier debugging
        if len(sys.argv) == 3 and sys.argv[1] == "--agent-name":
            self._agent_name = sys.argv[2]
        else:
            self._agent_name = rospy.get_param('~agent_name')
        self._should_bid_for_auctions = rospy.get_param('~bid_for_auctions', False)
        RhbpAgent.MAX_DECISION_MAKING_TIME = rospy.get_param("RhbpAgent.MAX_DECISION_MAKING_TIME",
                                                             RhbpAgent.MAX_DECISION_MAKING_TIME)
        RhbpAgent.BUILD_WELL_ENABLED = rospy.get_param("RhbpAgent.BUILD_WELL_ENABLED",
                                                       RhbpAgent.BUILD_WELL_ENABLED)

    def _sim_start_callback(self, sim_start):
        """
        Callback when simulation is started
        :param sim_start:  the message
        :type sim_start: SimStart
        """
        self._init_config()

        if not self._sim_started:

            # Init coordination manager only once. Even when restarting simulation
            if self._coordination_manager is None:
                # DebugUtils.instant_find_resources(self._agent_name)
                self._coordination_manager = AgentCoordinationManager(agent_name=self._agent_name,
                                                                      global_rhbp_components=self.global_rhbp_components,
                                                                      role=sim_start.role.name)
                ettilog.logerr("RhbpAgent(%s):: CoordinationContractors initialised", self._agent_name)
        self._sim_started = True

    def _sim_end_callback(self, sim_end):
        """
        Callback after the simulation is over
        :param sim_end:  the message
        :type sim_end: SimEnd
        """
        ettilog.loginfo("SimEnd:" + str(sim_end))
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        Callback when all the runs are over
        :param msg:  the message
        :type msg: Bye
        """
        ettilog.loginfo("Bye:" + str(msg))

    def _callback_action_request_first(self, request_action):
        """
        Action request callback that is called BEFORE all other components, providers, ...
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """
        self.request_time = rospy.get_rostime()

    def _callback_action_request_after_sensors(self, request_action):
        """
        Action request callback that is called AFTER all other components, providers, ...
        here we just trigger the decision-making and plannig
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """
        # We run until an action is found. reset flag here
        self._action_provider.reset_action_response_found()

        # If current agent is responsible for auction bidding, do so
        if self._should_bid_for_auctions:
            self._job_decider.save_jobs(request_action)
            self._job_decider.process_auction_jobs(request_action.auction_jobs)


        # Build a well if we are currently out of bounds.

        if RhbpAgent.BUILD_WELL_ENABLED:
            well_task = self.global_rhbp_components._choose_well_to_build_decision.choose(self.global_rhbp_components.well_task_mechanism,
                                                                   request_action.agent)
            if well_task is not None:
                ettilog.loginfo("RhbpAgent(%s):: building well", self._agent_name)
                self.global_rhbp_components.well_task_mechanism.start_task(well_task)

        manager_steps = 0
        time_passed = 0
        # Do this until a component sets the action response flag the max decision making time is exceeded
        while (not self._action_provider.action_response_found) and time_passed < RhbpAgent.MAX_DECISION_MAKING_TIME:
            if self._sim_started:
                self._manager.step(guarantee_decision=True)

                manager_steps += 1
                time_passed = (rospy.get_rostime()   - self.request_time).to_sec()
            else:
                time.sleep(0.5)

        # If decision making took too long -> log
        if time_passed > RhbpAgent.MAX_DECISION_MAKING_TIME:
            ettilog.logerr("RhbpAgent(%s): Manager took too long: %.2fs for %d steps. Action found: %s",
                            self._agent_name, time_passed, manager_steps, self._action_provider._action_response_found)

        # If no action was found at all -> use recharge as fallback
        if not self._action_provider.action_response_found:
            self._action_provider.send_action(action_type=Action.RECHARGE)


if __name__ == '__main__':

    agent_name = None

    try:
        rhbp_agent = RhbpAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
