#!/usr/bin/env python2

from mac_ros_bridge.msg import RequestAction, SimStart, SimEnd, Bye, sys, Agent

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from decisions.choose_best_available_job import ChooseBestAvailableJobDecision
from manager.action import ActionManager
from manager.coordination import AgentCoordinationManager
from provider.action_provider import ActionProvider, Action
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.simulation_provider import SimulationProvider
from global_rhbp_components import GlobalRhbpComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.node.agent')


class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    MAX_DECISION_MAKING_TIME = 3.9

    def __init__(self):

        self._sim_started = False

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)
        self._init_config()

        ettilog.logerr("RhbpAgent(%s):: Starting", self._agent_name)

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # Subscribers are served in order they register. These subscribers are called BEFORE all the subscribers from rhbp components
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction,
                         self._callback_action_request_first)

        # initialise providers
        self._simulation_provider = SimulationProvider(agent_name=self._agent_name)
        self._action_provider = ActionProvider(agent_name=self._agent_name)
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=self._agent_name)

        # initialise all components
        self.global_rhbp_components = GlobalRhbpComponents(agent_name=self._agent_name)
        ettilog.logerr("RhbpAgent(%s):: GlobalRhbpComponents initialized", self._agent_name)
        self._action_manager = ActionManager(agent_name=self._agent_name, global_rhbp_components=self.global_rhbp_components)
        self._coordination_manager = None # Will be initialised once simulation is started
        ettilog.logerr("RhbpAgent(%s):: ActionManager initialized", self._agent_name)

        # One agent has the task of bidding for auctions
        if self._should_bid_for_auctions:
            self._job_decider = ChooseBestAvailableJobDecision(agent_name=self._agent_name)

        # Subscribers are served in order they register. These subscribers are called AFTER all the subscribers from rhbp components
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction,
                         self._callback_action_request_after_sensors)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)
        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)
        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        ettilog.logerr("RhbpAgent(%s):: Initialisation finished", self._agent_name)

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
        RhbpAgent.MAX_DECISION_MAKING_TIME = rospy.get_param("~RhbpAgent.MAX_DECISION_MAKING_TIME",
                                                             RhbpAgent.MAX_DECISION_MAKING_TIME)

    def _sim_start_callback(self, sim_start):
        """
        Callback when simulation is started
        :param sim_start:  the message
        :type sim_start: SimStart
        """

        if not self._sim_started:
            self._sim_started = True

            # Init coordination manager only once. Even when restarting simulation
            if self._coordination_manager is None:
                DebugUtils.instant_find_resources()
                self._coordination_manager = AgentCoordinationManager(agent_name=self._agent_name,
                                                                      global_rhbp_components=self.global_rhbp_components,
                                                                      role=sim_start.role.name)
                ettilog.logerr("RhbpAgent(%s):: CoordinationContractors initialised", self._agent_name)

            self._self_organisation_provider.init_entity_listener()

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
        self._action_provider._action_response_found = False

        # If current agent is responsible for auction bidding, do so
        if self._should_bid_for_auctions:
            self._job_decider.save_jobs(request_action)
            self._job_decider.process_auction_jobs(request_action.auction_jobs)

        # TODO: When this happens, the agent is out of bounds. This can happen when the agent is close to the border
        # TODO: and tries to move to a different spot which is also close to the border.
        # TODO: This is a perfect spot for building wells, as opponents can reach this place only through grinding
        # TODO: the boarders themselves, which is very hard. (no one will implement this)
        if self._simulation_provider.out_of_bounds:
            pass

        manager_steps = 0
        time_passed = 0
        # Do this until a component sets the action response flag the max decision making time is exceeded
        while (not self._action_provider.action_response_found) and time_passed < RhbpAgent.MAX_DECISION_MAKING_TIME:
            self._action_manager.step(guarantee_decision=True)

            manager_steps += 1
            time_passed = (rospy.get_rostime() - self.request_time).to_sec()

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
