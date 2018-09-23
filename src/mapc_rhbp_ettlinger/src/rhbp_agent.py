#!/usr/bin/env python2
import time

import rospy
from mac_ros_bridge.msg import RequestAction, SimStart, SimEnd, Bye, sys, Agent

from agent_components.coordination_components import CoordinationComponent
from agent_components.main_rhbp_components import MainRhbpComponent
from agent_components.shared_components import SharedComponents
from behaviour_components.managers import Manager
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.choose_best_available_job import ChooseBestAvailableJobDecision
from provider.action_provider import ActionProvider, Action
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

# import cProfile as profile

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.node.agent')


class RhbpAgent(object):
    """
    Main class of an agent. Initialises all components and is responsible for starting the manager.
    """

    # Max time the agent should take to find an action, before giving up and just recharging
    MAX_DECISION_MAKING_TIME = 14
    MAX_MANAGER_STEPS = 3
    # Flag to enable/disable the well building mechanism
    BUILD_WELL_ENABLED = True

    def __init__(self):

        self._simulation_running = False
        self._steps_since_last_action = 0

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)
        self._init_config()

        ettilog.loginfo("RhbpAgent(%s):: Starting", self._agent_name)

        # Subscribers are served in order they register. These subscribers are called BEFORE all the subscribers from
        #  rhbp components
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=self._agent_name, postfix="request_action"),
                         RequestAction, self._callback_action_request_first)

        # initialise providers
        self._simulation_provider = SimulationProvider(agent_name=self._agent_name)
        self._well_provider = WellProvider(agent_name=self._agent_name)
        self._stats_provider = StatsProvider()
        self._action_provider = ActionProvider(agent_name=self._agent_name)
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=self._agent_name)

        # initialise all components
        self._shared_components = SharedComponents(agent_name=self._agent_name)
        self._main_rhbp_components = MainRhbpComponent(
            agent_name=self._agent_name,
            shared_components=self._shared_components,
            manager=Manager(
                prefix=self._agent_name,
                max_parallel_behaviours=1)
        )

        # Coordination component will be initialised once simulation is started
        self._coordination_component = None  

        # One agent has the task of bidding for auctions
        if self._should_bid_for_auctions:
            self._job_decider = ChooseBestAvailableJobDecision(agent_name=self._agent_name)
        else:
            self._job_decider = None

        # Subscribers are served in order they register. These subscribers are called AFTER all the subscribers from
        # rhbp components
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=self._agent_name, postfix="request_action"),
                         RequestAction, self._callback_action_request_after_sensors)

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=self._agent_name, postfix="start"), SimStart,
                         self._sim_start_callback)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=self._agent_name, postfix="end"), SimEnd,
                         self._sim_end_callback)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=self._agent_name, postfix="agent"), Agent,
                         self._agent_callback)

        ettilog.logerr("RhbpAgent(%s):: Initialisation finished", self._agent_name)

    def _agent_callback(self, agent):
        """
        Every step we check the action of the agent. If the agent doesn't provide proper actions for too long restart it
        :param agent:
        :type agent: Agent
        :return:
        """
        if agent.last_action == Action.NO_ACTION:
            self._steps_since_last_action += 1

            if self._steps_since_last_action >= 5:
                # If the agent doesn't provide proper actions for 5 steps, something must have gone terribly wrong.
                # In this case kill the agent, which will tell ros to restart it.
                rospy.signal_shutdown("RhbpAgent(%s)::Agent stuck, restarting ..." % self._agent_name)
        else:
            # If a proper action has been provided, reset the counter
            self._steps_since_last_action = 0

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

        # Init coordination manager only once. Even when restarting simulation
        if self._coordination_component is None:
            # TODO: This assumes, the agent will have the same role for every simulation.
            self._coordination_component = CoordinationComponent(agent_name=self._agent_name,
                                                                 shared_components=self._shared_components,
                                                                 role=sim_start.role.name)
            ettilog.logerr("RhbpAgent(%s):: CoordinationContractors initialised", self._agent_name)
        self._simulation_running = True

    def _sim_end_callback(self, sim_end):
        """
        Callback after the simulation is over
        """
        self._simulation_running = False

    @staticmethod
    def _bye_callback(msg):
        """
        Callback when all the runs are over
        :param msg:  the message
        :type msg: Bye
        """
        ettilog.loginfo("RhbpAgent(%s):: Bye message received: %s", msg)

    def _callback_action_request_first(self, request_action):
        """
        Action request callback that is called BEFORE all other components, providers, ...
        Just keeps track at what time it was received
        :return:
        """

        self.request_time = rospy.get_rostime()

    def _callback_action_request_after_sensors(self, request_action):
        """
        Action request callback that is called AFTER all other components, providers, ...
        here we trigger the decision-making and planning
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """

        # Reset the action response flag so we can iterate until we find an action again
        self._action_provider.reset_action_response_found()

        # If current agent is responsible for auction bidding, do so
        self._handle_auction_job(request_action)

        # Build a well if we are currently out of bounds.
        self._build_well_if_needed(request_action)

        manager_steps = 0
        time_passed = 0

        # Do this until a component sets the action response flag the max decision making time is exceeded
        while (not self._action_provider.action_response_found) and time_passed < RhbpAgent.MAX_DECISION_MAKING_TIME and manager_steps < RhbpAgent.MAX_MANAGER_STEPS:
            if self._simulation_running:
                self._main_rhbp_components.step(guarantee_decision=True)

                manager_steps += 1
                time_passed = (rospy.get_rostime() - self.request_time).to_sec()
            else:
                time.sleep(0.1)

        # If decision making took too long -> log it to the console
        if time_passed >= RhbpAgent.MAX_DECISION_MAKING_TIME or manager_steps >= RhbpAgent.MAX_MANAGER_STEPS:
            ettilog.logerr("RhbpAgent(%s): Manager took %.2fs for %d steps. Action found: %s",
                           self._agent_name, time_passed, manager_steps,
                           self._action_provider.action_response_found)

        # If no action was found at all -> use recharge as fallback
        if not self._action_provider.action_response_found:
            self._action_provider.send_action(action_type=Action.RECHARGE)

    def _build_well_if_needed(self, request_action):
        """
        Checks if the agent is currently out of bounds and if so, creates a well task
        :param request_action:
        :return:
        """
        if RhbpAgent.BUILD_WELL_ENABLED:
            well_task = self._shared_components.choose_well_to_build_decision.choose(
                self._shared_components.well_task_decision,
                request_action.agent)
            if well_task is not None:
                ettilog.logerr("RhbpAgent(%s):: building well", self._agent_name)
                self._shared_components.well_task_decision.start_task(well_task)

    def _handle_auction_job(self, request_action):
        """
        If the agent is responsible for auction bidding, checks the jobs and bids for one if the activation is high
        enough. 
        :param request_action:
        :return:
        """
        if self._job_decider is not None:
            self._job_decider.save_jobs(request_action)
            self._job_decider.process_auction_jobs(request_action.auction_jobs)


if __name__ == '__main__':

    agent_name = None

    try:
        rhbp_agent = RhbpAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
