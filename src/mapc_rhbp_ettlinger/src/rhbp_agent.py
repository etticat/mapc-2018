#!/usr/bin/env python2
import cProfile as profile
import time

import numpy as np
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye, sys, Agent

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from manager.action import ActionManager
from manager.coordination import CoordinationManager
from planner import Planner
from provider.action_provider import ActionProvider, Action
from provider.provider_info_distributor import ProviderInfoDistributor
from provider.self_organisation_provider import SelfOrganisationProvider
from sensor.sensor_map import SensorAndConditionMap

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.rhbp')


class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    def __init__(self, agent_name):
        """

        :param agent_name:
        :type agent_name:  str
        """
        # Take the name from the constructor parameter in case it was started from a development environment
        self.time_storage = []
        if agent_name != None:
            self._agent_name = agent_name
        else:
            self._agent_name = rospy.get_param('~agent_name', "agentA1")  # default for debugging 'agentA1'

        ettilog.logerr("RhbpAgent(%s):: Starting", self._agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # self.pr = profile.Profile()
        # self.pr.disable()

        # ensure also max_parallel_behaviours during debugging
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback_before_sensors)

        self.sensor_map = SensorAndConditionMap(agent_name=self._agent_name)
        ettilog.logerr("RhbpAgent(%s):: Sensor map initialized", self._agent_name)
        self._action_manager = ActionManager(agent_name=self._agent_name, sensor_map=self.sensor_map)
        self.action_provider = ActionProvider(agent_name=self._agent_name)
        ettilog.logerr("RhbpAgent(%s):: Behaviours initialized", self._agent_name)

        self._provider_info_distributor = ProviderInfoDistributor()
        self.self_organisation_provider = SelfOrganisationProvider(agent_name=self._agent_name)

        self._sim_started = False
        self._initialized = False

        # subscribe to MAC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback_after_sensors)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "agent", Agent, self._agent_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        ettilog.logerr("RhbpAgent(%s):: Constructor finished", self._agent_name)


    def _sim_start_callback(self, sim_start):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param sim_start:  the message
        :type sim_start: SimStart
        """

        self._provider_info_distributor.callback_sim_start(sim_start)
        if not self._sim_started:  # init only once here

            self._sim_started = True

            # init only once, even when run restarts
            if not self._initialized:
                self._coordination_manager = CoordinationManager(agent_name=self._agent_name,
                                                                 sensor_map=self.sensor_map, role=sim_start.role.name)
                ettilog.logerr("RhbpAgent(%s):: Initialisation finished", self._agent_name)
                # DebugUtils.start_thread_counter()
                DebugUtils.instant_find_resources()

                self.self_organisation_provider.init_entity_listener()
            self._initialized = True


    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        ettilog.loginfo("SimEnd:" + str(msg))
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        ettilog.loginfo("Bye:" + str(msg))

    def _agent_callback(self, agent):
        self._provider_info_distributor.callback_agent(agent)

    def _action_request_callback_before_sensors(self, request_action):
        """
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """
        self.request_time = time.time()
        # self.pr.enable()


    def _action_request_callback_after_sensors(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """
        sensor_finished_time = time.time()
        request_action.simulation_step

        self._provider_info_distributor.callback_request_action(request_action)

        provider_info_distributor_finished_time = time.time()

        start_time = rospy.get_rostime()
        # ettilog.logerr("-------------------------------------- Step %d (action_request) --------------------------------------", msg.simulation_step)

        self.agent_info = request_action.agent

        self.action_provider.current_action_sent = False


        start_time = rospy.get_rostime()
        steps = 0
        # wait for generic action response (send by any behaviour)
        while not self.action_provider.current_action_sent:
            if self._initialized:
                steps += 1


                self._action_manager.step(guarantee_decision=True)  # selected behaviours eventually trigger action

            else:
                time.sleep(0.2)
            # Recharge if decision-making-time > 3.9 seconds
            if (rospy.get_rostime() - start_time).to_sec() > 3.9:
                ettilog.logerr(
                    "%s: 3.9 seconds (%d steps). Decision-making duration exceeded. Recharging. initialized: %s",
                    self._agent_name, steps, str(self._initialized))
                self.fallback_recharge()
                break

        if self._initialized:
            self._coordination_manager.step()
        duration = rospy.get_rostime() - start_time
        ettilog.loginfo("%s: Decision-making duration %f", self._agent_name, duration.to_sec())

        manager_finished_time = time.time()
        manager_time = manager_finished_time - provider_info_distributor_finished_time
        sensor_time = sensor_finished_time - self.request_time

        time_tuple = (manager_time, steps, manager_time /steps)
        self.time_storage.append(time_tuple)
        # rospy.logerr("--------------------------------------")
        rospy.logerr("%d:::: curr: manager: %f, steps:%f, duration/step: %f" , request_action.simulation_step, time_tuple[0], time_tuple[1], time_tuple[2])
        # a = np.mean(np.array(self.time_storage), axis=0)
        # rospy.logerr("mean: sensor: %f, step: %f, steps:%f, duration/step: %f", a[0], a[1], a[2], a[3])
        # a = np.median(np.array(self.time_storage), axis=0)
        # rospy.logerr("medi: sensor: %f, step: %f, steps:%f, duration/step: %f", a[0], a[1], a[2], a[3])
        #

        # self.pr.disable()
        # self.pr.dump_stats("stats" + str(request_action.simulation_step) + ".pstat")

    def fallback_recharge(self):
        pub_generic_action = rospy.Publisher(
            self._agent_topic_prefix + 'generic_action',
            GenericAction, queue_size=10)
        self.action_provider.send_action(action_type=Action.RECHARGE)


if __name__ == '__main__':

    agent_name = None

    if len(sys.argv) == 3 and sys.argv[1] == "--agent-name":
        agent_name = sys.argv[2]
    try:

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)

        agent_n = rospy.get_param('~agent_name', "agentA1")
        if agent_n == "planner":
            # TODO #86: It doesnt work to put the planner as seperate node. It always says the py file cant be found.
            # TODO: Need to figure out what the problem is
            planner = Planner(agent_name="agentA1")
        else:
            rhbp_agent = RhbpAgent(agent_name)

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
