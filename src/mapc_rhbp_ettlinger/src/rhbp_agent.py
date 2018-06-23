#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye, sys

from behaviours.generic_action import GenericActionBehaviour, Action
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from common_utils.debug import DebugUtils
from coordination.assemble_contractor import AssembleContractor
from coordination.job_contractor import JobContractor
from network_behaviours.first_level import ActionManager
from provider.provider_info_distributor import ProviderInfoDistributor

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.agent.rhbp')

class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    def __init__(self, agent_name):
        """

        :param agent_name:
        :type agent_name:  str
        """
        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)

        # Take the name from the constructor parameter in case it was started from a development environment
        if agent_name != None:
            self._agent_name = agent_name
        else:
            self._agent_name = rospy.get_param('~agent_name', "agentA1")  # default for debugging 'agentA1'

        ettilog.loginfo("RhbpAgent(%s):: Starting", self._agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._action_manager = ActionManager(agent_name=self._agent_name)
        self._provider_info_distributor = ProviderInfoDistributor()

        self._sim_started = False
        self._initialized = False

        # subscribe to MAC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False



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
                self._action_manager.init_behaviours(msg=sim_start)

                self.assemble_contractor = AssembleContractor(
                    agent_name=self._agent_name,
                    role=sim_start.role.name)
                self.assemble_contractor = JobContractor(
                    agent_name=self._agent_name)

                ettilog.loginfo("RhbpAgent(%s):: Initialisation finished", self._agent_name)


            self._initialized = True

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        # ettilog.logerr("action response %s", str(msg))
        self._received_action_response = True

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

    def _action_request_callback(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param request_action: the message
        :type request_action: RequestAction
        :return:
        """

        self._provider_info_distributor.callback_request_action(request_action)
        start_time = rospy.get_rostime()
        # ettilog.logerr("-------------------------------------- Step %d (action_request) --------------------------------------", msg.simulation_step)

        self.agent_info = request_action.agent


        self._received_action_response = False

        if hasattr(self, "_action_network_behaviour"):
            DebugUtils.print_precondition_states(self._action_network_behaviour.exploration_network)


        if self._initialized:
            start_time = rospy.get_rostime()
            steps = 0
            # wait for generic action response (send by any behaviour)
            while not self._received_action_response:
                steps += 1
                self._action_manager.step() # selected behaviours eventually trigger action
                # Recharge if decision-making-time > 3.9 seconds
                if (rospy.get_rostime() - start_time).to_sec() > 3.9:
                    ettilog.logerr(
                        "%s: 3.9 seconds (%d steps). Decision-making duration exceeded. Recharging.",
                        self._agent_name, steps)
                    self.fallback_recharge()
                    break

        else:
            ettilog.logerr(
                "%s: Decision-making skipped. Not initialized yet. Recharging.",
                self._agent_name)
            self.fallback_recharge()

        duration = rospy.get_rostime() - start_time
        ettilog.loginfo("%s: Decision-making duration %f", self._agent_name, duration.to_sec())

    def fallback_recharge(self):
        pub_generic_action = rospy.Publisher(
            self._agent_topic_prefix + 'generic_action',
            GenericAction, queue_size=10)
        GenericActionBehaviour.action_generic(publisher=pub_generic_action,
                                              action_type=Action.RECHARGE)


if __name__ == '__main__':

    agent_name = None

    if len(sys.argv) == 3 and sys.argv[1] == "--agent-name":
        agent_name = sys.argv[2]

    try:
        rhbp_agent = RhbpAgent(agent_name)

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
