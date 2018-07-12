#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, Position

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from rhbp_selforga.behaviours import DecisionBehaviour

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.movement')


class GoToDestinationBehaviour(DecisionBehaviour):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, mechanism, recalculate_destination_every_step=False,
                 **kwargs):
        super(GoToDestinationBehaviour, self).__init__(mechanism=mechanism, requires_execution_steps=True, **kwargs)

        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)
        self._agent_name = agent_name
        self.destination_decision = mechanism
        self.destination = None
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.recalculate_destination_every_step = recalculate_destination_every_step

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # Sometimes movement doesnt work (e.g. when navigating into a building.)
        if agent.last_action == "goto" and agent.last_action_result == "failed_no_route":
            # Avoid this place until step 100000
            if self.destination is not None:
                self.self_organisation_provider.send_msg(pos=self.destination, frame="no_route", parent_frame="agent", time=100000)
                self.destination = None
                ettilog.logerr("GoToTaskDestinationBehaviour(%s):: Could not go to destination, picking new one ...", self.name)
            else:
                ettilog.logerr("GoToTaskDestinationBehaviour(%s):: Destination not available???", self.name)

    def start(self):
        self.destination = None
        super(GoToDestinationBehaviour, self).start()

    def do_step(self):
        if self.destination is None or self.recalculate_destination_every_step:
            self.destination = super(GoToDestinationBehaviour, self).do_step()
            if not isinstance(self.destination, Position) and self.destination is not None:
                self.destination = self.destination.pos


        if self.destination is not None:
            self.action_provider.action_go_to_destination(self.destination)
        else:
            ettilog.logerr("GoToTaskDestinationBehaviour(%s):: Could not decide for destination", self.name)