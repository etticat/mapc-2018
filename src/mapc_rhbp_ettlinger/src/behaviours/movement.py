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
    Behaviour, that moves towards a destination, provided by a mechanism
    """

    def __init__(self, agent_name, mechanism, recalculate_destination_every_step=False,
                 **kwargs):
        super(GoToDestinationBehaviour, self).__init__(mechanism=mechanism, requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name
        self.destination_decision = mechanism
        self.recalculate_destination_every_step = recalculate_destination_every_step

        self.destination = None

        # initialise providers
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)
        self._action_provider = ActionProvider(agent_name=agent_name)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """

        if self.destination is not None:
            # Sometimes movement doesnt work (e.g. when navigating into a building.)
            # We save this destination into the self organisation component, so we don't try it again later
            # TODO: build wells there using drones
            if agent.last_action == "goto" and agent.last_action_result == "failed_no_route":
                # Avoid this place until step 100000
                self._self_organisation_provider.send_msg(pos=self.destination, frame="no_route", parent_frame="agent", time=100000, payload=[
                    KeyValue(key="lat", value=str(self.destination.lat)), KeyValue(key="long", value=str(self.destination.long))], diffusion=0.01)

                # Reset destination, so we can pick a new one in the next round
                self.destination = None
                ettilog.logerr("GoToTaskDestinationBehaviour(%s):: Could not go to destination (%.3f, %3f), picking new one ...", self.name, self.destination.lat, self.destination.long)

    def start(self):
        """
        Reset the destination at every start, to avoid using destinations from previous tries
        :return:
        """
        self.destination = None
        super(GoToDestinationBehaviour, self).start()

    def do_step(self):
        """
        Retrieves the destination and tries to move there
        :return:
        """
        if self.destination is None or self.recalculate_destination_every_step:
            self.destination = super(GoToDestinationBehaviour, self).do_step()

            # Some behaviours do not provide positions but rather other objects, which have a pos attribute
            # This allows to reuse the mechanism for other purposes. In this case use the attribute
            if not isinstance(self.destination, Position) and self.destination is not None:
                self.destination = self.destination.pos
            ettilog.loginfo("GoToTaskDestinationBehaviour(%s):: Picked new destination (%.3f, %3f)", self.name, self.destination.lat, self.destination.long)

        if self.destination is not None:
            self._action_provider.action_go_to_destination(self.destination)
        else:
            ettilog.logerr("GoToTaskDestinationBehaviour(%s):: Could not decide for destination", self.name)

    def stop(self):
        """
        Reset the destination on stop of behaviour
        :return:
        """
        self.destination = None
        super(GoToDestinationBehaviour, self).stop()
