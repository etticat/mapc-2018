#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, Position
from mapc_rhbp_ettlinger.msg import Task

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

    def __init__(self, agent_name, mechanism, recalculate_destination_every_step=False, use_name_for_movement=False,
                 **kwargs):
        super(GoToDestinationBehaviour, self).__init__(mechanism=mechanism, requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name
        self._destination_decision = mechanism
        self._recalculate_destination_every_step = recalculate_destination_every_step

        self._use_name_for_movement = use_name_for_movement

        self._destination = None

        # initialise providers
        self._action_provider = ActionProvider(agent_name=agent_name)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """

        if self._destination is not None:
            # Sometimes movement doesnt work (e.g. when navigating into a building.)
            # We save this destination into the self organisation component, so we don't try it again later
            if agent.last_action == "goto" and agent.last_action_result == "failed_no_route":
                # Reset destination, so we can pick a new one in the next round
                ettilog.logerr("GoToDestinationBehaviour(%s):: Could not go to destination (%s), picking new one ...", self.name, str(self._destination))
                self.mechanism.destination_not_found(self._destination)
                self._destination = None

    def start(self):
        """
        Reset the destination at every start, to avoid using destinations from previous tries
        :return:
        """
        self._destination = None
        super(GoToDestinationBehaviour, self).start()

    def do_step(self):
        """
        Retrieves the destination and tries to move there
        :return:
        """
        if self._destination is None or self._recalculate_destination_every_step:
            self._destination = super(GoToDestinationBehaviour, self).do_step()

            if self._destination is not None:
                ettilog.loginfo("GoToDestinationBehaviour(%s):: Picked new destination (%s)", self.name, str(self._destination))

        if self._destination is not None:
            # Some behaviours do not provide positions but rather other objects, which have a pos attribute
            # This allows to reuse the mechanism for other purposes. In this case use the attribute
            if self._use_name_for_movement:
                if isinstance(self._destination, Task):
                    name = self._destination.destination_name
                else:
                    name = self._destination.name
                self._action_provider.action_go_to_facility(name)
            else:
                self._action_provider.action_go_to_destination(self._destination)
        else:
            ettilog.logerr("GoToDestinationBehaviour(%s):: Could not decide for destination", self.name)

    def stop(self):
        """
        Reset the destination on stop of behaviour
        :return:
        """
        self._destination = None
        super(GoToDestinationBehaviour, self).stop()
