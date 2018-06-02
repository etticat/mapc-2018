#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import abc

import rospy
from diagnostic_msgs.msg import KeyValue
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_ros_bridge.msg import GenericAction, Position

from common_utils.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action
from utils.ros_helpers import get_topic_type



class GotoLocationBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, **kwargs):

        super(GotoLocationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._movement_knowledge = MovementKnowledgebase()

        self._selected_pos = None

        self._selected_destination = "none"

        self.__client = KnowledgeBaseClient()

        self._pub_generic_action = rospy.Publisher(AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    @staticmethod
    def action_goto_location(lat, long, publisher):
        """
        Specific "goto" action publishing helper function
        :param facility_name: name of the facility we want to go to
        :param publisher: publisher to use
        """
        action = GenericAction()
        action.action_type = Action.GO_TO
        action.params = [
            KeyValue("latitude", str(lat)),
            KeyValue("longitude", str(long))]
        publisher.publish(action)
    @staticmethod
    def action_goto_facility(facility, publisher):
        """
        Specific "goto" action publishing helper function
        :param facility_name: name of the facility we want to go to
        :param publisher: publisher to use
        """
        action = GenericAction()
        action.action_type = Action.GO_TO
        action.params = [
            KeyValue("Facility", facility)]
        publisher.publish(action)

    @staticmethod
    def action_continue(publisher):
        """
        Specific "continue" action publishing helper function
        :param publisher: publisher to use
        """
        action_type = "continue"

        GenericActionBehaviour.action_generic(publisher, action_type)

    @staticmethod
    def action_abort(publisher):
        """
        Specific "abort" action publishing helper function
        :param publisher: publisher to use
        """
        action_type = "abort"

        GenericActionBehaviour.action_generic(publisher, action_type)

    def move(self):

        if self._selected_pos: # in case we did not find/know a facility

            rospy.loginfo(self._agent_name + "::" +self._name + " to "+ str(self._selected_pos))
            if isinstance(self._selected_pos, Position):
                GotoLocationBehaviour.action_goto_location(lat=self._selected_pos.lat, long=self._selected_pos.long, publisher=self._pub_generic_action)
            else:
                GotoLocationBehaviour.action_goto_facility(facility=self._selected_pos, publisher=self._pub_generic_action)
        else: # backup action recharge agent
            rospy.loginfo(self._agent_name + "::" + self._name + " recharging because of missing facility.")
            GenericActionBehaviour.action_generic_simple(publisher=self._pub_generic_action,action_type='recharge')

    @abc.abstractmethod
    def _select_pos(self):
        pass


    def start(self):
        rospy.loginfo(self._agent_name + "::" + self._name + " enabled")

        self._selected_pos = self._select_pos()

        rospy.loginfo(self._agent_name + "::" + self._name + " selected facility: " + str(self._selected_pos))

        # we set to false in order to fulfil the requirements of FinishExplorationBehaviour

        if self._selected_pos:
            self._movement_knowledge.start_movement(self._agent_name, self._name, self._selected_pos, self._selected_destination)

        super(GotoLocationBehaviour, self).start()

    def stop(self):

        rospy.loginfo(self._agent_name + "::" + self._name + " disabled")

        super(GotoLocationBehaviour, self).stop()

    def do_step(self):

        if not self._selected_pos:
            self._selected_pos = self._select_pos()

            if self._selected_pos:
                self._movement_knowledge.start_movement(self._agent_name, self._name, self._selected_pos, self._selected_destination)

        self.move()

class GoToFacilityBehaviour(GotoLocationBehaviour):

    def __init__(self, agent, plannerPrefix, topic,  **kwargs):
        self.agent = agent
        super(GoToFacilityBehaviour, self).__init__(
            plannerPrefix=plannerPrefix,
            **kwargs)

        facility_topic_type = get_topic_type(topic)

        self._facilities = {}
        rospy.Subscriber(topic , facility_topic_type, self._callback_facility)


    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility

    def _select_pos(self):
        """
        Determine the facility we want to go to, using euclidean distance
        TODO: Use graphhopper for non drone vehicles
        :return: facility
        """
        closest_facility = None
        min_distance = 9999

        for _, facility in self._facilities.items():
            # TODO: get the agent position from topic instead of passing through the instance
            distance = AgentUtils.euclidean_distance(self.agent.agent_info.pos, facility.pos)
            if distance < min_distance:
                min_distance = distance
                closest_facility = facility

        return closest_facility.pos

    def stop(self):
        super(GoToFacilityBehaviour, self).stop()