#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import abc

import rospy
from diagnostic_msgs.msg import KeyValue
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_ros_bridge.msg import GenericAction

from agent_common.agent_utils import get_bridge_topic_prefix
from agent_knowledge.movement import MovementKnowledge, \
    get_knowledge_base_tuple_facility_exploration
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import action_generic, action_generic_simple, Action


def action_goto_location(lat,long, publisher):
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


def action_continue(publisher):
    """
    Specific "continue" action publishing helper function
    :param publisher: publisher to use
    """
    action_type = "continue"

    action_generic(publisher, action_type)


def action_abort(publisher):
    """
    Specific "abort" action publishing helper function
    :param publisher: publisher to use
    """
    action_type = "abort"

    action_generic(publisher, action_type)

class GotoFacilityBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, facility_topic, **kwargs):

        super(GotoFacilityBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._movement_knowledge = MovementKnowledge(self._agent_name, self._name)

        self._selected_pos = None

        self.__client = KnowledgeBaseClient()

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name,
                                                                                    self._facility_topic)


    def move(self):

        if self._selected_pos: # in case we did not find/know a facility

            rospy.loginfo(self._agent_name + "::" +self._name + " to "+ str(self._selected_pos))
            action_goto_location(lat=self._selected_pos.lat, long=self._selected_pos.long, publisher=self._pub_generic_action)
        else: # backup action recharge agent
            rospy.loginfo(self._agent_name + "::" + self._name + " recharging because of missing facility.")
            action_generic_simple(publisher=self._pub_generic_action,action_type='recharge')

    @abc.abstractmethod
    def _select_pos(self):
        pass


    def start(self):
        rospy.loginfo(self._agent_name + "::" + self._name + " enabled")

        self._selected_pos = self._select_pos()

        rospy.loginfo(self._agent_name + "::" + self._name + " selected facility: " + str(self._selected_pos))

        # we set to false in order to fulfil the requirements of FinishExplorationBehaviour
        self.__client.update(self._exploration_knowledge + ('*',), self._exploration_knowledge + ('false',), )

        if self._selected_pos:
            self._movement_knowledge.start_movement(self._selected_pos)

        self.do_step() # this is important to directly answer the request as in the start() base implementation

    def stop(self):

        rospy.loginfo(self._agent_name + "::" + self._name + " disabled")

        super(GotoFacilityBehaviour, self).stop()

    def do_step(self):

        if not self._selected_pos:
            self._selected_pos = self._select_pos()

            if self._selected_pos:
                self._movement_knowledge.start_movement(self._selected_pos.lat)

        self.move()
