#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import abc

import rospy
from diagnostic_msgs.msg import KeyValue
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_ros_bridge.msg import GenericAction, Position

from agent_common.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledge
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action



class GotoLocationBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, **kwargs):

        super(GotoLocationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._movement_knowledge = MovementKnowledge(self._agent_name, self._name)

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
            self._movement_knowledge.start_movement(self._selected_pos, self._selected_destination)

        super(GotoLocationBehaviour, self).start()

    def stop(self):

        rospy.loginfo(self._agent_name + "::" + self._name + " disabled")

        super(GotoLocationBehaviour, self).stop()

    def do_step(self):

        if not self._selected_pos:
            self._selected_pos = self._select_pos()

            if self._selected_pos:
                self._movement_knowledge.start_movement(self._selected_pos, self._selected_destination)

        self.move()