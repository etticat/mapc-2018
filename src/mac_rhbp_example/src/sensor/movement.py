#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Position, Agent

from agent_common.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledge
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor


class DestinationDistanceSensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, behaviour_name, name):

        pattern = MovementKnowledge.get_movement_tuple(agent_name=agent_name, behaviour=behaviour_name, active=True)

        super(DestinationDistanceSensor, self).__init__(pattern=pattern, name=name)

        self._latest_ref_value = None

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self.subscription_callback_ref_topic)

    def subscription_callback_ref_topic(self, msg):
        self._latest_ref_value = msg

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        # If we don't have a destination we handle it as if we are far away
        res = 1

        if len(facts) > 0 and self._latest_ref_value != None:
            fact_tuple = facts.pop()  # only getting the first fact

            try:

                destination_pos = Position(lat=float(fact_tuple[4]), long=float(fact_tuple[5]))
                agent_position = self._latest_ref_value.pos

                res = AgentUtils.euclidean_distance(destination_pos, agent_position)

            except Exception:
                rospy.loginfo("Couldn't get last tuple element of: %s. Resetting to initial_value", str(fact_tuple))

        return res