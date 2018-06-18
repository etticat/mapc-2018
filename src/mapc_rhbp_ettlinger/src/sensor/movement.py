#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Position, Agent

from common_utils.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledgebase
from provider.step_provider import StepProvider
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor


class DestinationDistanceSensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, behaviour_name, name):

        pattern = MovementKnowledgebase.generate_tuple(agent_name=agent_name, behaviour=behaviour_name)

        super(DestinationDistanceSensor, self).__init__(pattern=pattern, name=name, initial_value=999.0)

        self._last_pos = None

        self._movement_knowledge = MovementKnowledgebase()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self.subscription_callback_ref_topic)

    def subscription_callback_ref_topic(self, msg):
        if self._last_pos != msg.pos:
            self._last_pos = msg.pos
            self._cache_update_callback() # Force update when position of agent changes

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        # If we don't have a destination we handle it as if we are far away
        res = 1.0

        if len(facts) > 0 and self._last_pos != None:
            movement = MovementKnowledgebase.generate_movement_from_fact(facts.pop())  # only getting the first fact

            try:

                destination_pos = movement.pos
                agent_position = self._last_pos

                # if self.name == "resource_destination_sensor_hoarding":
                #     rospy.logerr("----------------- %s %s", str(destination_pos), str(agent_position))

                res = AgentUtils.calculate_distance(destination_pos, agent_position)

            except Exception:
                rospy.logerr("Couldn't get last tuple element of: %s. Resetting to initial_value", str(movement))

            # if self.name == "at_charging_station":
            #     rospy.logerr("////// %s", str(movement))
        # if self.name == "at_charging_station":
        #     rospy.logerr("---%s", str(res))
        return res