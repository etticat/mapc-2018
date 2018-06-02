#!/usr/bin/env python2

import rospy
from mapc_rhbp_ettlinger.msg import Movement
from mac_ros_bridge.msg import Position

from agent_knowledge.base_knowledge import BaseKnowledgebase


class MovementKnowledgebase(BaseKnowledgebase):

    INDEX_MOVEMENT_BEHAVIOUR = 1
    INDEX_MOVEMENT_AGENT_NAME = 2
    INDEX_MOVEMENT_ACTIVE = 3
    INDEX_MOVEMENT_LAT = 4
    INDEX_MOVEMENT_LONG = 5
    INDEX_MOVEMENT_DESTINATION = 6

    @staticmethod
    def generate_tuple(agent_name, behaviour="*", active="*", lat="*", long="*", destination="*"):
        return ('moving', behaviour, agent_name, str(active), str(lat), str(long), str(destination))

    @staticmethod
    def generate_movement_from_fact(fact):
        """
        Generates a Movement object from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: Movement
        """

        movement = Movement(
            behaviour = fact[MovementKnowledgebase.INDEX_MOVEMENT_BEHAVIOUR],
            agent_name = fact[MovementKnowledgebase.INDEX_MOVEMENT_AGENT_NAME],
            pos = Position(
                lat=fact[MovementKnowledgebase.INDEX_MOVEMENT_LAT],
                long=fact[MovementKnowledgebase.INDEX_MOVEMENT_LONG]
            ),
            destination = fact[MovementKnowledgebase.INDEX_MOVEMENT_DESTINATION],
            active = bool(fact[MovementKnowledgebase.INDEX_MOVEMENT_ACTIVE]),
        )
        return movement
    @staticmethod
    def generate_fact_from_movement(movement):
        """
        Generates a Knowledgebase fact from a Movement object
        :param movement: The movement object
        :type movement: Movement
        :return:
        """

        return MovementKnowledgebase.generate_tuple(
            agent_name=movement.agent_name,
            behaviour=movement.behaviour,
            active=movement.active,
            lat=movement.pos.lat,
            long=movement.pos.long,
            destination=movement.destination
        )



    def start_movement(self, agent_name, behaviour_name, destinationPos, destination):
        """
        Starts movement to a new destination.
        :param agent_name:
        :param behaviour_name:
        :param destinationPos:
        :param destination:
        :return:
        """
        search = MovementKnowledgebase.generate_tuple(agent_name, behaviour_name)
        new = MovementKnowledgebase.generate_tuple(agent_name, behaviour_name, active=True, lat=destinationPos.lat, long=destinationPos.long, destination=destination)

        rospy.logerr("MovementKnowledge(%s:%s):: Moving to %s ", agent_name, behaviour_name, destination)
        ret_value = self._kb_client.update(search, new, push_without_existing = True)

    def stop_movement(self, agent_name, behaviour_name):
        """
        Stops the movement of acertain behaviour
        :param agent_name:
        :param behaviour_name:
        :return:
        """
        search = MovementKnowledgebase.generate_tuple(agent_name, behaviour_name)
        new = MovementKnowledgebase.generate_tuple(agent_name, behaviour_name, active=False, lat="none", long="none", destination="none")

        rospy.logerr("MovementKnowledge(%s:%s):: Stopping movement", agent_name, behaviour_name)

        ret_value = self._kb_client.update(search, new, push_without_existing = False)
        return ret_value

    def get_movement(self, agent_name, behaviour_name):
        """
        Returns the current active movement of a behaviour
        :param agent_name:
        :param behaviour_name:
        :return: Movement
        """

        search = MovementKnowledgebase.generate_tuple(
            agent_name=agent_name,
            behaviour=behaviour_name)
        fact = self._kb_client.peek(search)

        if fact != None:
            return MovementKnowledgebase.generate_movement_from_fact(fact)
        else:
            return None