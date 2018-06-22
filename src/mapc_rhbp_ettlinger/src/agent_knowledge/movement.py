#!/usr/bin/env python2

from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import Movement

from agent_knowledge.base_knowledge import BaseKnowledgebase
from common_utils import rhbp_logging

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.movement')

class MovementKnowledgebase(BaseKnowledgebase):

    INDEX_MOVEMENT_AGENT_NAME = 1
    INDEX_MOVEMENT_identifier = 2
    INDEX_MOVEMENT_LAT = 3
    INDEX_MOVEMENT_LONG = 4
    INDEX_MOVEMENT_DESTINATION = 5

    IDENTIFIER_CHARGING_STATION = "charging_station"
    IDENTIFIER_EXPLORATION = "exploration"

    @staticmethod
    def generate_tuple(identifier="*", agent_name="*", lat="*", long="*", destination="*"):
        return 'moving', agent_name, identifier, str(lat), str(long), str(destination)

    @staticmethod
    def generate_movement_from_fact(fact):
        """
        Generates a Movement object from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: Movement
        """

        movement = Movement(
            identifier = fact[MovementKnowledgebase.INDEX_MOVEMENT_identifier],
            agent_name = fact[MovementKnowledgebase.INDEX_MOVEMENT_AGENT_NAME],
            pos = Position(
                lat=float(fact[MovementKnowledgebase.INDEX_MOVEMENT_LAT]),
                long=float(fact[MovementKnowledgebase.INDEX_MOVEMENT_LONG])
            ),
            destination = fact[MovementKnowledgebase.INDEX_MOVEMENT_DESTINATION]
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
            identifier=movement.identifier,
            lat=movement.pos.lat,
            long=movement.pos.long,
            destination=movement.destination
        )



    def start_movement(self, movement):
        """
        Starts movement to a new destination.
        :param agent_name:
        :param identifier_name:
        :param destinationPos:
        :param destination:
        :return:
        """
        search = MovementKnowledgebase.generate_tuple(agent_name=movement.agent_name, identifier=movement.identifier)
        new = MovementKnowledgebase.generate_fact_from_movement(movement)

        ettilog.loginfo("MovementKnowledge(%s:%s):: Moving to %s ", movement.agent_name, movement.identifier, movement.destination)
        ret_value = self._kb_client.update(search, new, push_without_existing = True)

    def stop_movement(self, agent_name, identifier_name):
        """
        Stops the movement of acertain identifier
        :param agent_name:
        :param identifier_name:
        :return:
        """
        search = MovementKnowledgebase.generate_tuple(agent_name, identifier_name)
        ettilog.loginfo("MovementKnowledge(%s:%s):: Stopping movement", agent_name, identifier_name)

        return self._kb_client.pop(search)

    def get_movement(self, agent_name, identifier):
        """
        Returns the current active movement of a identifier
        :param agent_name:
        :param identifier:
        :return: Movement
        """

        search = MovementKnowledgebase.generate_tuple(
            agent_name=agent_name,
            identifier=identifier)
        fact = self._kb_client.peek(search)

        if fact != None:
            return MovementKnowledgebase.generate_movement_from_fact(fact)
        else:
            return None