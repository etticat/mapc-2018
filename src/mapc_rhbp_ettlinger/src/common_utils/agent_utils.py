from __future__ import division

import math

from mac_ros_bridge.msg import Position

class AgentUtils:

    @staticmethod
    def get_bridge_topic_prefix(agent_name):
        """
        Determine the topic prefix for all topics of the bridge node corresponding to the agent
        :param agent_name: current agents name
        :return: prefix just before the topic name of the bridge
        """
        return '/bridge_node_' + agent_name + '/'

    @staticmethod
    def get_bridge_topic_agent(agent_name):
        """
        Determine the topic prefix for all topics of the bridge node corresponding to the agent
        :param agent_name: current agents name
        :return: prefix just before the topic name of the bridge
        """
        return AgentUtils.get_bridge_topic_prefix(agent_name)+ 'agent'

    @staticmethod
    def calculate_distance(pos1, pos2):
        """
        TODO: Calculate distance on rode not only euclidean
        Calculate the euclidean distance between two positions
        :param pos1: position 1
        :type pos1: Position
        :param pos2: position 2
        :type pos2: Position
        :return: euclidean distance
        """
        return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)

    @staticmethod
    def calculate_closest_facility(pos, facilities):
        minDistance = 999999
        closest_facility = None
        for facility in facilities: 
            distance = AgentUtils.calculate_distance(pos, facility.pos)
            if distance < minDistance:
                minDistance = distance
                closest_facility = facility
        return closest_facility

    @classmethod
    def get_internal_prefix(cls, agent_name):
        return '/internal_' + agent_name + '/'

    @classmethod
    def get_assemble_prefix(cls):
        return '/assemble/'

    @classmethod
    def get_job_prefix(cls):
        return '/job/'
        
        