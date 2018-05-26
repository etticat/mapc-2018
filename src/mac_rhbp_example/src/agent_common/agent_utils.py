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
    def euclidean_distance(pos1, pos2):
        """
        Calculate the euclidean distance between two positions
        :param pos1: position 1
        :type pos1: Position
        :param pos2: position 2
        :type pos2: Position
        :return: euclidean distance
        """
        return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)
