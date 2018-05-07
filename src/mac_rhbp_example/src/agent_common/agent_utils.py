from __future__ import division # force floating point division when using plain /

import math

from mac_ros_bridge.msg import Position


def get_bridge_topic_prefix(agent_name):
    """
    Determine the topic prefix for all topics of the bridge node corresponding to the agent
    :param agent_name: current agents name
    :return: prefix just before the topic name of the bridge
    """
    return '/bridge_node_' + agent_name + '/'


def get_knowledge_base_tuple_facility_exploration(agent_name, facility):
    """
    Simple function to create uniform knowledge tuples for facility exploration
    :param agent_name: name of the considered agent
    :param facility: facility name or topic that is explored
    :return: generate tuple (agent_name, exploration_key)
    """
    return agent_name, 'exploring_' + facility


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
