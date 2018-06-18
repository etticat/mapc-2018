#!/usr/bin/env python2
import copy
import itertools
import operator

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg, JobAssignment

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import JobKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton
from math import sin, cos, sqrt, atan2, radians



class StepProvider(object):
    __metaclass__ = Singleton

    RADIUS_EARTH_METERS = 6373000.0 # Using same approximation as server

    def __init__(self, agent_name):


        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        self.cell_size = 1.0
        self.can_fly = ""

    def _sim_start_callback(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self.cell_size = sim_start.cell_size
        self.can_fly = "air" in sim_start.role.roads
        self.speed = sim_start.role.base_speed # TODO: Update on upgrade

    def calculate_steps(self, startPosition, endPosition):
        if self.can_fly:
            return self.calculate_steps_air(startPosition, endPosition)
        else:
            # TODO: Either implement with graphhopper or approximate with a constant
            return self.calculate_steps_air(startPosition, endPosition) * 1.6

    def calculate_steps_air(self, pos1, pos2):
        size_ = self.calculate_distance_air(pos1, pos2) / (self.speed * self.cell_size)
        return size_ / 200 # TODO: I dont know why yet but deviding by 200 gives the correct result


    def calculate_distance_air(self, pos1, pos2):
        """
        Logic extracted from massim server massim.protocol.scenario.city.util.LocationUtil.java
        :param pos1:
        :param pos2:
        :return:
        """

        lat1 = radians(pos1.lat)
        lon1 = radians(pos1.long)
        lat2 = radians(pos2.lat)
        lon2 = radians(pos2.long)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return StepProvider.RADIUS_EARTH_METERS * c