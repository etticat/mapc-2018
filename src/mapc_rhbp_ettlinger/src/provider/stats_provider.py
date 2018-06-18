#!/usr/bin/env python2
import copy
import itertools
import operator

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item, WellMsg, Well, Team
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg, JobAssignment

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import JobKnowledgebase
from agent_knowledge.well import WellTaskKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton
from provider.well_provider import WellProvider


class StatsProvider(object):
    __metaclass__ = Singleton

    STATS_TOPIC = "/team"

    def __init__(self):

        self.well_task_knowledgebase = WellTaskKnowledgebase()
        self.well_provider = WellProvider()
        rospy.Subscriber(StatsProvider.STATS_TOPIC, Team, self._callback_team)

        self.massium = 0
        self.score = 0

    def _callback_team(self, team):
        """

        :param team:
        :type team: Team
        :return:
        """

        self.massium = team.massium
        self.score = team.score

    def get_goal_massium(self):

        delayed_cost = 0
        for well in self.well_task_knowledgebase.get_tasks(WellTaskKnowledgebase.generate_tuple(built="False")):
            delayed_cost += self.well_provider.get_well(well.well_type).cost

        future_massimum = self.massium - delayed_cost

        return future_massimum