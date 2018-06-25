#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import Team, RequestAction

from agent_knowledge.task import TaskKnowledgeBase
from common_utils import etti_logging
from common_utils.singleton import Singleton
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.stats')


class StatsProvider(object):
    __metaclass__ = Singleton

    STATS_TOPIC = "/team"

    def __init__(self):
        self._task_knowledgebase = TaskKnowledgeBase()
        self.well_provider = WellProvider()
        rospy.Subscriber(StatsProvider.STATS_TOPIC, Team, self._callback_team)
        rospy.Subscriber(StatsProvider.STATS_TOPIC, Team, self._callback_team)

        self.simulation_step = 0
        self.massium = 0
        self.score = 0

    def callback_request_action(self, request_action):
        """

        :param request_action:
        :type request_action: RequestAction
        :return:
        """
        self.simulation_step = request_action.simulation_step

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
        for well in self._task_knowledgebase.get_tasks(
                type=TaskKnowledgeBase.TYPE_BUILD_WELL,
                task="build_up"):
            delayed_cost += self.well_provider.get_well(well.well_type).cost

        future_massimum = self.massium - delayed_cost

        return future_massimum
