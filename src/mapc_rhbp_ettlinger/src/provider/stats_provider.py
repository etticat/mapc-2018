#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import Team

from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.stats')


class StatsProvider(object):
    """
    Provider that keeps track of stats of the simulation
    """
    __metaclass__ = Singleton

    STATS_TOPIC = "/team"

    def __init__(self):
        self.massium = 0
        self.score = 0

        rospy.Subscriber(StatsProvider.STATS_TOPIC, Team, self._callback_team)

    def _callback_team(self, team):
        """
        Save massium and score at current step
        :param team:
        :type team: Team
        :return:
        """

        self.massium = team.massium
        self.score = team.score

    def get_current_massium(self):
        """
        Returns the current amount of massium
        :return:
        """
        return self.massium
