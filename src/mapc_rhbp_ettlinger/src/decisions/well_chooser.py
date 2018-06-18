import random

import rospy
from mac_ros_bridge.msg import SimStart, Position

from common_utils.agent_utils import AgentUtils
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider


class ChooseWellToBuild(object):

    def __init__(self):
        self.stats_provider = StatsProvider()
        self.well_provider = WellProvider()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        self.min_lat = 0
        self.max_lat = 0
        self.min_long = 0
        self.max_long = 0

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

    def _sim_start_callback(self, simStart):
        """

        :param simStart:
        :type simStart: SimStart
        :return:
        """
        self.min_lat = simStart.min_lat
        self.max_lat = simStart.max_lat
        self.min_long = simStart.min_lon
        self.max_long = simStart.max_lon
        pass

    def choose_well_type(self):

        res_type = None
        res_cost = 99999
        # TODO: Choose it more elegantly
        for type, well in self.well_provider.get_wells_to_build().iteritems():
            if well.cost < self.stats_provider.get_goal_massium() and well.cost < res_cost:
                res_type = type
                res_cost = well.cost

        return res_type

    def choose_well_position(self):

        # TODO: Choose it more elegantly
        return Position(
            lat=random.uniform(self.min_lat, self.max_lat),
            long=random.uniform(self.min_long, self.max_long))

    def choose_agent_for_building(self, bids):

        if len(bids) == 0:
            return []
        # TODO: Select it more elegantly
        # Could also be done by multiple (to build up faster)
        return [random.choice(bids)]