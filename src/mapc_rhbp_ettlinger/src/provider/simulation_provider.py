import random

import rospy
from mac_ros_bridge.msg import Position, RequestAction, SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.simulation')


class SimulationProvider(object):
    """
    Provider, that provides some basic infos about the simulation
    """
    __metaclass__ = Singleton

    COORDINATES_MARGIN = 0.001

    def __init__(self, agent_name):
        self._min_lat = 0
        self._max_lat = 0
        self._min_long = 0
        self._max_long = 0
        self._min_lat_outer = 0
        self._max_lat_outer = 0
        self._min_long_outer = 0
        self._max_long_outer = 0
        self._step = 0
        self._team = None
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart,
                         self.callback_sim_start)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="request_action"), RequestAction,
                         self.callback_request_action)

    def callback_sim_start(self, sim_start):
        """
        Keep track of some simulation values
        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self._min_lat = sim_start.min_lat + SimulationProvider.COORDINATES_MARGIN
        self._max_lat = sim_start.max_lat - SimulationProvider.COORDINATES_MARGIN
        self._min_long = sim_start.min_lon + SimulationProvider.COORDINATES_MARGIN
        self._max_long = sim_start.max_lon - SimulationProvider.COORDINATES_MARGIN

        self._min_lat_outer = sim_start.min_lat
        self._max_lat_outer = sim_start.max_lat
        self._min_long_outer = sim_start.min_lon
        self._max_long_outer = sim_start.max_lon

        self._team = sim_start.team
        pass

    def callback_request_action(self, request_action):
        """

        :param request_action:
        :type request_action: RequestAction
        :return:
        """
        self._step = request_action.simulation_step

    def out_of_bounds(self, pos):
        return pos.lat < self._min_lat_outer or pos.lat > self._max_lat_outer \
               or pos.long < self._min_long_outer or pos.long > self._max_long_outer

    def get_random_position(self):
        """
        Returns a random position on the simulation
        :return:
        """
        return Position(
            lat=random.uniform(self._min_lat, self._max_lat),
            long=random.uniform(self._min_long, self._max_long))

    @property
    def step(self):
        return self._step

    @property
    def team(self):
        return self._team

    def get_corners(self):
        return [Position(lat=self._min_lat, long=self._min_long),
                Position(lat=self._min_lat, long=self._max_long),
                Position(lat=self._max_lat, long=self._min_long),
                Position(lat=self._max_lat, long=self._max_long), ]
