import random

from mac_ros_bridge.msg import Position, RequestAction, SimStart

from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.simulation')


class SimulationProvider(object):
    __metaclass__ = Singleton

    def __init__(self):
        self.min_lat = 0
        self.max_lat = 0
        self.min_long = 0
        self.max_long = 0
        self.step = 0

    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self.min_lat = sim_start.min_lat + 0.001 # TODO: The agents always get stuck as the min lat is not reachable
        self.max_lat = sim_start.max_lat - 0.001 # TODO: Find a better way
        self.min_long = sim_start.min_lon + 0.001
        self.max_long = sim_start.max_lon - 0.001
        self.team = sim_start.team
        pass

    def callback_request_action(self, request_action):
        """

        :param request_action:
        :type request_action: RequestAction
        :return:
        """
        self.step = request_action.simulation_step

    def get_random_position(self):
        return Position(
            lat=random.uniform(self.min_lat, self.max_lat),
            long=random.uniform(self.min_long, self.max_long))

    def team_identifier(self):
        return self.team
