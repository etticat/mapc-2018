import numpy as np
import random
import traceback

import rospy
from common_utils import etti_logging
from decisions.map_decisions import MapDecision

from provider.distance_provider import DistanceProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.discovery_progress')


class DiscoverProgressDecision(MapDecision):
    """
    Calculates the percentage of the map tha has been already visited
    """
    def __init__(self, buffer, agent_name):
        super(DiscoverProgressDecision, self).__init__(buffer=buffer, frame=None, key='destination',
                                                       target_frames=["agent", "exploration_goal", "no_route"],
                                                       mode=MapDecision.MODE_OLDEST_VISITED, agent_name=agent_name)

    def calc_value(self):
        res = super(DiscoverProgressDecision, self).calc_value()

        if res is None:
            return [0.0, self.state]

        environment_array = res[0]

        progress = float(np.count_nonzero(environment_array)) / environment_array.size

        return [progress, self.state]


class OldestCellAgeDecision(MapDecision):
    """
    Calculates the max steps that have passed since all cells have been seen last
    """
    def __init__(self, buffer, init_value, agent_name):
        super(OldestCellAgeDecision, self).__init__(buffer=buffer, frame='noneTODOremove', key='destination',
                                                    target_frames=["agent", "exploration_goal", "no_route"],
                                                    mode=MapDecision.MODE_OLDEST_VISITED, agent_name=agent_name)
        self.init_value = init_value

    def calc_value(self):

        res = super(OldestCellAgeDecision, self).calc_value()

        if res is None:
            return [0.0, self.state]

        environment_array = res[0]

        min_val = np.min(environment_array)

        if min_val == 0:
            return [self.init_value, self.state]
        else:
            return min_val
