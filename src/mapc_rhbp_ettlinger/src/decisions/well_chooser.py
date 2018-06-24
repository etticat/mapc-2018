import random

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.well_chooser')

class ChooseWellToBuild(object):

    def __init__(self):
        self.stats_provider = StatsProvider()
        self.well_provider = WellProvider()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")
        self._simulation_provider = SimulationProvider()

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
        return self._simulation_provider.get_random_position()

    def choose_agent_for_building(self, bids):

        if len(bids) == 0:
            return []
        # TODO: Select it more elegantly
        # Could also be done by multiple (to build up faster)
        return [random.choice(bids)]