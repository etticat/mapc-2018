import random

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.well_chooser')


class ChooseWellToBuildDecision(object):
    """
    Choose a well type to build.
    TODO: This should be completely redone
    """

    def __init__(self, agent_name):
        self.task_prices = {}
        self.stats_provider = StatsProvider(agent_name=agent_name)
        self.well_provider = WellProvider(agent_name=agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")
        self._simulation_provider = SimulationProvider(agent_name=agent_name)

    def choose_well_type(self):

        res_type = None
        res_cost = 99999
        # TODO: Choose it more elegantly
        massium = self.stats_provider.get_massium()
        massium -= self.get_massium_for_current_well_tasks()
        for type, well in self.well_provider.possible_wells.iteritems():
            if well.cost < massium and well.cost < res_cost:
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

    def get_massium_for_current_well_tasks(self):
        return sum(self.task_prices.values())

    def start_task(self, _id, well_type):
        self.task_prices[id] = self.well_provider.get_well_price(well_type)

    def end_task(self, well_task_id):
        self.task_prices.pop(well_task_id, None)
