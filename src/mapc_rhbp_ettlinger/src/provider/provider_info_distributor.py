from agent_knowledge.resource import ResourceBaseKnowledgeBase
from common_utils import etti_logging
from provider.distance_provider import DistanceProvider
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.distributor')


class ProviderInfoDistributor(object):

    def __init__(self):
        self.step_provider = DistanceProvider()
        self.well_provider = WellProvider()
        self.simulation_provider = SimulationProvider()
        self.stats_provider = StatsProvider()
        self._resource_knowledge_base = ResourceBaseKnowledgeBase()

    def callback_sim_start(self, sim_start):
        self.step_provider.callback_sim_start(sim_start=sim_start)
        self.well_provider.callback_sim_start(sim_start=sim_start)
        self.simulation_provider.callback_sim_start(sim_start=sim_start)

    def callback_request_action(self, request_action):
        self.stats_provider.callback_request_action(request_action=request_action)

        for resource in request_action.resources:
            self._resource_knowledge_base.add_new_resource(resource)
