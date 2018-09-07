import random
import time

import numpy as np
import rospy
from mapc_rhbp_ettlinger.msg import Task

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.current_task import CurrentTaskDecision
from provider.agent_info_provider import AgentInfoProvider
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

        self._agent_name = agent_name

        self.task_prices = {}

        self.stats_provider = StatsProvider(agent_name=agent_name)
        self.well_provider = WellProvider(agent_name=agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)

    def choose(self, current_well_task_mechanism, agent):
        """

        :param current_well_task_mechanism:
        :type current_well_task_mechanism: CurrentTaskDecision
        :return:
        """
        # If we are currently out of bounds we build the best available well at the current position
        if self._simulation_provider.out_of_bounds(agent.pos) and current_well_task_mechanism.value is None and not agent.in_facility:
            well_type = self.choose_well_type()
            if well_type is not None:
                task = Task(
                    id=int(time.time() / 17),
                    type=CurrentTaskDecision.TYPE_BUILD_WELL,
                    agent_name=self._agent_name,
                    items=[],
                    pos=agent.pos,
                    destination_name="none",
                    task=well_type
                )
                return task
        return None

    def choose_well_type(self):

        res_type = None
        res_activation = -np.inf
        massium = self.stats_provider.get_current_massium()
        massium -= self.get_massium_for_current_well_tasks()
        for type, well in self.well_provider.possible_wells.iteritems():
            activation = well.efficiency / well.cost
            if well.cost < massium and activation > res_activation:
                res_type = type
                res_activation = activation

        return res_type

    def get_massium_for_current_well_tasks(self):
        return sum(self.task_prices.values())

    def start_task(self, _id, well_type):
        self.task_prices[id] = self.well_provider.get_well_price(well_type)

    def end_task(self, well_task_id):
        self.task_prices.pop(well_task_id, None)