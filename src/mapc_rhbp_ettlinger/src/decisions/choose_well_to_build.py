import numpy as np
import time

import rospy
from mapc_rhbp_ettlinger.msg import Task

from common_utils import etti_logging
from decisions.current_task import CurrentTaskDecision
from provider.agent_info_provider import AgentInfoProvider
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.well_chooser')


class ChooseWellToBuildDecision(object):
    """
    Choose a well type to build.
    """

    BUILD_WELL_OUTSIDE_PLAY_AREA = True

    def __init__(self, agent_name):

        self._agent_name = agent_name

        self.task_prices = {}
        
        self._init_config()

        self._stats_provider = StatsProvider()
        self._well_provider = WellProvider(agent_name=agent_name)
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)


    @staticmethod
    def _init_config(sim_start=None):

        ChooseWellToBuildDecision.BUILD_WELL_OUTSIDE_PLAY_AREA = rospy.get_param(
            "ChooseWellToBuildDecision.BUILD_WELL_OUTSIDE_PLAY_AREA", ChooseWellToBuildDecision.BUILD_WELL_OUTSIDE_PLAY_AREA)

    def choose(self, current_well_task_decision, agent):
        """
        Returns a WellTask if a well has to be built, None otherwise.
        :param agent:
        :param current_well_task_decision:
        :type current_well_task_decision: CurrentTaskDecision
        :return:
        """
        # If we are currently out of bounds we build the best available well at the current position
        out_of_bounds = self._simulation_provider.out_of_bounds(agent.pos)
        if (out_of_bounds != ChooseWellToBuildDecision.BUILD_WELL_OUTSIDE_PLAY_AREA) \
                and current_well_task_decision.value is None and not agent.in_facility:
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
        """
        Picks the best well_type to build. Always returns the one with the best cost/efficiency ratio, that possible
        to build with current massium.
        :return:
        """
        res_type = None
        res_activation = -np.inf
        massium = self._stats_provider.get_current_massium()
        massium -= self.get_massium_for_current_well_tasks()
        for well_type, well in self._well_provider.possible_wells.iteritems():
            activation = well.efficiency / well.cost
            if well.cost < massium and activation > res_activation:
                res_type = well_type
                res_activation = activation

        return res_type

    def get_massium_for_current_well_tasks(self):
        """
        Returns massium for all current well tasks
        :return:
        """
        return sum(self.task_prices.values())

    def start_task(self, _id, well_type):
        """
        When starting a task, reserve the massium so no one else builds faster
        :param _id:
        :param well_type:
        :return:
        """
        self.task_prices[id] = self._well_provider.get_well_price(well_type)

    def end_task(self, well_task_id):
        """
        When finishing the task, we can just remove the build goal again
        :param well_task_id:
        :return:
        """
        self.task_prices.pop(well_task_id, None)
