import numpy as np
import time

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import Task

from common_utils import etti_logging
from decisions.current_task import CurrentTaskDecision
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
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
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

        self._drone_positions = []

        # Use these positions if graphhoppper fails to load on axeltower
        # These are just random positions retreived from graphhopper, that road agents cant access
        self._add_unreachable_square(55.602586, 12.548, 55.600226, 12.529)
        self._add_unreachable_square(52.487, 13.49876, 52.479, 13.4999)
        self._add_unreachable_square(-23.64888, -46.72131, -23.64574, -46.72868)

    def _add_unreachable_square(self, max_lat, max_lon, min_lat, min_lon):
        for lat in self.linspace(min_lat, max_lat):
            for lon in self.linspace(min_lon, max_lon):
                self._drone_positions.append(Position(lat=lat, long=lon))

    def linspace(self, a, b):
        return np.linspace(a, b, num=int(abs((b - a) / 0.00022)))

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
        if self._distance_provider.can_fly and current_well_task_decision.value is None and not agent.in_facility:
            well_type = self.choose_well_type()

            current_drone_positions = list(filter(lambda x: not self._simulation_provider.out_of_bounds(x), self._drone_positions))

            well_index = self._simulation_provider.step % len(current_drone_positions)

            next_well_position = current_drone_positions[well_index]

            if well_type is not None:
                task = Task(
                    id=int(time.time() / 17),
                    type=CurrentTaskDecision.TYPE_BUILD_WELL,
                    agent_name=self._agent_name,
                    items=[],
                    pos=next_well_position,
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
