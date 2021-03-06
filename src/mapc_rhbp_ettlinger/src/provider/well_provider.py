#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.well')


class WellProvider(object):
    """
    Well provider, that keeps track of all wells of the current team
    """
    __metaclass__ = Singleton

    WELL_TOPIC = "/well"

    def __init__(self, agent_name):
        self._possible_wells = {}

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart,
                         self.callback_sim_start)

    def callback_sim_start(self, sim_start):
        """
        Keep track of all wells in current simulation
        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        # Reset known wells for the new simulation
        self._possible_wells = {}

        # Safe all wells that can be built
        for well in sim_start.wells:
            self._possible_wells[well.name] = well

    def get_well(self, well_type):
        """
        Returns a well object by well type
        :param well_type:
        :return:
        """
        return self._possible_wells[well_type]

    def get_wells_to_build(self):
        """
        Returns all wells that are possible to build in current simulation
        :return:
        """
        return self._possible_wells

    def get_well_price(self, well_type):
        """
        Returns cost of well in massium
        :param well_type:
        :return:
        """
        return self._possible_wells[well_type].cost

    @property
    def possible_wells(self):
        return self._possible_wells
