#!/usr/bin/env python2
import copy
import itertools
import operator

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item, WellMsg, Well
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg, JobAssignment

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import JobKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton


class WellProvider(object):
    __metaclass__ = Singleton

    WELL_TOPIC = "/well"

    def __init__(self):
        self.wells = {}
        self.wells_to_build = {}

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")


        rospy.Subscriber(WellProvider.WELL_TOPIC, WellMsg, self._callback_well)


    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        for well in sim_start.wells:
            self.wells_to_build[well.name] = well


    def _callback_well(self, wellMsg):
        """

        :param wellMsg:
        :type wellMsg: WellMsg
        :return:
        """

        for well in wellMsg.facilities:
            self._save_well(well)

    def _save_well(self, well):
        """

        :param well:
        :param well: Well
        :return:
        """

        self.wells[well.name] = well


    def get_existing_wells(self):
        return self.wells

    def get_well(self, well_type):
        return self.wells_to_build[well_type]

    def get_wells_to_build(self):
        return self.wells_to_build
