#!/usr/bin/env python2
import copy
import itertools
import operator

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg, JobAssignment

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import JobKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton


class StepProvider(object):

    def calculate_steps(self, startPosition, endPosition, speed, transport):
        # TODO: implement
        return 7