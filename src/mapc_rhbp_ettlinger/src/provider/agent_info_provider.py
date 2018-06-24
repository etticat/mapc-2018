#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import Agent

from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.provider.agent')

class AgentInfoProvider(object):
    __metaclass__ = Singleton


    def __init__(self, agent_name):
        self._pos = None
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, self._callback_agent)


    def _callback_agent(self, agent):
        self._pos = agent.pos

    @property
    def pos(self):
        return self._pos