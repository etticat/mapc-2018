#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import Agent

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.agent')


class AgentInfoProvider(object):
    """
    Provider, that provides information on the agent
    """
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self._pos = None
        self._skill = None

        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, self._callback_agent)

    def _callback_agent(self, agent):
        """
        Retrieve agent info after every simulation step
        :param agent:
        :type agent: Agent
        :return:
        """
        self._pos = agent.pos
        self._skill = agent.skill

    @property
    def pos(self):
        return self._pos

    @property
    def skill(self):
        return self._skill
