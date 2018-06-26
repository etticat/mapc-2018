#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.battery')


class ClosestChargingStationSensor(Sensor):

    def __init__(self, agent_name, name=None):
        super(ClosestChargingStationSensor, self).__init__(name=name, initial_value=None)

        self._agent_name = agent_name
        self._last_agent_position = None

        self.movement_knowledge_base = TaskKnowledgeBase()
        self.facility_provider = FacilityProvider()
        self.distance_provider = DistanceProvider()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent,
                                         self.subscription_callback_ref_topic)

    def subscription_callback_ref_topic(self, msg):
        if msg.pos != self._last_agent_position:
            self._last_agent_position = msg.pos
            charging_stations = self.facility_provider.get_charging_stations()
            closest_facility = self.distance_provider.get_closest_facility(agent_position=self._last_agent_position, facilities=charging_stations)

            self.update(closest_facility)

    def update(self, newValue):
        if self._latestValue is not newValue:
            self.movement_knowledge_base.create_task(Task(
                type=TaskKnowledgeBase.TYPE_CHARGING_STATION,
                agent_name=self._agent_name,
                pos=newValue.pos,
                task=''))
        super(ClosestChargingStationSensor, self).update(newValue)
