#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Position

from behaviour_components.sensors import AggregationSensor, Sensor
from common_utils import etti_logging
from provider.distance_provider import DistanceProvider
from rhbp_utils.knowledge_sensors import KnowledgeFactSensor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.movement')


class StepDistanceSensor(Sensor):
    """
    Sensor that takes two other sensors which provide a position and calculates the step distance between them
    """

    def __init__(self, name, agent_name, destination_sensor, initial_value, use_in_facility_flag=True):
        super(StepDistanceSensor, self).__init__(
            name=name,
            initial_value=initial_value)
        self._destination_sensor = destination_sensor
        self._use_in_facility_flag = use_in_facility_flag
        self._distance_provider = DistanceProvider(agent_name=agent_name)

    def sync(self):

        destination_position = self._destination_sensor.sync()

        if destination_position is None:
            return self._initial_value

        # This sensor can also take objects that hold a position. Extract it here
        if not isinstance(destination_position, Position):
            destination_position = destination_position.pos

        steps = self._distance_provider.calculate_steps(destination_position, self._use_in_facility_flag)

        self.update(steps)

        res = super(StepDistanceSensor, self).sync()

        return res
