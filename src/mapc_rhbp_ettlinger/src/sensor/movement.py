#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Position


from behaviour_components.sensors import AggregationSensor
from common_utils import etti_logging
from provider.distance_provider import DistanceProvider
from rhbp_utils.knowledge_sensors import KnowledgeFactSensor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.movement')




class StepDistanceSensor(AggregationSensor):

    def __init__(self, name, position_sensor_1, position_sensor_2, initial_value):
        super(StepDistanceSensor, self).__init__(
            name=name,
            sensors=[position_sensor_1, position_sensor_2],
            func=None,
            initial_value=initial_value)
        self.distance_provider = DistanceProvider()
        self.log = False

    def _aggregate(self, sensor_values):
        assert len(sensor_values) == 2

        pos1 = sensor_values[0]
        pos2 = sensor_values[1]
        if pos1 is None or pos2 is None:
            if self.log:
                rospy.logerr("StepDistanceSensor(%s):: Cant get distance of %s and %s", self.name, str(pos1), str(pos2))

            return self._initial_value

        if not isinstance(pos1, Position):
            pos1 = pos1.pos

        if not isinstance(pos2, Position):
            pos2 = pos2.pos

        steps = self.distance_provider.calculate_steps(pos1, pos2)

        return steps

