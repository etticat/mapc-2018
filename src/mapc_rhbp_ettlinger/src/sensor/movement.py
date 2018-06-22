#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Agent, Position
from mapc_rhbp_ettlinger.msg import Movement

from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.sensors import Sensor, AggregationSensor
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor, KnowledgeFactSensor

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.sensors.movement')

class DestinationDistanceSensor(KnowledgeFirstFactSensor):

    def __init__(self, agent_name, behaviour_name, name):

        pattern = MovementKnowledgebase.generate_tuple(agent_name=agent_name, behaviour=behaviour_name)

        super(DestinationDistanceSensor, self).__init__(pattern=pattern, name=name, initial_value=999.0)

        self._last_pos = None

        self._movement_knowledge = MovementKnowledgebase()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self.subscription_callback_ref_topic)

    def subscription_callback_ref_topic(self, msg):
        if self._last_pos != msg.pos:
            self._last_pos = msg.pos
            self._cache_update_callback() # Force update when position of agent changes

    def _reduce_facts(self, facts):
        """
        Reduce the tuple of facts to a single value
        :param facts: fact tuple
        :return: single value, e.g. bool, float, str
        """

        # If we don't have a destination we handle it as if we are far away
        res = 1.0

        if len(facts) > 0 and self._last_pos is not None:
            movement = MovementKnowledgebase.generate_movement_from_fact(facts.pop())  # only getting the first fact

            try:

                destination_pos = movement.pos
                agent_position = self._last_pos

                res = AgentUtils.calculate_distance(destination_pos, agent_position)

            except Exception:
                ettilog.logerr("Couldn't get last tuple element of: %s. Resetting to initial_value", str(movement))

        return res

class SelectedTargetPositionSensor(KnowledgeFactSensor):

    def __init__(self, identifier, agent_name, name=None):
        super(SelectedTargetPositionSensor, self).__init__(
            name=name,
            initial_value=None,
            pattern=MovementKnowledgebase.generate_tuple(agent_name=agent_name, identifier=identifier)
        )


    def update(self, newValue):
        if len(newValue) > 0:
            movement = MovementKnowledgebase.generate_movement_from_fact(newValue.pop())  # only getting the first fact

            if movement is not None:
                destination = movement.pos
            else:
                destination = None

            super(SelectedTargetPositionSensor, self).update(destination)
        else:
            super(SelectedTargetPositionSensor, self).update(None)


class StepDistanceSensor(AggregationSensor):

    def __init__(self, name, position_sensor_1, position_sensor_2, initial_value):
        super(StepDistanceSensor, self).__init__(
            name=name,
            sensors=[position_sensor_1, position_sensor_2],
            func=None,
            initial_value=initial_value)
        self.distance_provider = DistanceProvider()

    def _aggregate(self, sensor_values):
        assert len(sensor_values) == 2


        pos1 = sensor_values[0]
        pos2 = sensor_values[1]

        if pos1 is None or pos2 is None:
            rospy.logerr("Cant get distance of %s and %s", str(pos1), str(pos2))
            return self._initial_value

        if not isinstance(pos1, Position):
            pos1 = pos1.pos

        if not isinstance(pos2, Position):
            pos2 = pos2.pos

        steps = self.distance_provider.calculate_steps(pos1, pos2)

        if self._name == "charging_station_step_distance":
            ettilog.logerr("STEPS: %s", steps)
        return steps

class ClosestChargingStationSensor(Sensor):

    def __init__(self, agent_name, name=None, optional=False, initial_value=None):
        super(ClosestChargingStationSensor, self).__init__(name=name, initial_value=None)

        self._agent_name = agent_name
        self._last_agent_position = None

        self.movement_knowledgebase = MovementKnowledgebase()
        self.facility_provider = FacilityProvider()
        self.distance_provider = DistanceProvider()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent,
                                         self.subscription_callback_ref_topic)



    def subscription_callback_ref_topic(self, msg):
        if msg.pos != self._last_agent_position:
            self._last_agent_position = msg.pos
            charging_stations = self.facility_provider.get_charging_stations()
            closest_facility = self.distance_provider.get_closest_facility(self._last_agent_position, charging_stations)

            self.update(closest_facility)


    def update(self, newValue):
        if self._latestValue is not newValue:
            self.movement_knowledgebase.start_movement(Movement(
                identifier = MovementKnowledgebase.IDENTIFIER_CHARGING_STATION,
                agent_name = self._agent_name,
                pos = newValue.pos,
                destination = ''
            ))
        super(ClosestChargingStationSensor, self).update(newValue)