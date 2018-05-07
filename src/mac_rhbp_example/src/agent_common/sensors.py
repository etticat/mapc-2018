from __future__ import division # force floating point division when using plain /
import rospy

import sys
from abc import abstractmethod

from behaviour_components.sensors import RawTopicSensor, AggregationSensor
from utils.ros_helpers import get_topic_type

from agent_common.agent_utils import euclidean_distance


class AbstractFacilitySensor(RawTopicSensor):
    """
    A base class for all sensor implementations that are selecting a facility from a topic based on a reference topic
    (other facility or agent..)
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        """
        :param topic: see :class:RawTopicSensor
        :param ref_topic: Reference topic that is used to select the considered facility
        :param name: see :class:RawTopicSensor
        :param message_type: see :class:RawTopicSensor
        :param initial_value: see :class:RawTopicSensor
        :param facility_attribute: An attribute that should be taken as the sensor value from the finally selected
            facility e.g. Position, Rate, ...  if None the raw/entire facitliy is passed
        :param create_log: see :class:RawTopicSensor
        :param print_updates: see :class:RawTopicSensor
        """
        self._facilities = {}
        super(AbstractFacilitySensor, self).__init__(name=name, topic=topic, message_type=message_type,
                                                     initial_value=initial_value, create_log=create_log,
                                                     print_updates=print_updates)
        self._facility_attribute = facility_attribute

        self._latest_ref_value = None

        ref_message_type = get_topic_type(ref_topic)
        if ref_message_type is not None:
            self._sub_ref = rospy.Subscriber(ref_topic, ref_message_type, self.subscription_callback_ref_topic)
        else:
            rospy.logerr("Could not determine message type of: " + topic)

    def subscription_callback_ref_topic(self, msg):
        self._latest_ref_value = msg

    def update(self, new_value):

        for facility in new_value.facilities:
            self._facilities[facility.name] = facility

        super(AbstractFacilitySensor, self).update(newValue=new_value)

    def sync(self):

        reduced_facility_value = self._reduce_facility(facilities=self._facilities, ref_value=self._latest_ref_value)

        if self._facility_attribute:
            self._latestValue = getattr(reduced_facility_value, self._facility_attribute)
        else:
            self._latestValue = reduced_facility_value

        return super(AbstractFacilitySensor, self).sync()

    @abstractmethod
    def _reduce_facility(self, facilities, ref_value):
        """
        This method has to be implemented in order to select a facility msg from all received messages based on the
        reference value
        :param facilities: dict with sensor msgs, key is the facility name
        :param ref_value: the current reference msg
        :return: the selected facility
        """
        raise NotImplementedError()

    def __del__(self):
        super(AbstractFacilitySensor, self).__del__()
        self._sub_ref.unregister()


class ClosestFacilitySensor(AbstractFacilitySensor):
    """
    Facility sensor that determines the closest facility to the reference
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        super(ClosestFacilitySensor, self).__init__(name=name, topic=topic, ref_topic=ref_topic,
                                                    message_type=message_type,
                                                    facility_attribute=facility_attribute, initial_value=initial_value,
                                                    create_log=create_log,
                                                    print_updates=print_updates)
        self._closest_facility = None

    @property
    def closest_facility(self):
        return self._closest_facility

    def _reduce_facility(self, facilities, ref_value):
        """
        Determining the closest facility by euclidean distance
        :param facilities: dict of facilities
        :param ref_value: reference value for position comparison
        :return: closest facility
        """

        if ref_value and self._closest_facility:
            min_distance = euclidean_distance(ref_value.pos, self._closest_facility.pos)
        else:
            min_distance = sys.float_info.max

        if ref_value:
            for _, facility in facilities.iteritems():
                distance = euclidean_distance(ref_value.pos, facility.pos)
                if distance < min_distance:
                    min_distance = distance
                    self._closest_facility = facility

        return self._closest_facility


class ClosestFacilityDistanceSensor(ClosestFacilitySensor):
    """
    Facility sensor that determines the closest distance of a facility to the reference
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        super(ClosestFacilityDistanceSensor, self).__init__(name=name, topic=topic, ref_topic=ref_topic,
              message_type=message_type, facility_attribute=facility_attribute, initial_value=initial_value,
              create_log=create_log, print_updates=print_updates)

    def _reduce_facility(self, facilities, ref_value):
        """
        Overwrite the method for returning the distance to the closest facility instead of the facility itself
        :param facilities: 
        :param ref_value: 
        :return: 
        """
        closest_facility = super(ClosestFacilityDistanceSensor, self)._reduce_facility(facilities, ref_value)

        if closest_facility:

            distance = euclidean_distance(ref_value.pos, closest_facility.pos)
        else:
            distance = sys.float_info.max

        rospy.logdebug("Facility distance: %f", distance)

        return distance


class FurthestFacilitySensor(AbstractFacilitySensor):
    """
    Facility sensor that determines the furthest facility to the reference
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None,
                 create_log=False, print_updates=False):
        super(FurthestFacilitySensor, self).__init__(name=name, topic=topic, ref_topic=ref_topic, message_type=message_type,
                                                    facility_attribute=facility_attribute, initial_value=initial_value,
                                                     create_log=create_log, print_updates=print_updates)

        self._furthest_facility = None

    def _reduce_facility(self, facilities, ref_value):

        if ref_value and self._furthest_facility:
            max_distance = euclidean_distance(ref_value.pos, self._furthest_facility.pos)
        else:
            max_distance = -1

        if ref_value:
            for _, facility in facilities.iteritems():
                distance = euclidean_distance(ref_value.pos, facility.pos)
                if distance > max_distance:
                    max_distance = distance
                    self._furthest_facility = facility

        return self._furthest_facility


class DistanceSensor(AggregationSensor):
    """
    Distance Sensor for two TopicsSensors containing a "pos" (type Position) attribute
    for instance for comparing agent or facility positions
    """

    def __init__(self, name, position_sensors, optional=False, initial_value=None):
        """
        :param sensors: list of position sensors to aggregate
        """
        super(AggregationSensor, self).__init__(name=name, sensors=position_sensors, optional=optional,
                                                initial_value=initial_value)

    def _aggregate(self, sensor_values):
        """
        computes the euclidean distance between the two sensors for normalization
        """

        if sensor_values[0] is None or sensor_values[1] is None:
            normalised_value = self._initial_value
        else:
            normalised_value = euclidean_distance(sensor_values[0].pos, sensor_values[0].pos)

        return normalised_value
