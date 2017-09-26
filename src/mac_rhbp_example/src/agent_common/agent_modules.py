from __future__ import division # force floating point division when using plain /
import rospy
import random
import math
import sys
from abc import abstractmethod

from builtins import list

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.activators import MultiSensorCondition
from behaviour_components.sensors import PassThroughTopicSensor
from utils.ros_helpers import get_topic_type

from knowledge_base.knowledge_base_client import KnowledgeBaseClient

from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import Shop, GenericAction, ChargingStation, Position, Job

def get_bridge_topic_prefix(agent_name):
    """
    Determine the topic prefix for all topics of the bridge node corresponding to the agent
    :param agent_name: current agents name
    :return: prefix just before the topic name of the bridge
    """
    return '/bridge_node_' + agent_name + '/'


def action_generic_simple(publisher, action_type, params=[]):
    """
    Generic helper function for publishing GenericAction msg
    :param publisher: publisher to use
    :param action_type: the type of the action msg
    :param params: optional parameter for the msg
    """
    action = GenericAction()
    action.action_type = action_type
    action.params = params
    publisher.publish(action)


def action_goto(facility_name, publisher):
    """
    Specific "goto" action publishing helper function
    :param facility_name: name of the facility we want to go to
    :param publisher: publisher to use
    """
    action = GenericAction()
    action.action_type = "goto"
    action.params = [KeyValue("Facility", facility_name)]
    publisher.publish(action)


def action_bidForJob(job_name, publisher):
    """
    Specific "bidForJob" action publishing helper function
    :param job_name: name of the job we want to bid on
    :param publisher: publisher to use
    """
    action = GenericAction()
    action.action_type = "bidForJob"
    action.params = [KeyValue("Job", job_name)]
    publisher.publish(action)


def euclidean_distance(pos1, pos2):
    """
    Calculate the euclidean distance between two positions
    :param pos1: position 1
    :type pos1: Position
    :param pos2: position 2
    :type pos2: Position
    :return: euclidean distance
    """
    return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)

def get_knowledge_base_tuple_facility_exploration(agent_name, facility):
    """
    Simple function to create uniform knowledge tuples for facility exploration
    :param agent_name: name of the considered agent
    :param facility: facility name or topic that is explored
    :return: generate tuple (agent_name, exploration_key)
    """
    return agent_name, 'exploring_' + facility


def get_knowledge_base_tuple_job_rating(agent_name, job):
    """
    Simple function to create uniform knowledge tuples for job_rating
    :param agent_name: name of the considered agent
    :param job: job name or topic that is explored
    :return: generate tuple (agent_name, exploration_key)
    """
    return agent_name, 'exploring_' + job


class GotoFacilityBehaviour(BehaviourBase):
    '''
    Behaviour that explores the environment by going to a randomly selected facility
    '''

    def __init__(self, agent_name, facility_topic,  **kwargs):
        '''
        Constructor
        '''
        super(GotoFacilityBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._facilities = {}

        self._selected_facility = None

        self.__client = KnowledgeBaseClient()

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction, queue_size=10)

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name, self._facility_topic)

        facility_topic_type = get_topic_type(facility_topic)

        if facility_topic_type:
            rospy.Subscriber(facility_topic, facility_topic_type, self._callback_facility)
        else:
            rospy.logerr(self._agent_name + "::" +self._name + ": Failed to determine topic type of " + facility_topic)

    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility

    def move(self):

        if self._selected_facility: # in case we did not find/know a facility

            rospy.loginfo(self._agent_name + "::" +self._name + " to "+ self._selected_facility.name)

            action_goto(facility_name=self._selected_facility.name, publisher=self._pub_generic_action)
        else: #backup action recharge agent
            rospy.loginfo(self._agent_name + "::" + self._name + " recharging because of missing facility.")
            action_generic_simple(publisher=self._pub_generic_action,action_type='recharge')

    def _select_facility(self):
        """
        Determine the facility we want to go to, could also be overriden in a more elaborate sub class
        :return: facility
        """
        _, facility = random.choice(list(self._facilities.items()))

        return facility

    def start(self):
        rospy.loginfo(self._agent_name + "::" + self._name + " enabled")

        self._selected_facility = self._select_facility()

        rospy.loginfo(self._agent_name + "::" + self._name + " selected facility: " + str(self._selected_facility.name))

        # we set to false in order to fulfil the requirements of FinishExplorationBehaviour
        self.__client.update(self._exploration_knowledge + ('*',), self._exploration_knowledge + ('false',), )

        self.do_step() #this is important to directly answer the request as in the start() base implementation

    def do_step(self):

        if not self._selected_facility:
            self._selected_facility = self._select_facility()

        self.move()


class FinishExplorationBehaviour(BehaviourBase):
    '''
    Behaviour that finishes an exploration cycle by setting a corresponding knowledge fact
    '''

    def __init__(self, agent_name, facility_topic,  **kwargs):
        '''
        Constructor
        '''
        super(FinishExplorationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name,
                                                                                    self._facility_topic)
        self.__client = KnowledgeBaseClient()

    def do_step(self):
        # exloration done
        self.__client.update(self._exploration_knowledge + ('*',), self._exploration_knowledge + ('true',))


class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAC actions that just need a type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params =[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(GenericActionBehaviour, self) \
            .__init__(name=name,
                      requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction, queue_size=10)

    def do_step(self):
        rospy.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)


class AbstractFacilitySensor(PassThroughTopicSensor):
    """
    A base class for all sensor implementations that are selecting a facility from a topic based on a reference topic (other facility or agent..)
    """

    def __init__(self, topic, ref_topic, name=None, message_type=None, initial_value=None, facility_attribute=None, create_log=False, print_updates=False):
        """
        :param topic: see :class:PassThroughTopicSensor
        :param ref_topic: Reference topic that is used to select the considered facility
        :param name: see :class:PassThroughTopicSensor
        :param message_type: see :class:PassThroughTopicSensor
        :param initial_value: see :class:PassThroughTopicSensor
        :param facility_attribute: An attribute that should be taken as the sensor value from the finally selected facility e.g. Position, Rate, ...
                if None the raw/entire facitliy is passed
        :param create_log: see :class:PassThroughTopicSensor
        :param print_updates: see :class:PassThroughTopicSensor
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
            self._value = getattr(reduced_facility_value, self._facility_attribute)
        else:
            self._value = reduced_facility_value
        return self._value

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
                                                    facility_attribute=facility_attribute, initial_value=initial_value, create_log=create_log,
                                                     print_updates=print_updates)

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


class DistanceCondition(MultiSensorCondition):
    """
    Distance condition for two topics containing a "pos" (type Position) attribute
    for instance for comparing agent or facility positions
    """

    def __init__(self, sensors, activator, name = None, optional = False):
        super(DistanceCondition, self).__init__(sensors=sensors, activator=activator, name=name,optional=optional )


    def _normalize(self):
        """
        computes the euclidean distance between the two sensors for normalization
        """

        if self._sensors[0].value == None or self._sensors[1].value == None:
            normalizedSensorValue = self._activator._zeroActivationValue
        else:
            normalizedSensorValue = euclidean_distance(self._sensors[0].pos, self._sensors[1].pos)

        self._normalizedSensorValues[self._sensors[0]] = normalizedSensorValue
        self._normalizedSensorValues[self._sensors[1]] = normalizedSensorValue

    def getWishes(self):
        try:
            effect_name = self._get_pddl_effect_name(self._sensors[1])
            return [(effect_name, self._activator.getSensorWish(self._normalizedSensorValues[self._sensors[1]]))]
        except AssertionError:
            rospy.logwarn("Wrong data type for %s in %s. Got %s. Possibly uninitialized%s sensor %s?", self._sensors,
                          self._name, type(self._sensors.value), " optional" if self._sensors.optional else "",
                          self._sensors.name)
            raise

    def _reduceSatisfaction(self):
        # just return the first one, since it will always be the same
        return self._sensorSatisfactions[self._sensors[0]]