from __future__ import division # force floating point division when using plain /
import rospy
import random

from builtins import list

from behaviour_components.behaviours import BehaviourBase

from utils.ros_helpers import get_topic_type

from knowledge_base.knowledge_base_client import KnowledgeBaseClient

from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from agent_common.agent_utils import get_bridge_topic_prefix, get_knowledge_base_tuple_facility_exploration


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


def action_bid_for_job(job_name, publisher):
    """
    Specific "bidForJob" action publishing helper function
    :param job_name: name of the job we want to bid on
    :param publisher: publisher to use
    """
    action = GenericAction()
    action.action_type = "bidForJob"
    action.params = [KeyValue("Job", job_name)]
    publisher.publish(action)


class GotoFacilityBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, facility_topic,  **kwargs):

        super(GotoFacilityBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._facilities = {}

        self._selected_facility = None

        self.__client = KnowledgeBaseClient()

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name,
                                                                                    self._facility_topic)

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
        else: # backup action recharge agent
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

        self.do_step() # this is important to directly answer the request as in the start() base implementation

    def do_step(self):

        if not self._selected_facility:
            self._selected_facility = self._select_facility()

        self.move()


class FinishExplorationBehaviour(BehaviourBase):
    """
    Behaviour that finishes an exploration cycle by setting a corresponding knowledge fact
    """

    def __init__(self, agent_name, facility_topic,  **kwargs):
        """
        Constructor
        """
        super(FinishExplorationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._exploration_knowledge = get_knowledge_base_tuple_facility_exploration(self._agent_name,
                                                                                    self._facility_topic)
        self.__client = KnowledgeBaseClient()

    def do_step(self):
        # exploration done
        self.__client.update(self._exploration_knowledge + ('*',), self._exploration_knowledge + ('true',))


class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAC actions that just need a type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params=[], **kwargs):
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
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        rospy.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)
