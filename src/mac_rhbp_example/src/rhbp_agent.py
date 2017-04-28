#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye

from behaviour_components.managers import Manager

from behaviour_components.activators import BooleanActivator, Condition

from behaviour_components.goals import GoalBase

from behaviour_components.pddl import Effect

from rhbp_utils.knowledge_sensors import KnowledgeSensor

from agent_modules import ExplorationBehaviour,get_bridge_topic_prefix

class RhbpAgent:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True)

        self._agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        self._manager = Manager(prefix=self._agent_name)

        self._exploration = None

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._exploration: #init only once here

            rospy.loginfo(self._agent_name + " startet")

            self._exploration = ExplorationBehaviour(plannerPrefix=self._agent_name, agent_name=self._agent_name)

            exploration_sensor = KnowledgeSensor(pattern=(self._agent_name, 'exploring', 'true'))

            exploration_condition = Condition(exploration_sensor, BooleanActivator())

            self._exploration.correlations = [Effect(exploration_condition.getFunctionNames()[0], 1.0, sensorType = bool)]

            self._exploration_goal = GoalBase(name='exploration_goal', permanent=True, plannerPrefix=self._agent_name, conditions=[exploration_condition])

            #TODO alternative goals
            #number of visited facilities? (for this we could actually incorporate SO in order to exhibit patrolling using pheromones)

            #TODO add charge behaviour -> go to closest available charing station if not possible use solar??!

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + msg)

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Bye:" + msg)

    def _action_request_callback(self, msg):
        """
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        rospy.logdebug("RhbpAgent::callback %s", msg)

        self._manager.step()
        #action send is finally triggered by a selected behaviour

if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
