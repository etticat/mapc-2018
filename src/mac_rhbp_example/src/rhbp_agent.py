#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye

from behaviour_components.managers import Manager
from behaviour_components.activators import BooleanActivator, Condition, ThresholdActivator, LinearActivator
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviour_components.pddl import Effect
from behaviour_components.sensors import SimpleTopicSensor

from rhbp_utils.knowledge_sensors import KnowledgeSensor

from agent_modules import GotoFacilityBehaviour, GenericActionBehaviour, ClosestFacilityDistanceSensor, \
    get_bridge_topic_prefix


class RhbpAgent:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True)

        self._agent_name = rospy.get_param('~agent_name', 'agentA1')  # default for debugging 'agentA1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1) #ensure also max_parallel_behaviours during debugging

        self._shop_exploration = None

        self._sim_started = False

        # subscribe to MAC bridge core simulation topics
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
        # TODO get proximity from msg?
        proximity = 0.0002
        agent_recharge_upper_bound = 500
        agent_recharge_lower_bound = 350
        agent_charge_critical = 100

        if not self._sim_started:  # init only once here

            self._sim_started = True

            rospy.loginfo(self._agent_name + " startet")

            # exploring shops

            self._shop_exploration = GotoFacilityBehaviour(plannerPrefix=self._agent_name, \
                                                           agent_name=self._agent_name, name='explore_shops',
                                                           facility_topic='/shop')

            shop_exploration_sensor = KnowledgeSensor(
                pattern=self._shop_exploration.get_knowledge_base_tuple() + ('true',))
            shop_exploration_condition = Condition(shop_exploration_sensor, BooleanActivator())
            # here we could easily add other facilities for exploration
            self._shop_exploration.correlations = [
                Effect(shop_exploration_condition.getFunctionNames()[0], 1.0, sensorType=bool)]

            self._exploration_goal = GoalBase(name='exploration_goal', permanent=True, plannerPrefix=self._agent_name,
                                              conditions=[shop_exploration_condition])

            # TODO add backup solar "recharge" behaviour

            # find charging station and charge

            agent_topic = self._agent_topic_prefix + "agent"

            charge_sensor = SimpleTopicSensor(topic=agent_topic, name="charge_sensor", message_attr='charge')
            require_charging_cond = Condition(charge_sensor, LinearActivator(  zeroActivationValue=agent_recharge_upper_bound,
                                                                               fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty

            enough_battery_cond = Condition(charge_sensor, ThresholdActivator(thresholdValue=agent_charge_critical, isMinimum=True))

            self._shop_exploration.addPrecondition(enough_battery_cond) # only proceed exploring if we have enough battery

            at_charging_station_sensor = ClosestFacilityDistanceSensor(topic='/charging_station', ref_topic=agent_topic)
            at_charging_station_cond = Condition(at_charging_station_sensor,
                                                 ThresholdActivator(thresholdValue=proximity,
                                                                    isMinimum=False))  # highest activation if the value is below threshold

            self.find_charging_station = GotoFacilityBehaviour(plannerPrefix=self._agent_name, \
                                                               agent_name=self._agent_name,
                                                               name='explore_charging_stations',
                                                               facility_topic='/charging_station')

            self.find_charging_station.addPrecondition(require_charging_cond)  # only find charing station if necessary
            self.find_charging_station.addPrecondition(
                 Negation(at_charging_station_cond))  # do not move if we are already at a charging station
            self.find_charging_station.correlations = [Effect(at_charging_station_cond.getFunctionNames()[0], -1.0,
                                                              sensorType=float)]  # -1 for reducing effect on the distance

            self.charge = GenericActionBehaviour(plannerPrefix=self._agent_name, \
                                                 agent_name=self._agent_name, name='charge', action_type='charge')
            self.charge.addPrecondition(at_charging_station_cond)  # only charge if we are at a charging station
            self.charge.addPrecondition(require_charging_cond)  # only charge if necessary.
            self.charge.correlations = [Effect(require_charging_cond.getFunctionNames()[0], -1.0,
                                               sensorType=float)]  # -1 for reducing the need of charging

            self._charging_goal = GoalBase(name='charging_goal', permanent=True, plannerPrefix=self._agent_name,
                                           conditions=[Negation(require_charging_cond)])

            # TODO more ideas
            # goal/sensor for number of visited facilities? (for this we could actually incorporate SO in order to exhibit patrolling using pheromones)

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + msg)
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Bye:" + msg)

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        rospy.logdebug("RhbpAgent::callback %s", msg)

        self._manager.step()
        # action send is finally triggered by a selected behaviour


if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
