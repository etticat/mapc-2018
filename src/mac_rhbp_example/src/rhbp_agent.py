#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye

from behaviour_components.managers import Manager
from behaviour_components.activators import BooleanActivator, Condition, ThresholdActivator, LinearActivator
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviour_components.condition_elements import Effect
from behaviour_components.sensors import SimpleTopicSensor

from rhbp_utils.knowledge_sensors import KnowledgeSensor

from agent_common.agent_modules import GotoFacilityBehaviour, GenericActionBehaviour, ClosestFacilityDistanceSensor, \
    get_bridge_topic_prefix, FinishExplorationBehaviour, get_knowledge_base_tuple_facility_exploration


class RhbpAgent:
    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.INFO)

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

        rospy.Subscriber(self._agent_topic_prefix +"generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """
        proximity = msg.proximity
        agent_recharge_upper_bound = msg.role.max_battery
        agent_recharge_lower_bound = msg.role.max_battery * 0.7
        agent_charge_critical = msg.role.max_battery * 0.1

        if not self._sim_started:  # init only once here

            self._sim_started = True

            rospy.loginfo(self._agent_name + " startet")

            agent_topic = self._agent_topic_prefix + "agent"

            # exploring shops

            self._shop_exploration = GotoFacilityBehaviour(plannerPrefix=self._agent_name,
                                                           agent_name=self._agent_name, name='explore_shops',
                                                           facility_topic='/shop')

            # knowledge base flag to track if we are in exploration stage or not
            shop_exploration_sensor = KnowledgeSensor(name='shop_exploration',
                pattern=get_knowledge_base_tuple_facility_exploration(self._agent_name,'/shop') + ('true',))
            shop_exploration_condition = Condition(shop_exploration_sensor, BooleanActivator(desiredValue=True))

            # Condition to check if we are close to (in) a shop
            at_shop_sensor = ClosestFacilityDistanceSensor(name='at_shop', topic='/shop', ref_topic=agent_topic)
            at_shop_cond = Condition(at_shop_sensor,
                                     ThresholdActivator(thresholdValue=proximity,
                                                        isMinimum=False))  # highest activation if the value is below threshold

            # here we could easily add other facilities for exploration
            self._shop_exploration.add_effect(Effect(shop_exploration_condition.getFunctionNames()[0], -1.0, sensor_type=bool))
            self._shop_exploration.add_effect(Effect(at_shop_cond.getFunctionNames()[0], -1.0, sensor_type=float))

            self._exploration_goal = GoalBase(name='exploration_goal', permanent=True, plannerPrefix=self._agent_name,
                                              conditions=[shop_exploration_condition])

            self._finish_shop_exploration = FinishExplorationBehaviour(plannerPrefix=self._agent_name,
                                    agent_name=self._agent_name, name='finish_explore_shops', facility_topic='/shop')

            self._finish_shop_exploration.add_effect(Effect(shop_exploration_condition.getFunctionNames()[0], 1.0, sensor_type=bool))

            self._finish_shop_exploration.addPrecondition(at_shop_cond) #only finish if we are at a charging station
            self._finish_shop_exploration.addPrecondition(Negation(shop_exploration_condition)) #only execute once if exploration is not yet True

            # find charging station and charge

            charge_sensor = SimpleTopicSensor(topic=agent_topic, name="charge_sensor", message_attr='charge')
            require_charging_cond = Condition(charge_sensor, LinearActivator(zeroActivationValue=agent_recharge_upper_bound,
                                                                             fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty

            enough_battery_cond = Condition(charge_sensor, ThresholdActivator(thresholdValue=agent_charge_critical, isMinimum=True))
            self._shop_exploration.addPrecondition(enough_battery_cond) # only proceed exploring if we have enough battery

            at_charging_station_sensor = ClosestFacilityDistanceSensor(name='closest_charging_station', topic='/charging_station', ref_topic=agent_topic)
            at_charging_station_cond = Condition(at_charging_station_sensor,
                                                 ThresholdActivator(thresholdValue=proximity,
                                                                    isMinimum=False))  # highest activation if the value is below threshold

            # battery is really empty
            battery_empty_cond = Condition(charge_sensor,
                                           ThresholdActivator(thresholdValue=agent_charge_critical, isMinimum=False))

            # find a charging station
            self.find_charging_station = GotoFacilityBehaviour(plannerPrefix=self._agent_name,
                                                               agent_name=self._agent_name,
                                                               name='explore_charging_stations',
                                                               facility_topic='/charging_station')

            self.find_charging_station.addPrecondition(require_charging_cond)  # only find charging station if necessary
            self.find_charging_station.addPrecondition(
                 Negation(at_charging_station_cond))  # do not move if we are already at a charging station

            self.find_charging_station.addPrecondition(
                Negation(battery_empty_cond))  # do not move if battery is almost empty to favour recharge

            self.find_charging_station.add_effect(Effect(at_charging_station_cond.getFunctionNames()[0], -1.0, # -1 for a reducing effect on the distance
                                                              sensor_type=float))

            self.charge = GenericActionBehaviour(plannerPrefix=self._agent_name,
                                                 agent_name=self._agent_name, name='charge', action_type='charge')
            self.charge.addPrecondition(at_charging_station_cond)  # only charge if we are at a charging station
            self.charge.addPrecondition(require_charging_cond)  # only charge if necessary.
            self.charge.add_effect(Effect(require_charging_cond.getFunctionNames()[0], 2.0, # here we could also use the real charging effects
                                               sensor_type=float))

            self.recharge = GenericActionBehaviour(plannerPrefix=self._agent_name,
                                                 agent_name=self._agent_name, name='recharge', action_type='recharge')
            self.recharge.add_effect(Effect(require_charging_cond.getFunctionNames()[0], 1.0, # here we could also use the real charging effects
                                               sensor_type=float))

            self.recharge.addPrecondition(battery_empty_cond)

            self._charging_goal = GoalBase(name='charging_goal', permanent=True, plannerPrefix=self._agent_name,
                                           conditions=[Negation(require_charging_cond)])

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + str(msg))
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Bye:" + str(msg))

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """
        start_time = rospy.get_rostime()
        rospy.logdebug("RhbpAgent::callback %s", str(msg))

        self._received_action_response = False

        # self._received_action_response is set to True if a generic action response was received (send by any behaviour)
        while not self._received_action_response:
            self._manager.step()
            # action send is finally triggered by a selected behaviour

        duration = rospy.get_rostime() - start_time
        rospy.loginfo("%s: Decision-making duration %f", self._agent_name, duration.to_sec())


if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
