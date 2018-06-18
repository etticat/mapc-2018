from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.activators import ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import TopicSensor
from behaviours.battery import ChargeBehaviour, RechargeBehaviour
from behaviours.movement import GoToFacilityBehaviour
from sensor.movement import DestinationDistanceSensor


class BatteryChargingNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, msg, name, recharge_lower_bound_percentage=0.7, recharge_critical_bound_percentage=0.2,
                 **kwargs):

        # find a charging station
        super(BatteryChargingNetworkBehaviour, self).__init__(
            name=name, **kwargs)

        self._movement_knowledge = MovementKnowledgebase()
        self._agent_name = agent._agent_name
        agent_topic = agent._agent_topic_prefix + "agent"
        proximity = msg.proximity

        agent_recharge_upper_bound = msg.role.base_battery

        agent_recharge_lower_bound = agent_recharge_upper_bound * recharge_lower_bound_percentage
        self.agent_charge_critical = agent_recharge_upper_bound * recharge_critical_bound_percentage


        # Behaviour to go to the closest Charging station
        self._go_to_charging_station_behaviour = GoToFacilityBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name="go_to_charging_station_behaviour",
            topic="/charging_station",
            agent_name=agent._agent_name,
            agent=agent)

        # Sensor that checks if vehicles is charged at the moment
        self._charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')

        # charging required condition: When closer to lower bound -> higher activation
        self._require_charging_cond = Condition(
            sensor=self._charge_sensor,
            activator=LinearActivator(
            zeroActivationValue=agent_recharge_upper_bound,
            fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty

        # Sensor to check distance to charging station
        self._charging_station_distance_sensor = DestinationDistanceSensor(
            name='at_charging_station',
            agent_name=agent._agent_name,
            behaviour_name=self._go_to_charging_station_behaviour._name)


        # Condition to check if we are at a charging station
        self._at_charging_station_cond = Condition(
            sensor=self._charging_station_distance_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))

        # battery is completely empty
        self._battery_empty_cond = Condition(
            sensor=self._charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # only look for charging stations if charging is required
        self._go_to_charging_station_behaviour.add_precondition(
            self._require_charging_cond)

        # do not look for charging station if we are already at one
        self._go_to_charging_station_behaviour.add_precondition(
            Negation(
                self._at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        self._go_to_charging_station_behaviour.add_precondition(
            Negation(
                self._battery_empty_cond))

        # looking for a charging station has an effect (transitively) of being at a charging station
        self._go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=self._charging_station_distance_sensor.name,
                indicator=-1.0,  # -1 for a reducing effect on the distance
                sensor_type=float))

        # Charge (normally)
        self._charge_behaviour = ChargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=agent._agent_name,
            name='charge_behaviour',)

        # only charge_behaviour when at charging station
        self._charge_behaviour.add_precondition(self._at_charging_station_cond)

        # only charge_behaviour if required
        self._charge_behaviour.add_precondition(self._require_charging_cond)

        #charging has an effect on charging_sensor
        self._charge_behaviour.add_effect(Effect(
            sensor_name=self._charge_sensor.name,
            indicator=2.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Recharge (through solar)
        self._recharge_behaviour = RechargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=agent._agent_name,
            name='recharge_behaviour')

        # Recharging has effect on charge_behaviour sensor
        self._recharge_behaviour.add_effect(Effect(
            sensor_name=self._charge_sensor.name,
            indicator=1.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Only recharge if battery is empty
        self._recharge_behaviour.add_precondition(self._battery_empty_cond)

        # CONDITION: Vehicle has enough charge to function
        self._enough_battery_cond = Condition(
            sensor=self._charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=self.agent_charge_critical,
                isMinimum=True))


        self._charging_goal = GoalBase(
            name='charging_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self._require_charging_cond)])

    def stop(self):

        super(BatteryChargingNetworkBehaviour, self).stop()
        self._movement_knowledge.stop_movement(self._agent_name, self._go_to_charging_station_behaviour.name)