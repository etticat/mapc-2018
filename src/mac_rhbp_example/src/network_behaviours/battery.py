from behaviour_components.activators import ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import TopicSensor
from behaviours.battery import GoToChargingstationBehaviour, ChargeBehaviour, RechargeBehaviour
from sensor.movement import DestinationDistanceSensor


class BatteryChargingNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, msg, name, recharge_lower_bound_percentage=0.7, recharge_critical_bound_percentage=0.2,
                 **kwargs):

        # find a charging station
        super(BatteryChargingNetworkBehaviour, self).__init__(
            name=name, **kwargs)

        agent_topic = agent._agent_topic_prefix + "agent"
        proximity = msg.proximity

        agent_recharge_upper_bound = msg.role.base_battery  # TODO this would have to be updated when upgrades are performed

        agent_recharge_lower_bound = agent_recharge_upper_bound * recharge_lower_bound_percentage
        self.agent_charge_critical = agent_recharge_upper_bound * recharge_critical_bound_percentage


        self.go_to_charging_station_behaviour = GoToChargingstationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent=agent)

        # Sensor that checks if vehicles is charged at the moment
        self.charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')

        # charging required condition: When closer to lower bound -> higher activation
        require_charging_cond = Condition(
            sensor=self.charge_sensor,
            activator=LinearActivator(
            zeroActivationValue=agent_recharge_upper_bound,
            fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty

        # Sensor to check distance to charging station
        at_charging_station_sensor = DestinationDistanceSensor(
            name='at_charging_station',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_charging_station_behaviour._name)


        # Condition to check if we are at a charging station
        at_charging_station_activator = ThresholdActivator(thresholdValue=proximity, isMinimum=False)

        at_charging_station_cond = Condition(
            sensor=at_charging_station_sensor,
            activator=at_charging_station_activator)  # highest activation if the value is below threshold

        # battery is completely empty
        battery_empty_activator = ThresholdActivator(thresholdValue=0, isMinimum=False)
        battery_empty_cond = Condition(
            sensor=self.charge_sensor,
            activator=battery_empty_activator)

        # only look for charging stations if charging is required
        self.go_to_charging_station_behaviour.add_precondition(
            require_charging_cond)

        # do not look for charging station if we are already at one
        self.go_to_charging_station_behaviour.add_precondition(
            Negation(
                at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        self.go_to_charging_station_behaviour.add_precondition(
            Negation(
                battery_empty_cond))

        # looking for a charging station has an effect (transitively) of being at a charging station
        self.go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=at_charging_station_sensor.name,
                indicator=-1.0,  # -1 for a reducing effect on the distance
                sensor_type=float))

        # Charge (normally)
        charge = ChargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=agent._agent_name,
            name='charge',)

        # only charge when at charging station
        charge.add_precondition(at_charging_station_cond)

        # only charge if required
        charge.add_precondition(require_charging_cond)

        #charging has an effect on charging_sensor
        charge.add_effect(Effect(
            sensor_name=self.charge_sensor.name,
            indicator=2.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Recharge (through solar)
        recharge = RechargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=agent._agent_name,
            name='recharge')

        # Recharging has effect on charge sensor
        recharge.add_effect(Effect(
            sensor_name=self.charge_sensor.name,
            indicator=1.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Only recharge if battery is empty
        recharge.add_precondition(battery_empty_cond)

        self.add_precondition(require_charging_cond)

        # The goal is to always have charge
        self._charging_goal = GoalBase(
            name='charging_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(require_charging_cond)])

    def stop(self):

        super(BatteryChargingNetworkBehaviour, self).stop()
        self.go_to_charging_station_behaviour._movement_knowledge.stop_movement()