from behaviour_components.activators import ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.sensors import TopicSensor
from behaviours.battery import GoToChargingstationBehaviour, ChargeBehaviour, RechargeBehaviour
from sensor.movement import DestinationDistanceSensor


class BatteryChargingBehaviourGraph():

    def __init__(self, agent, msg, recharge_lower_bound_percentage = 0.7, recharge_critical_bound_percentage = 0.1):


        # find a charging station
        find_charging_station = GoToChargingstationBehaviour(
            plannerPrefix=agent._agent_name,
            agent=agent)


        agent_topic = agent._agent_topic_prefix + "agent"
        proximity = msg.proximity

        agent_recharge_upper_bound = msg.role.base_battery  # TODO this would have to be updated when upgrades are performed

        agent_recharge_lower_bound = agent_recharge_upper_bound * recharge_lower_bound_percentage
        agent_charge_critical = agent_recharge_upper_bound * recharge_critical_bound_percentage

        # Sensor that checks if vehicles is charged at the moment
        charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')

        # charging required condition: When closer to lower bound -> higher activation
        charge_required_activator = LinearActivator(zeroActivationValue=agent_recharge_upper_bound,
                                    fullActivationValue=agent_recharge_lower_bound)
        require_charging_cond = Condition(
            sensor=charge_sensor,
            activator=charge_required_activator)  # highest activation already before battery empty

        # CONDITION: Vehicle has enough charge to function
        charge_critical_activator = ThresholdActivator(thresholdValue=agent_charge_critical, isMinimum=True)
        self.enough_battery_cond = Condition(
            sensor=charge_sensor,
            activator=charge_critical_activator)

        # Sensor to check distance to charging station
        at_charging_station_sensor = DestinationDistanceSensor(
            name='at_charging_station',
            agent_name=agent._agent_name,
            behaviour_name=find_charging_station._name)


        # Condition to check if we are at a charging station
        at_charging_station_activator = ThresholdActivator(thresholdValue=proximity, isMinimum=False)
        at_charging_station_cond = Condition(
            sensor=at_charging_station_sensor,
            activator=at_charging_station_activator)  # highest activation if the value is below threshold

        # battery is completely empty
        battery_empty_activator = ThresholdActivator(thresholdValue=agent_charge_critical, isMinimum=False)
        battery_empty_cond = Condition(
            sensor=charge_sensor,
            activator=battery_empty_activator)

        # only look for charging stations if charging is required
        find_charging_station.add_precondition(
            require_charging_cond)

        # do not look for charging station if we are already at one
        find_charging_station.add_precondition(
            Negation(
                at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        find_charging_station.add_precondition(
            Negation(
                battery_empty_cond))

        # looking for a charging station has an effect (transitively) of being at a charging station
        find_charging_station.add_effect(
            Effect(
                sensor_name=at_charging_station_sensor.name,
                indicator=-1.0,  # -1 for a reducing effect on the distance
                sensor_type=float))

        # Charge (normally)
        charge = ChargeBehaviour(
            plannerPrefix=agent._agent_name,
            agent_name=agent._agent_name,
            name='charge',)

        # only charge when at charging station
        charge.add_precondition(at_charging_station_cond)

        # only charge if required
        charge.add_precondition(require_charging_cond)

        #charging has an effect on charging_sensor
        charge.add_effect(Effect(
            sensor_name=charge_sensor.name,
            indicator=2.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Recharge (through solar)
        recharge = RechargeBehaviour(
            plannerPrefix=agent._agent_name,
            agent_name=agent._agent_name,
            name='recharge')

        # Recharging has effect on charge sensor
        recharge.add_effect(Effect(
            sensor_name=charge_sensor.name,
            indicator=1.0,  # here we could also use the real charging effects
            sensor_type=float))

        # Only recharge if battery is empty
        recharge.add_precondition(battery_empty_cond)

        # Only recharge if we are not at a charging station. If we are, just charge normally
        recharge.add_precondition(Negation(at_charging_station_cond))

        # The goal is to always have charge
        self._charging_goal = GoalBase(
            name='charging_goal',
            permanent=True,
            plannerPrefix=agent._agent_name,
            conditions=[Negation(require_charging_cond)])

    def get_enough_battery_cond(self):
        return self.enough_battery_cond