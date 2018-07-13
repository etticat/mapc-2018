from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.battery import ChargeBehaviour, RechargeBehaviour
from behaviours.movement import GoToDestinationBehaviour
from decisions.battery import ClosestChargingStationDecision
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from sensor.sensor_map import SensorAndConditionMap


class BatteryChargingNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent_name, sensor_map, name, **kwargs):
        """

        :param agent:
        :param msg:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        """
        super(BatteryChargingNetworkBehaviour, self).__init__(name=name, guarantee_decision=True, **kwargs)
        self._agent_name = agent_name
        self._facility_provider = FacilityProvider()
        self._distance_provider = DistanceProvider()
        self._agent_name = self._agent_name
        self._sensor_map = sensor_map

        # Behaviour to go to the closest Charging station
        go_to_charging_station_behaviour = GoToDestinationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + "/go_to_charging_station_behaviour",
            mechanism=self._sensor_map.closest_charging_station_decision,
            agent_name=self._agent_name)
        go_to_charging_station_behaviour.add_effect(
            effect=self._sensor_map.go_to_charging_station_effect
        )
        # looking for a charging station has an effect (transitively) of being at a charging station
        go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=self._sensor_map.charging_station_step_sensor.name,
                indicator=-1.0,  # 1 step at a time
                sensor_type=float))

        go_to_charging_station_behaviour.add_precondition(
            precondition=self._sensor_map.enough_battery_to_move_cond
        )

        self._go_to_charging_station_behaviour = go_to_charging_station_behaviour

        # Charge (normally)
        charge_behaviour = ChargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
            name=self.get_manager_prefix() + '/charge_behaviour', )

        # only charge_behaviour when at charging station
        charge_behaviour.add_precondition(self._sensor_map.at_charging_station_cond)
        charge_behaviour.add_precondition(Negation(self._sensor_map.battery_full_cond))
        charge_behaviour.add_effect(self._sensor_map.charge_behaviour_effect)

        self._charge_behaviour = charge_behaviour

        # Recharge (through solar)
        recharge_behaviour = RechargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
            name=self.get_manager_prefix() + '/recharge_behaviour')
        recharge_behaviour.add_effect(self._sensor_map.recharge_effect)

        self._recharge_behaviour = recharge_behaviour

        # do not look for charging station if we are already at one
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._sensor_map.at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._sensor_map.battery_empty_cond))

        recharge_behaviour.add_precondition(self._sensor_map.battery_empty_cond)
        go_to_charging_station_behaviour.add_precondition(self._sensor_map.require_charging_cond)

        self.charge_goal = GoalBase(
            name='charge_goal',
            permanent=True,
            priority=10,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self._sensor_map.require_charging_cond)])

    def apply_charging_restrictions(self, movement_behaviour):
        # exploration has the effect that we well be at the place at some point
        movement_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._sensor_map.charge_sensor.name,
                indicator=-1.0,
                sensor_type=float))
        # TODO #86: Is this sufficient to tell the planner, that even in 5 steps it has to be met?
        movement_behaviour.add_precondition(
            precondition=self._sensor_map.enough_battery_to_move_cond
        )

        movement_behaviour.add_precondition(
            precondition=Disjunction(
                Negation(self._sensor_map.at_charging_station_cond),
                self._sensor_map.battery_full_cond
            )
        )