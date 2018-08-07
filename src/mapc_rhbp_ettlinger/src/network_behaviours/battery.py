from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Disjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToDestinationBehaviour
from provider.action_provider import Action
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider


class BatteryChargingNetworkBehaviour(NetworkBehaviour):
    """
    Base Network behaviour that contains all components for Battery handling
    """
    def __init__(self, agent_name, global_rhbp_components, name, **kwargs):

        super(BatteryChargingNetworkBehaviour, self).__init__(name=name, guarantee_decision=True, **kwargs)

        self._agent_name = agent_name
        self._global_rhbp_components = global_rhbp_components

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

        # Behaviour to go to the closest Charging station
        go_to_charging_station_behaviour = GoToDestinationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + "/go_to_charging_station_behaviour",
            mechanism=self._global_rhbp_components.closest_charging_station_decision,
            agent_name=self._agent_name)
        go_to_charging_station_behaviour.add_effect(
            effect=self._global_rhbp_components.go_to_charging_station_effect
        )
        # looking for a charging station has an effect (transitively) of being at a charging station
        go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=self._global_rhbp_components.charging_station_step_sensor.name,
                indicator=-1.0,  # 1 step at a time
                sensor_type=float))

        go_to_charging_station_behaviour.add_precondition(
            precondition=self._global_rhbp_components.enough_battery_to_move_cond
        )

        self._go_to_charging_station_behaviour = go_to_charging_station_behaviour

        # Charge (normally)
        charge_behaviour = GenericActionBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
          action_type=Action.CHARGE,
            name=self.get_manager_prefix() + '/charge_behaviour', )

        # only charge_behaviour when at charging station
        charge_behaviour.add_precondition(self._global_rhbp_components.at_charging_station_cond)
        charge_behaviour.add_precondition(Negation(self._global_rhbp_components.battery_full_cond))
        charge_behaviour.add_effect(self._global_rhbp_components.charge_behaviour_effect)

        self._charge_behaviour = charge_behaviour

        # Recharge (through solar)
        recharge_behaviour = GenericActionBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
            action_type=Action.RECHARGE,
            name=self.get_manager_prefix() + '/recharge_behaviour')
        recharge_behaviour.add_effect(self._global_rhbp_components.recharge_effect)

        self._recharge_behaviour = recharge_behaviour

        # do not look for charging station if we are already at one
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._global_rhbp_components.at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._global_rhbp_components.battery_empty_cond))

        recharge_behaviour.add_precondition(self._global_rhbp_components.battery_empty_cond)
        go_to_charging_station_behaviour.add_precondition(self._global_rhbp_components.require_charging_cond)

        self.charge_goal = GoalBase(
            name='charge_goal',
            permanent=True,
            planner_prefix=self.get_manager_prefix(),
            conditions=[Negation(self._global_rhbp_components.require_charging_cond)])

    def apply_charging_restrictions(self, movement_behaviour):
        """
        This method can be used for sub classes to apply effects and preconditions to behaviours, which affect battery life
        :param movement_behaviour:
        :return:
        """

        # exploration has the effect that we well be at the place at some point
        movement_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.charge_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        movement_behaviour.add_precondition(
            precondition=self._global_rhbp_components.enough_battery_to_move_cond
        )

        movement_behaviour.add_precondition(
            precondition=Disjunction(
                Negation(self._global_rhbp_components.at_charging_station_cond),
                self._global_rhbp_components.battery_full_cond
            )
        )