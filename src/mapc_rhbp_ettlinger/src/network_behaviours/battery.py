from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.battery import ChargeBehaviour, RechargeBehaviour
from behaviours.movement import GotoLocationBehaviour2
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider


class BatteryChargingNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, sensor_map, name, **kwargs):
        """

        :param agent:
        :param msg:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        """
        super(BatteryChargingNetworkBehaviour, self).__init__(name=name, **kwargs)

        self._movement_knowledge = MovementKnowledgebase()
        self.facility_knowledgebase = FacilityProvider()
        self.distance_provider = DistanceProvider()
        self._agent_name = agent._agent_name
        self.sensor_map = sensor_map

        # Behaviour to go to the closest Charging station
        go_to_charging_station_behaviour = GotoLocationBehaviour2(
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + "/go_to_charging_station_behaviour",
            identifier=MovementKnowledgebase.IDENTIFIER_CHARGING_STATION,
            agent_name=self._agent_name)
        go_to_charging_station_behaviour.add_effect(
            effect=self.sensor_map.go_to_charging_station_effect
        )
        # looking for a charging station has an effect (transitively) of being at a charging station
        go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=self.sensor_map._charging_station_step_sensor.name,
                indicator=-1.0,  # 1 step at a time
                sensor_type=float))

        go_to_charging_station_behaviour.add_precondition(
            precondition=self.sensor_map.enough_battery_to_move_cond
        )

        self._go_to_charging_station_behaviour = go_to_charging_station_behaviour

        # Charge (normally)
        charge_behaviour = ChargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
            name=self.get_manager_prefix() + '/charge_behaviour', )

        # only charge_behaviour when at charging station
        charge_behaviour.add_precondition(self.sensor_map._at_charging_station_cond)
        charge_behaviour.add_effect(self.sensor_map._charge_behaviour_effect)

        self._charge_behaviour = charge_behaviour

        # Recharge (through solar)
        recharge_behaviour = RechargeBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=self._agent_name,
            name=self.get_manager_prefix() + '/recharge_behaviour')
        recharge_behaviour.add_effect(self.sensor_map.recharge_effect)

        self._recharge_behaviour = recharge_behaviour

        #### Technically not needed????? ###################
        # TODO #86 why do I need those?
        # The behaviours don't seem to get enough activation before its too late (and the agent can't make it to the station anymore)
        # Ideally we don't use any preconditions for this and let the planner decide. Why does it not worko without?


        # do not look for charging station if we are already at one
        go_to_charging_station_behaviour.add_precondition(
            Negation(self.sensor_map._at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        go_to_charging_station_behaviour.add_precondition(
            Negation(self.sensor_map._battery_empty_cond))

        charge_behaviour.add_precondition(self.sensor_map._require_charging_cond)
        recharge_behaviour.add_precondition(self.sensor_map._battery_empty_cond)
        go_to_charging_station_behaviour.add_precondition(self.sensor_map._require_charging_cond)


        self.charge_goal = GoalBase(
            name='charge_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self.sensor_map._require_charging_cond)])