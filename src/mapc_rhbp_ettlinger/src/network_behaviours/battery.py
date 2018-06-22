from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.activators import ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.sensors import TopicSensor
from behaviours.battery import ChargeBehaviour, RechargeBehaviour
from behaviours.movement import GotoLocationBehaviour2
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from sensor.movement import StepDistanceSensor, ClosestChargingStationSensor


class BatteryChargingNetworkBehaviour():

    def __init__(self, agent, msg, sensor_map, recharge_lower_bound_percentage=0.7, recharge_critical_bound_percentage=0.2,):

        self._movement_knowledge = MovementKnowledgebase()
        self.facility_knowledgebase = FacilityProvider()
        self.distance_provider = DistanceProvider()
        self._agent_name = agent._agent_name
        agent_topic = agent._agent_topic_prefix + "agent"
        self._last_agent_pos = None
        proximity = msg.proximity

        self._go_to_charging_station_behaviours = {}
        self._charge_behaviours= {}
        self._recharge_behaviours= {}

        agent_recharge_upper_bound = msg.role.base_battery

        agent_recharge_lower_bound = agent_recharge_upper_bound * recharge_lower_bound_percentage
        self.agent_charge_critical = agent_recharge_upper_bound * recharge_critical_bound_percentage

        # Sensor that checks if vehicles is charged at the moment
        self.charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')

        self.init_step_sensor(agent_name=agent._agent_name, sensor_map=sensor_map)


        # charging required condition: When closer to lower bound -> higher activation
        self._require_charging_cond = Condition(
            sensor=self.charge_sensor,
            activator=LinearActivator(
            zeroActivationValue=agent_recharge_upper_bound,
            fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty


        # Condition to check if we are at a charging station
        self._at_charging_station_cond = Condition(
            sensor=self._charging_station_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # battery is completely empty
        self._battery_empty_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # CONDITION: Vehicle has enough charge to function
        self.enough_battery_to_move_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=1,
                isMinimum=True))


        # Movement decreases charge
        self.go_to_charging_station_effect = \
            Effect(
                sensor_name=self.charge_sensor.name,
                sensor_type=float,
                indicator=-1.0)

        # Recharging has effect on charge_behaviour sensor
        self.recharge_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=1.0,
            sensor_type=float)

        self._charge_behaviour_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=100,  # here we could also use the real charging effects
            # this is replaced immediately by the subscriber callback
            sensor_type=float)

        # Only recharge if battery is empty
        # self._recharge_behaviour.add_precondition(self._battery_empty_cond)

    def init_step_sensor(self, agent_name, sensor_map):

        self.closest_charging_station_sensor = ClosestChargingStationSensor(
            name="closest_charging_station_sensor",
            agent_name=agent_name
        )

        # Sensor to check distance to charging station
        self._charging_station_step_sensor = StepDistanceSensor(
            name='charging_station_step_distance',
            position_sensor_1=sensor_map.agent_position_sensor,
            position_sensor_2=self.closest_charging_station_sensor,
            initial_value=10
        )

    def init_charge_behaviours(self, planner_prefix):
        # Behaviour to go to the closest Charging station
        go_to_charging_station_behaviour = GotoLocationBehaviour2(
            plannerPrefix=planner_prefix,
            name=planner_prefix + "/go_to_charging_station_behaviour",
            identifier=MovementKnowledgebase.IDENTIFIER_CHARGING_STATION,
            agent_name=self._agent_name)
        go_to_charging_station_behaviour.add_effect(
            effect=self.go_to_charging_station_effect
        )
        # looking for a charging station has an effect (transitively) of being at a charging station
        go_to_charging_station_behaviour.add_effect(
            Effect(
                sensor_name=self._charging_station_step_sensor.name,
                indicator=-1.0,  # 1 step at a time
                sensor_type=float))

        go_to_charging_station_behaviour.add_precondition(
            precondition=self.enough_battery_to_move_cond
        )

        self._go_to_charging_station_behaviours[planner_prefix] = go_to_charging_station_behaviour

        # Charge (normally)
        charge_behaviour = ChargeBehaviour(
            plannerPrefix=planner_prefix,
            agent_name=self._agent_name,
            name=planner_prefix + '/charge_behaviour', )

        # only charge_behaviour when at charging station
        charge_behaviour.add_precondition(self._at_charging_station_cond)
        charge_behaviour.add_effect(self._charge_behaviour_effect)

        self._charge_behaviours[planner_prefix] = charge_behaviour

        # Recharge (through solar)
        recharge_behaviour = RechargeBehaviour(
            plannerPrefix=planner_prefix,
            agent_name=self._agent_name,
            name=planner_prefix + '/recharge_behaviour')
        recharge_behaviour.add_effect(self.recharge_effect)

        self._recharge_behaviours[planner_prefix] = recharge_behaviour

        #### Technically not needed????? ###################
        # TODO #86 why do I need those?
        # The behaviours don't seem to get enough activation before its too late (and the agent can't make it to the station anymore)
        # Ideally we don't use any preconditions for this and let the planner decide. Why does it not worko without?


        # do not look for charging station if we are already at one
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._at_charging_station_cond))

        # do not try to find charging station when battery is completely empty
        go_to_charging_station_behaviour.add_precondition(
            Negation(self._battery_empty_cond))

        charge_behaviour.add_precondition(self._require_charging_cond)
        recharge_behaviour.add_precondition(self._battery_empty_cond)
        go_to_charging_station_behaviour.add_precondition(self._require_charging_cond)


        self.charge_goal = GoalBase(
            name='charge_goal',
            permanent=True,
            plannerPrefix=planner_prefix,
            conditions=[Negation(self._require_charging_cond)])