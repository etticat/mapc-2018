from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

from sensor.movement import StepDistanceSensor


class GoAndDoNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, mechanism, **kwargs):
        self._go_behaviour = None
        self._reset_destination_behaviour = None
        self.destination_decision = mechanism
        super(GoAndDoNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name, **kwargs)
        self.init_destination_step_sensor()
        self.init_go_behaviour()



        self.move_goal = GoalBase(
            name='move_goal',
            permanent=True,
            priority=15,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self.at_destination_cond])

    def init_do_behaviour(self, do_behaviour, effect_on_goal=True):
        # Only perform behaviour when at target destination

        self._reset_destination_behaviour = do_behaviour
        self._reset_destination_behaviour.add_precondition(
            precondition=self.at_destination_cond)

        if effect_on_goal:
            pass
            # TODO: Create goal here?

    def init_go_behaviour(self):
        ####################### GO TO DESTINATION BEHAVIOUR ###################
        self._go_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix()  + '_go_behaviour',
            mechanism=self.destination_decision
        )

        # Going to Shop gets us 1 step closer to the shop
        self._go_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        # We can only walk to shop until we are there
        self._go_behaviour.add_precondition(
            precondition=Negation(self.at_destination_cond)
        )
        self.apply_charging_restrictions(self._go_behaviour)

    def init_destination_step_sensor(self):
        self.target_sensor = GradientSensor(
            mechanism=self.destination_decision,
            name=self.get_manager_prefix() + "_target_sensor",
            sensor_type=SENSOR.CACHED_VALUE
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name=self.get_manager_prefix() + "_target_step_sensor",
            position_sensor_1=self._sensor_map.agent_position_sensor,
            position_sensor_2=self.target_sensor,
            initial_value=10
        )

        self.at_destination_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))


