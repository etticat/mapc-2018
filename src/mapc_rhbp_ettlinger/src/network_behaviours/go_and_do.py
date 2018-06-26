from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviours.movement import GoToTaskDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour

from sensor.movement import SelectedTargetPositionSensor, StepDistanceSensor


class GoAndDoNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, task_type, **kwargs):
        self.task_type = task_type
        self._go_behaviour = None
        self._do_behaviour = None
        super(GoAndDoNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name, **kwargs)
        self.init_destination_step_sensor()
        self.init_go_behaviour()
        self.init_task_goal()

    def init_task_goal(self):
        self.goal = GoalBase(
            name='job_fulfillment_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self._sensor_map.has_task_assigned_cond)])

    def init_do_behaviour(self, do_behaviour, effect_on_goal=True):
        # Only perform behaviour when at target destination

        self._do_behaviour = do_behaviour
        self._do_behaviour.add_precondition(
            precondition=self.at_destination_cond)

        if effect_on_goal:
            self._do_behaviour.add_effect(
                effect=Effect(
                    sensor_name=self._sensor_map.has_task_sensor.name,
                    indicator=-1.0,
                    sensor_type=bool
                )
            )


    def init_go_behaviour(self):
        ####################### GO TO DESTINATION BEHAVIOUR ###################
        self._go_behaviour = GoToTaskDestinationBehaviour(
            agent_name=self._agent_name,
            task_type=self.task_type,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix()  + '_go_behaviour',
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
        self.target_sensor = SelectedTargetPositionSensor(
            type=self.task_type,
            name=self.get_manager_prefix() + "_target_sensor",
            agent_name=self._agent_name
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name=self.get_manager_prefix() + "_target_step_sensor",
            position_sensor_1=self._sensor_map.agent_position_sensor,
            position_sensor_2=self.target_sensor,
            initial_value=0
        )

        self.at_destination_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))


