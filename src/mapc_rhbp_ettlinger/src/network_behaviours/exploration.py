from agent_knowledge.task import TaskBaseKnowledge
from behaviour_components.activators import ThresholdActivator, GreedyActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, Disjunction
from behaviour_components.goals import GoalBase
from behaviours.exploration import ChooseDestinationBehaviour
from behaviours.movement import GoToTaskDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import StepDistanceSensor, SelectedTargetPositionSensor


class ExplorationNetworkBehaviour(BatteryChargingNetworkBehaviour):
    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(ExplorationNetworkBehaviour, self).__init__(name=name, agent_name=agent_name, sensor_map=sensor_map,
                                                          **kwargs)

        self.init_destination_step_sensor()

        self.init_choose_destination_behaviour()
        self.init_go_to_destination_behaviour()

        self.apply_charging_restrictions(self._go_to_exploration_target_behaviour)

        self.exploration_goal = GoalBase(

            name='exploration_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Condition(
                sensor=self._sensor_map.resource_discovery_progress_sensor,
                activator=GreedyActivator()
            )])

    def init_choose_destination_behaviour(self):
        self.choose_destination_behaviour = ChooseDestinationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name='choose_destination_behaviour',
            type=TaskBaseKnowledge.TYPE_EXPLORATION,
            agent_name=self._agent_name,
        )
        self.choose_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=1.0,  # Choosing a new destination increases the distance to the target
                sensor_type=float))

        self.has_exploration_task_sensor = KnowledgeSensor(
            name="has_exploration_task_sensor",
            pattern=TaskBaseKnowledge.generate_tuple(
                agent_name=self._agent_name,
                type=TaskBaseKnowledge.TYPE_EXPLORATION
            )
        )

        self.has_exploration_task_cond = Condition(
            sensor=self.has_exploration_task_sensor,
            activator=BooleanActivator(desiredValue=True))

        self.choose_destination_behaviour.add_precondition(
            Disjunction(Negation(
                conditional=self.has_exploration_task_cond
            ), self.at_shop_cond),

        )

        self.choose_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.has_exploration_task_sensor.name,
                indicator=1.0,
                sensor_type=bool))

    def init_go_to_destination_behaviour(self):
        self._go_to_exploration_target_behaviour = GoToTaskDestinationBehaviour(
            agent_name=self._agent_name,
            task_type=TaskBaseKnowledge.TYPE_EXPLORATION,
            plannerPrefix=self.get_manager_prefix(),
            name='go_to_exploration_target',
        )
        # exploration increases the nr of resources we know
        self._go_to_exploration_target_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._sensor_map.resource_discovery_progress_sensor.name,
                indicator=0.01,  # This should be less right? TODO #86
                sensor_type=float))

        # Going to Shop gets us 1 step closer to the shop
        self._go_to_exploration_target_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        # We can only walk to shop until we are there
        self._go_to_exploration_target_behaviour.add_precondition(
            precondition=Negation(self.at_shop_cond)
        )
        # We can only walk to shop until we are there
        self._go_to_exploration_target_behaviour.add_precondition(
            precondition=self.has_exploration_task_cond
        )

    def init_destination_step_sensor(self):
        self.exploration_target_sensor = SelectedTargetPositionSensor(
            type=TaskBaseKnowledge.TYPE_EXPLORATION,
            name="exploration_target_sensor",
            agent_name=self._agent_name
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='target_step_sensor',
            position_sensor_1=self._sensor_map.agent_position_sensor,
            position_sensor_2=self.exploration_target_sensor,
            initial_value=0
        )

        self.at_shop_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

    def stop(self):
        self._movement_knowledge.finish_task(agent_name=self._agent_name, type=TaskBaseKnowledge.TYPE_EXPLORATION)
        super(ExplorationNetworkBehaviour, self).stop()
