from agent_knowledge.task import TaskBaseKnowledge
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviours.task import FinishTaskBehaviour
from behaviours.well import BuildWellBehaviour, BuildUpWellBehaviour, WellIntegritySensor
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class BuildWellNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(BuildWellNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            task_type=TaskBaseKnowledge.TYPE_BUILD_WELL,
            **kwargs)

        self.init_well_sensors()
        self.init_build_behaviour()
        self.init_build_up_behaviour()
        self.init_finish_behaviour()

    def init_well_sensors(self):
        # Sensor that checks if agent has at least one assigned task

        self.target_well_integrity_sensor = WellIntegritySensor(
            agent_name=self._agent_name,
            name="target_well_integrity_sensor")

        self._target_well_intact_condition = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=1,
                isMinimum=True
            )
        )

        self._target_well_damaged_condition = Negation(self._target_well_intact_condition)

        self._target_well_exists_sensor = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=True
            )
        )

    def init_build_behaviour(self):
        self.build_well_behaviour = BuildWellBehaviour(
            name="build_well_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.build_well_behaviour.add_precondition(
            precondition=Negation(self._target_well_exists_sensor)
        )
        self.build_well_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.build_well_behaviour, effect_on_goal=False)

    def init_build_up_behaviour(self):
        self.build_up_well_bahviour = BuildUpWellBehaviour(
            name="build_up_well_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_damaged_condition
        )
        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_exists_sensor
        )

        self.build_well_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.build_up_well_bahviour, effect_on_goal=False)

    def init_finish_behaviour(self):
        self.finish_building_behaviour = FinishTaskBehaviour(
            name="finish_build_well_behaviour",
            type=TaskBaseKnowledge.TYPE_BUILD_WELL,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.finish_building_behaviour.add_precondition(
            precondition=self._target_well_intact_condition
        )

        self.finish_building_behaviour.add_precondition(
            precondition=self._target_well_exists_sensor
        )

        self.init_do_behaviour(self.finish_building_behaviour)
