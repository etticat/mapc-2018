
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Conjunction
from behaviour_components.goals import GoalBase
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToDestinationBehaviour
from decisions.p_task_decision import CurrentTaskDecision
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.action_provider import Action
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.gather import NextVolumeSensor
from sensor.general import SubtractionSensor
from sensor.movement import StepDistanceSensor


class GatheringNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(GatheringNetworkBehaviour, self).__init__(name=name, agent_name=agent_name, sensor_map=sensor_map,
                                                         **kwargs)

        self._product_provider = ProductProvider(
            agent_name=self._agent_name)

        self._agent_name = self._agent_name

        self.init_sensors(sensor_map=sensor_map)
        self.init_go_to_resource_behaviour(sensor_map)
        self.init_gather_behaviour()

        self.apply_charging_restrictions(self.go_to_resource_node_behaviour)



        self.move_goal = GoalBase(
            name='move_goal',
            permanent=True,
            priority=15,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self.at_resource_node_condition])

        self.charge_goal = GoalBase(
            name='gather_goal',
            permanent=True,
            priority=100,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self._sensor_map.load_fullness_condition])

    def init_gather_behaviour(self):
        ############### Gathering ##########################
        self.gather_behviour = GenericActionBehaviour(
            name="gather_behaviour",
            action_type=Action.GATHER,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        # Only gather if we are at the intended resource node
        self.gather_behviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_behviour.add_precondition(
            precondition=self.next_item_fits_in_storage_cond)

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self._sensor_map.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

    def init_go_to_resource_behaviour(self, sensor_map):
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix(),
            mechanism=sensor_map.gather_decision_mechanism
        )

        self.gather_target_sensor = GradientSensor(
            mechanism=self._sensor_map.gather_decision_mechanism,
            name="gather_target_sensor",
            sensor_type=SENSOR.VALUE
        )
        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='gather_target_step_sensor',
            position_sensor_1=self._sensor_map.agent_position_sensor,
            position_sensor_2=self.gather_target_sensor,
            initial_value=10 # TODO: Find out when initial value is used (When target sensor fails?)
        )
        self.at_resource_node_condition = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))  # highest activation if the value is below threshold

        # Only go to resource node if we aren't already there
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.at_resource_node_condition))

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,  # decreasing by 1 step
                sensor_type=float

            )
        )


    def init_sensors(self, sensor_map):

        self.next_ingredient_volume_sensor = GradientSensor(
            mechanism=sensor_map.gather_decision_mechanism,
            sensor_type=SENSOR.VALUE_ATTRIBUTE,
            attr_name="item.volume",
            initial_value=0,
            name="next_ingredient_volume_sensor"
        )

        self.load_after_next_ingredient_sensor = SubtractionSensor(
            minuend_sensor=self._sensor_map.free_load_sensor,
            subtrahend_sensor=self.next_ingredient_volume_sensor,
            name="load_after_next_ingredient_sensor"
        )
        self.next_item_fits_in_storage_cond = Condition(
            sensor=self.load_after_next_ingredient_sensor,
            activator=ThresholdActivator(thresholdValue=0, isMinimum=True)
        )


    def stop(self):
        super(GatheringNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goal KB
        self._sensor_map.gather_decision_mechanism.end_gathering()