from agent_knowledge.local_knowledge_sensors import LocalKnowledgeSensor
from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Conjunction
from behaviour_components.goals import GoalBase
from behaviours.gather import ChooseIngredientBehaviour
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToTaskDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.action_provider import Action
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.gather import NextIngredientVolumeSensor
from sensor.general import SubtractionSensor
from sensor.movement import SelectedTargetPositionSensor, StepDistanceSensor


class GatheringNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(GatheringNetworkBehaviour, self).__init__(name=name, agent_name=agent_name, sensor_map=sensor_map,
                                                        **kwargs)

        self._product_provider = ProductProvider(
            agent_name=self._agent_name)

        self._task_knowledge_base = TaskKnowledgeBase()

        self._movement_knowledge = TaskKnowledgeBase()
        self._agent_name = self._agent_name

        self.init_choose_ingredient_behaviour()
        self.init_go_to_resource_behaviour()
        self.init_gather_behaviour()

        self.apply_charging_restrictions(self.go_to_resource_node_behaviour)

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
        self.gather_behviour.add_precondition(
            precondition=self.can_fulfill_next_gathering_action)
        # Only gather if we are at the intended resource node
        self.gather_behviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_behviour.add_precondition(
            precondition=self.next_item_fits_in_storage_cond)

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self._sensor_map.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=bool

            )
        )
        # Gathering will eventually lead to finishing the gathering task
        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self.has_gathering_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

    def init_go_to_resource_behaviour(self):
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GoToTaskDestinationBehaviour(
            agent_name=self._agent_name,
            task_type=TaskKnowledgeBase.TYPE_GATHERING,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix(),
        )

        self.gather_target_sensor = SelectedTargetPositionSensor(
            type=TaskKnowledgeBase.TYPE_GATHERING,
            name="gather_target_sensor",
            agent_name=self._agent_name
        )
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=self.can_fulfill_next_gathering_action)
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

    def init_choose_ingredient_behaviour(self):
        self.choose_ingredient_behaviour = ChooseIngredientBehaviour(
            name="choose_ingredient_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.has_gathering_task_sensor = LocalKnowledgeSensor(
            name="has_gathering_task_sensor",
            pattern=TaskKnowledgeBase.generate_tuple(
                agent_name=self._agent_name,
                type=TaskKnowledgeBase.TYPE_GATHERING
            ),
            knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME
        )

        self.next_ingredient_volume_sensor = NextIngredientVolumeSensor(
            agent_name=self._agent_name,
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

        self.has_gathering_task_cond = Condition(
            sensor=self.has_gathering_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )
        self.can_fulfill_next_gathering_action = Conjunction(
            self.has_gathering_task_cond,
            self.next_item_fits_in_storage_cond
        )

        # only chose an item if we currently don't have a goal
        self.choose_ingredient_behaviour.add_precondition(
            precondition=Negation(self.can_fulfill_next_gathering_action)
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_ingredient_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.has_gathering_task_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

    def stop(self):
        super(GatheringNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goal KB
        self._task_knowledge_base.finish_task(type=TaskKnowledgeBase.TYPE_GATHERING, agent_name=self._agent_name)
        self._product_provider.stop_gathering()