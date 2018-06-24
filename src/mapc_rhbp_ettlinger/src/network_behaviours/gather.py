from agent_knowledge.task import TaskKnowledgebase
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviours.gather import ChooseIngredientBehaviour
from behaviours.generic_action import Action, GenericActionBehaviour
from behaviours.movement import GotoLocationBehaviour2
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import SelectedTargetPositionSensor, StepDistanceSensor


class GatheringNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):


        super(GatheringNetworkBehaviour, self).__init__(name=name, agent_name=agent_name, sensor_map=sensor_map, **kwargs)

        self._product_provider = ProductProvider(
            agent_name=self._agent_name)

        self._movement_knowledge = TaskKnowledgebase()
        self._agent_name= self._agent_name

        self.init_choose_ingredient_behaviour()
        self.init_go_to_resource_behaviour()
        self.init_gather_behaviour()

        self.apply_charging_restrictions(self.go_to_resource_node_behaviour)


        self.charge_goal = GoalBase(
            name='gather_goal',
            permanent=True,
            priority=100,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self.sensor_map.load_fullnes_condition])


    def init_gather_behaviour(self):
        ############### Gathering ##########################
        self.gather_behviour = GenericActionBehaviour(
            name="gather_behviour",
            action_type=Action.GATHER,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.gather_behviour.add_precondition(
            precondition=self.has_gathering_task_cond)
        # Only gather if we are at the intended resource node
        self.gather_behviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self.sensor_map.load_factor_sensor.name,
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
        self.go_to_resource_node_behaviour = GotoLocationBehaviour2(
            agent_name=self._agent_name,
            task_type=TaskKnowledgebase.TYPE_GATHERING,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix()
        )

        self.gather_target_sensor = SelectedTargetPositionSensor(
            type=TaskKnowledgebase.TYPE_GATHERING,
            name="gather_target_sensor",
            agent_name=self._agent_name
        )
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=self.has_gathering_task_cond)
        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='gather_target_step_sensor',
            position_sensor_1=self.sensor_map.agent_position_sensor,
            position_sensor_2=self.gather_target_sensor,
            initial_value=0
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
                indicator=-1.0, # decreasing by 1 step
                sensor_type=float

            )
        )


    def init_choose_ingredient_behaviour(self):
        self.choose_ingredient_behaviour = ChooseIngredientBehaviour(
            name="choose_ingredient_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.has_gathering_task_sensor = KnowledgeSensor(
            name="has_gathering_task_sensor",
            pattern=TaskKnowledgebase.generate_tuple(
                agent_name=self._agent_name,
                type=TaskKnowledgebase.TYPE_GATHERING
            )
        )
        self.has_gathering_task_cond = Condition(
            sensor=self.has_gathering_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        # only chose an item if we currently don't have a goal
        self.choose_ingredient_behaviour.add_precondition(
            precondition=Negation(self.has_gathering_task_cond)
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_ingredient_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.has_gathering_task_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

