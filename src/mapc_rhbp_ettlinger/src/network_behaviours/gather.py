from agent_behaviours.generic_action_behaviour import Action, GenericActionBehaviour
from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviours.gather import ChooseIngredientBehaviour
from behaviours.movement import GotoLocationBehaviour2
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import SelectedTargetPositionSensor, StepDistanceSensor


class GatheringNetworkBehaviour(BatteryChargingNetworkBehaviour):

    def __init__(self, agent, name, msg, sensor_map, **kwargs):

        proximity = msg.proximity
        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        self._movement_knowledge = MovementKnowledgebase()
        self._agent_name= agent._agent_name

        super(GatheringNetworkBehaviour, self).__init__(name=name, agent=agent, msg=msg, sensor_map=sensor_map, **kwargs)

        self.init_choose_ingredient_behaviour(agent, proximity)
        self.init_go_to_resource_behaviour(agent, proximity, sensor_map)
        self.init_gather_behaviour(agent, sensor_map)


    def init_gather_behaviour(self, agent, sensor_map):
        ############### Gathering ##########################
        self.gather_behviour = GenericActionBehaviour(
            name="gather_behviour",
            action_type=Action.GATHER,
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=self.has_gathering_task_cond)
        # Only gather if we are at the intended resource node
        self.gather_behviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=sensor_map.load_factor_sensor.name,
                indicator=-1.0,
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

    def init_go_to_resource_behaviour(self, agent, proximity, sensor_map):
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GotoLocationBehaviour2(
            agent_name=agent._agent_name,
            identifier=MovementKnowledgebase.IDENTIFIER_GATHERING,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix()
        )

        self.gather_target_sensor = SelectedTargetPositionSensor(
            identifier=MovementKnowledgebase.IDENTIFIER_GATHERING,
            name="gather_target_sensor",
            agent_name=agent._agent_name
        )
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=self.has_gathering_task_cond)
        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='gather_target_step_sensor',
            position_sensor_1=sensor_map.agent_position_sensor,
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


    def init_choose_ingredient_behaviour(self, agent, proximity):
        self.choose_ingredient_behaviour = ChooseIngredientBehaviour(
            name="choose_ingredient_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.has_gathering_task_sensor = KnowledgeSensor(
            name="has_gathering_task_sensor",
            pattern=MovementKnowledgebase.generate_tuple(
                agent_name=agent._agent_name,
                identifier=MovementKnowledgebase.IDENTIFIER_GATHERING
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

