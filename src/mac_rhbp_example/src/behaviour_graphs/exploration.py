from agent_knowledge.movement import get_movement_tuple
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviours.exploration import ExplorationBehaviour, FinishExplorationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import DestinationDistanceSensor


class ExplorationBehaviourGraph:
    def __init__(self, agent, msg):
        self._shop_exploration = ExplorationBehaviour(agent)


        proximity = msg.proximity

        # knowledge base flag to track if we are in exploration stage or not
        shop_exploration_sensor = KnowledgeSensor(
            name='shop_exploration',
            pattern=get_movement_tuple(
                agent_name=agent._agent_name,
                behaviour=self._shop_exploration._name,
                active=False))

        shop_exploration_condition = Condition(
            sensor=shop_exploration_sensor,
            activator=BooleanActivator(desiredValue=True))

        # Condition to check if we are close to (in) a shop
        at_shop_sensor = DestinationDistanceSensor(
            name='at_shop',
            agent_name=agent._agent_name,
            behaviour_name=self._shop_exploration._name)

        at_shop_cond = Condition(
            sensor=at_shop_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

        # Exploring has the effect that we will end exploration at some point
        self._shop_exploration.add_effect(
            effect=Effect(
                sensor_name=shop_exploration_sensor.name,
                indicator=-1.0,
                sensor_type=bool))

        # exploration has the effect that we well be at the place at some point
        self._shop_exploration.add_effect(
            effect=Effect(
                sensor_name=at_shop_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        self._finish_shop_exploration = FinishExplorationBehaviour(
            plannerPrefix=agent._agent_name,
            agent_name=agent._agent_name,
            name='finish_explore_shops',
            facility_topic='/shop',
            graph_name=self._shop_exploration._name)

        self._finish_shop_exploration.add_effect(
            effect=Effect(
                sensor_name=shop_exploration_sensor.name,
                indicator=1.0,
                sensor_type=bool))

        self._finish_shop_exploration.add_precondition(
            precondition=at_shop_cond)  # only finish if we are at a charging station

        self._finish_shop_exploration.add_precondition(
            precondition=Negation(
                conditional=shop_exploration_condition))  # only execute once if exploration is not yet True

        # Exploration goal
        self._exploration_goal = GoalBase(
            name='exploration_goal',
            permanent=True,
            plannerPrefix=agent._agent_name,
            conditions=[shop_exploration_condition])
