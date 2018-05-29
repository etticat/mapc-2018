from agent_knowledge.movement import MovementKnowledge
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.exploration import ExplorationBehaviour, FinishExplorationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import DestinationDistanceSensor


class ExplorationBehaviourNetwork(NetworkBehaviour):
    def __init__(self, agent, msg, name, **kwargs):
        super(ExplorationBehaviourNetwork, self).__init__(name, **kwargs)

        proximity = msg.proximity

        self._shop_exploration = ExplorationBehaviour(
            agent_name=agent._agent_name,
            plannerPrefix = self.get_manager_prefix(),
            name = 'explore_shops'
        )

        # TODO: This sensor checks  how much of the map has already been seen.
        # TODO: Currently it just always returns false
        self.map_discovery_progress_sensor = KnowledgeSensor(
            name='map_discovery_sensor',
            pattern=("something", "that", "definetly", "does", "not", "exist"))
        self.map_discovered_everything_condition = Condition(
            sensor=self.map_discovery_progress_sensor,
            activator=BooleanActivator(
                desiredValue=True))


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
                sensor_name=self.map_discovery_progress_sensor.name,
                indicator=-1.0,
                sensor_type=bool))

        # exploration has the effect that we well be at the place at some point
        self._shop_exploration.add_effect(
            effect=Effect(
                sensor_name=at_shop_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        self._finish_shop_exploration = FinishExplorationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent_name=agent._agent_name,
            name='finish_explore_shops',
            facility_topic='/shop',
            graph_name=self._shop_exploration._name)

        self._finish_shop_exploration.add_effect(
            effect=Effect(
                sensor_name=self.map_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=bool))

        self._finish_shop_exploration.add_precondition(
            precondition=at_shop_cond)  # only finish if we are at a charging station

        self._shop_exploration.add_precondition(
            precondition=Negation(at_shop_cond)
        )

        # Exploration goal
        self._exploration_goal = GoalBase(
            name='exploration_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self.map_discovered_everything_condition])