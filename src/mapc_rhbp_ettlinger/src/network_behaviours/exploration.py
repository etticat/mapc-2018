from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.exploration import ExplorationBehaviour, FinishExplorationBehaviour
from sensor.exploration import ResourceDiscoveryProgressSensor
from sensor.movement import DestinationDistanceSensor


class ExplorationNetworkBehaviour(NetworkBehaviour):
    def __init__(self, agent, msg, name, **kwargs):
        super(ExplorationNetworkBehaviour, self).__init__(name, **kwargs)

        proximity = msg.proximity

        self._shop_exploration = ExplorationBehaviour(
            agent_name=agent._agent_name,
            plannerPrefix = self.get_manager_prefix(),
            name = 'explore_shops'
        )

        self.resource_discovery_progress_sensor = ResourceDiscoveryProgressSensor(
            name="resource_discovery_progress_sensor",
            agent_name=agent._agent_name

        )

        self.all_resources_discovered_condition = Condition(
            sensor=self.resource_discovery_progress_sensor,
            activator=ThresholdActivator(
                thresholdValue=1.0,
                isMinimum=True))


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
            movement_behaviour_name=self._shop_exploration._name)

        self._finish_shop_exploration.add_effect(
            effect=Effect(
                sensor_name=self.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float))

        self._finish_shop_exploration.add_precondition(
            precondition=at_shop_cond)  # only finish if we are at a charging station

        self._shop_exploration.add_precondition(
            precondition=Negation(at_shop_cond)
        )

        # TODO: Do I need these #33-1
        # Seems to work without. With them I get errors but it continues to run normally
        # Exploration goal
        # self._exploration_goal = GoalBase(
        #     name='exploration_goal',
        #     permanent=True,
        #     plannerPrefix=self.get_manager_prefix(),
        #     conditions=[self.map_discovered_everything_condition])