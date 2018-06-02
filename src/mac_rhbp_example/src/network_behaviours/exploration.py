import rospy

from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.resource import ResourceKnowledgebase
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import Sensor
from behaviours.exploration import ExplorationBehaviour, FinishExplorationBehaviour
from common_utils.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import DestinationDistanceSensor
from knowledge_base.knowledge_base_manager import KnowledgeBase
from knowledge_base.update_handler import KnowledgeBaseFactCache


class ResourceDiscoveryProgressSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """

    def __init__(self, agent_name, optional=False, name=None, initial_value=0.0):
        super(ResourceDiscoveryProgressSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)
        self._resource_knowledge = ResourceKnowledgebase()
        self.task_knowledge = TaskKnowledge()
        self._product_provider = ProductProvider(agent_name=agent_name)

    def sync(self):
        resources = self._resource_knowledge.get_resources_for_item(item="*")
        base_ingredients = self._product_provider.get_base_ingredients().keys()

        total_ingredients = len(base_ingredients)

        for resource in resources:
            if resource.item.name in base_ingredients:
                base_ingredients.remove(resource.item.name)

        discovered_ingredients = total_ingredients - len(base_ingredients)

        res = float(discovered_ingredients) / total_ingredients
        rospy.logerr("Discovery progress: %s", str(res))

        self.update(res)
        super(ResourceDiscoveryProgressSensor, self).sync()




class ExplorationBehaviourNetwork(NetworkBehaviour):
    def __init__(self, agent, msg, name, **kwargs):
        super(ExplorationBehaviourNetwork, self).__init__(name, **kwargs)

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