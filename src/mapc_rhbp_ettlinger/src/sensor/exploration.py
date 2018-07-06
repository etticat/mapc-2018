from agent_knowledge.resource import ResourceBaseKnowledgeBase
from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from provider.product_provider import ProductProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from self_organisation.decisions import DiscoverProgressDecision, OldestCellAgeDecision

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.exploration')


class ResourceDiscoveryProgressSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """

    def __init__(self, agent_name, optional=False, name=None, initial_value=0.0):
        super(ResourceDiscoveryProgressSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)
        self._resource_knowledge = ResourceBaseKnowledgeBase()
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
        ettilog.loginfo("%s:: Discovery progress: %s", self.name, str(res))

        self.update(res)
        super(ResourceDiscoveryProgressSensor, self).sync()


class DiscoveryProgressSensor(GradientSensor):

    def __init__(self, agent_name, name, thread=False, time=5, initial_value=None, sensor_type=SENSOR.VALUE):

        self._task_knowledge_base  = TaskKnowledgeBase()

        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.discovery_progress_decision = DiscoverProgressDecision(self.self_organisation_provider.buffer)

        super(DiscoveryProgressSensor, self).__init__(name, self.discovery_progress_decision, thread, time, initial_value, sensor_type)

class OldestCellLastSeenSensor(GradientSensor):

    def __init__(self, agent_name, name, initial_value, thread=False, time=5, sensor_type=SENSOR.VALUE):

        self._task_knowledge_base  = TaskKnowledgeBase()

        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.discovery_progress_decision = OldestCellAgeDecision(self.self_organisation_provider.buffer, init_value=initial_value)

        super(OldestCellLastSeenSensor, self).__init__(name, self.discovery_progress_decision, thread, time, initial_value, sensor_type)