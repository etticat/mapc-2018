from agent_knowledge.local_knowledge_sensors import LocalKnowledgeFirstFactSensor
from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.gather')


class SmallestGatherableItemSensor(Sensor):

    def __init__(self, agent_name, name=None, optional=False, initial_value=100):
        super(SmallestGatherableItemSensor, self).__init__(name, optional, initial_value)

        self._facility_provider = FacilityProvider()
        self.product_provider = ProductProvider(agent_name=agent_name)

        self.smallest_item_volume = initial_value

    def sync(self):
        if self.smallest_item_volume == self._initial_value:
            resources = self._facility_provider.get_resources()
            gatherable_items = set([resource.item.name for resource in resources.values()])

            for item in gatherable_items:
                volume = self.product_provider.get_product_by_name(item).volume
                if volume < self.smallest_item_volume:
                    self.smallest_item_volume = volume


        self.update(self.smallest_item_volume)
        return super(SmallestGatherableItemSensor, self).sync()

class NextIngredientVolumeSensor(LocalKnowledgeFirstFactSensor):

    def __init__(self, name, agent_name):
        self._agent_name = agent_name
        self._product_provider = ProductProvider(agent_name=agent_name)
        super(NextIngredientVolumeSensor, self).__init__(name=name, index=TaskKnowledgeBase.INDEX_MOVEMENT_TASK, pattern=TaskKnowledgeBase.generate_tuple(
            agent_name=agent_name, type=TaskKnowledgeBase.TYPE_GATHERING),
            knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME)

    def _reduce_facts(self, facts):
        task = super(NextIngredientVolumeSensor, self)._reduce_facts(facts=facts)
        if task is not None:
            volume = self._product_provider.get_product_by_name(task).volume

        else:
            volume = 0
        ettilog.loginfo("NextIngredientVolumeSensor(%s):: Volume of next item: %d %s", self._agent_name, volume, str(task))
        return volume