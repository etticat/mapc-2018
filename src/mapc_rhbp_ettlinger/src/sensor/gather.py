from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
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

class NextVolumeSensor(GradientSensor):

    def __init__(self, name, agent_name, mechanism):
        self._agent_name = agent_name
        self._product_provider = ProductProvider(agent_name=agent_name)
        super(NextVolumeSensor, self).__init__(name=name, sensor_type=SENSOR.VALUE, mechanism=mechanism)

    def calc(self):
        task = super(NextVolumeSensor, self).calc()
        if task is not None:
            volume = task.volume
        else:
            volume = 0
        ettilog.loginfo("NextIngredientVolumeSensor(%s):: Volume of next item: %d %s", self._agent_name, volume, str(task))
        return volume