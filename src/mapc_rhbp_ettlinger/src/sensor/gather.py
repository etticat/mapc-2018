from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.gather')


class SmallestGatherableItemVolumeSensor(Sensor):
    """
    Calculates the volume of the smallest gatherable item
    """

    def __init__(self, agent_name, name=None, optional=False, initial_value=100):
        super(SmallestGatherableItemVolumeSensor, self).__init__(name, optional, initial_value)

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self.product_provider = ProductProvider(agent_name=agent_name)

    def sync(self):
        smallest_item_volume = self._initial_value

        # only do the calculation once
        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        for item in gatherable_items:
            item = self.product_provider.get_product_by_name(item)
            if item is None:
                continue
            volume = item.volume
            if volume < smallest_item_volume:
                smallest_item_volume = volume

        self.update(smallest_item_volume)
        return super(SmallestGatherableItemVolumeSensor, self).sync()
