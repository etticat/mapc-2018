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

        self._facility_provider = FacilityProvider()
        self.product_provider = ProductProvider(agent_name=agent_name)

    def sync(self):
        smallest_item_volume = self._initial_value

        # only do the calculation once
        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        for item in gatherable_items:
            volume = self.product_provider.get_product_by_name(item).volume
            if volume < smallest_item_volume:
                smallest_item_volume = volume

        self.update(smallest_item_volume)
        return super(SmallestGatherableItemVolumeSensor, self).sync()


class NextGatheredItemVolumeSensor(GradientSensor):
    """
    Sensor that calculates the volume of the next item selected for gathering
    """

    def __init__(self, name, agent_name, mechanism):
        super(NextGatheredItemVolumeSensor, self).__init__(name=name, sensor_type=SENSOR.VALUE, mechanism=mechanism)

        self._agent_name = agent_name

        self._product_provider = ProductProvider(agent_name=agent_name)

    def calc(self):
        task = super(NextGatheredItemVolumeSensor, self).calc()
        if task is not None:
            volume = task.volume
        else:
            volume = 0
        ettilog.loginfo("NextIngredientVolumeSensor(%s):: Volume of next item: %d %s", self._agent_name, volume,
                        str(task))
        return volume
