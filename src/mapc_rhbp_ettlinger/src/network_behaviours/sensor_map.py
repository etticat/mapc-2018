from behaviour_components.activators import LinearActivator, ThresholdActivator
from behaviour_components.conditions import Condition
from behaviour_components.sensors import TopicSensor, AggregationSensor, Sensor
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider




class SensorAndConditionMap(object):

    def __init__(self, agent_name):

        self.agent_name = agent_name
        self.agent_topic = AgentUtils.get_bridge_topic_agent(agent_name=agent_name)
        self.init_load_sensors()
        self.init_agent_sensors()

    def init_load_sensors(self):

        self.max_load_sensor = TopicSensor(
            topic=self.agent_topic,
            name="max_load_sensor",
            message_attr='load_max')
        self.load_sensor = TopicSensor(
            topic=self.agent_topic,
            name="load_sensor",
            message_attr='load')

        self.free_load_sensor = SubtractionSensor(
            name="free_load_sensor",
            minuend_sensor=self.max_load_sensor,
            subtrahend_sensor=self.load_sensor
        )

        self.free_load_sensor = SubtractionSensor(
            name="free_load_sensor",
            minuend_sensor=self.max_load_sensor,
            subtrahend_sensor=self.load_sensor
        )
        self.load_factor_sensor = FactorSensor(
            name="free_load_sensor",
            dividend_sensor=self.load_sensor,
            divisor_sensor=self.max_load_sensor
        )

        self.load_fullnes_condition = Condition(
            name="load_fullnes_condition",
            sensor=self.load_factor_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            )
        )
        self.smallest_gatherable_item_sensor = SmallestGatherableItemSensor(
            name="smallest_gatherable_item_sensor",
            agent_name=self.agent_name
        )

        self.load_after_next_gathering_sensor = SubtractionSensor(
            name="load_after_next_gathering_sensor",
            minuend_sensor=self.free_load_sensor,
            subtrahend_sensor=self.smallest_gatherable_item_sensor,
        )

        self.can_fit_more_ingredients_cond = Condition(
            sensor=self.load_after_next_gathering_sensor,
            activator=ThresholdActivator(
                isMinimum=True,
                thresholdValue=-1
            )
        )

    def init_agent_sensors(self):
        self.agent_position_sensor = TopicSensor(
            topic=self.agent_topic,
            name="agent_position_sensor",
            message_attr='pos')



class SubtractionSensor(AggregationSensor):


    def __init__(self, name, minuend_sensor, subtrahend_sensor, publish_aggregate=False, optional=False, initial_value=None):

        super(SubtractionSensor, self).__init__(name, [minuend_sensor, subtrahend_sensor], publish_aggregate=publish_aggregate, optional=optional, initial_value=initial_value)

    def _aggregate(self, sensor_values):

        return float(sensor_values[0]) - sensor_values[1]

class FactorSensor(AggregationSensor):

    def __init__(self, name, dividend_sensor, divisor_sensor, publish_aggregate=False, optional=False, initial_value=None):

        super(FactorSensor, self).__init__(name, [dividend_sensor, divisor_sensor], publish_aggregate=publish_aggregate, optional=optional, initial_value=initial_value)

    def _aggregate(self, sensor_values):

        return float(sensor_values[0]) / sensor_values[1]


class SmallestGatherableItemSensor(Sensor):

    def __init__(self, agent_name, name=None, optional=False, initial_value=None):
        super(SmallestGatherableItemSensor, self).__init__(name, optional, initial_value)

        self._facility_provider = FacilityProvider()
        self.product_provider = ProductProvider(agent_name=agent_name)

    def sync(self):
        resources = self._facility_provider.get_resources()
        gatherable_items = set([resource.item.name for resource in resources.values()])

        smallest_item = 99

        for item in gatherable_items:
            volume = self.product_provider.get_product_by_name(item).volume
            if volume < smallest_item:
                smallest_item = volume

        return smallest_item