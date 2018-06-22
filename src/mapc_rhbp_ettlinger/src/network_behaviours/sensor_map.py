from behaviour_components.activators import ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition
from behaviour_components.sensors import AggregationSensor, Sensor
from behaviour_components.sensors import TopicSensor
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from sensor.movement import StepDistanceSensor, ClosestChargingStationSensor


class SensorAndConditionMap(object):

    def __init__(self, msg, agent_name):

        self.agent_name = agent_name
        self.agent_topic = AgentUtils.get_bridge_topic_agent(agent_name=agent_name)
        self.init_load_sensors()
        self.init_agent_sensors()
        self.init_battery_sensors(msg)

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

    def init_battery_sensors(self, msg, recharge_lower_bound_percentage=0.7, recharge_critical_bound_percentage=0.2):
        agent_recharge_upper_bound = msg.role.base_battery

        agent_recharge_lower_bound = agent_recharge_upper_bound * recharge_lower_bound_percentage
        self.agent_charge_critical = agent_recharge_upper_bound * recharge_critical_bound_percentage

        agent_topic = AgentUtils.get_bridge_topic_agent(self.agent_name)


        self.closest_charging_station_sensor = ClosestChargingStationSensor(
            name="closest_charging_station_sensor",
            agent_name=self.agent_name
        )

        # Sensor to check distance to charging station
        self._charging_station_step_sensor = StepDistanceSensor(
            name='charging_station_step_distance',
            position_sensor_1=self.agent_position_sensor,
            position_sensor_2=self.closest_charging_station_sensor,
            initial_value=10
        )


        # Sensor that checks if vehicles is charged at the moment
        self.charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')


        # charging required condition: When closer to lower bound -> higher activation
        self._require_charging_cond = Condition(
            sensor=self.charge_sensor,
            activator=LinearActivator(
                zeroActivationValue=agent_recharge_upper_bound,
                fullActivationValue=agent_recharge_lower_bound))  # highest activation already before battery empty

        # Condition to check if we are at a charging station
        self._at_charging_station_cond = Condition(
            sensor=self._charging_station_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # battery is completely empty
        self._battery_empty_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # CONDITION: Vehicle has enough charge to function
        self.enough_battery_to_move_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=1,
                isMinimum=True))

        # Movement decreases charge
        self.go_to_charging_station_effect = \
            Effect(
                sensor_name=self.charge_sensor.name,
                sensor_type=float,
                indicator=-1.0)

        # Recharging has effect on charge_behaviour sensor
        self.recharge_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=1.0,
            sensor_type=float)

        self._charge_behaviour_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=100,  # here we could also use the real charging effects
            # this is replaced immediately by the subscriber callback
            sensor_type=float)


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