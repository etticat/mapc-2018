from behaviour_components.sensors import AggregationSensor
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.general')

class SubtractionSensor(AggregationSensor):

    def __init__(self, name, minuend_sensor, subtrahend_sensor, publish_aggregate=False, optional=False,
                 initial_value=None):
        super(SubtractionSensor, self).__init__(name, [minuend_sensor, subtrahend_sensor],
                                                publish_aggregate=publish_aggregate, optional=optional,
                                                initial_value=initial_value)

    def _aggregate(self, sensor_values):
        return float(sensor_values[0]) - sensor_values[1]


class FactorSensor(AggregationSensor):

    def __init__(self, name, dividend_sensor, divisor_sensor, publish_aggregate=False, optional=False,
                 initial_value=None):
        super(FactorSensor, self).__init__(name, [dividend_sensor, divisor_sensor], publish_aggregate=publish_aggregate,
                                           optional=optional, initial_value=initial_value)

    def _aggregate(self, sensor_values):
        return float(sensor_values[0]) / sensor_values[1]
