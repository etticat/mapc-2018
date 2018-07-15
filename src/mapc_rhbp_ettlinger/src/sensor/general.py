from behaviour_components.sensors import AggregationSensor
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.general')


class SubtractionSensor(AggregationSensor):
    """
    Sensor that takes two sensors, and calculates the first minus the second sensor value
    """

    def __init__(self, name, minuend_sensor, subtrahend_sensor, publish_aggregate=False, optional=False,
                 initial_value=None):
        super(SubtractionSensor, self).__init__(name, [minuend_sensor, subtrahend_sensor],
                                                publish_aggregate=publish_aggregate, optional=optional,
                                                initial_value=initial_value)

    def _aggregate(self, sensor_values):
        v1 = sensor_values[0]
        v2 = sensor_values[1]
        if v1 is None or v2 is None or isinstance(v1, str) or isinstance(v2, str):
            ettilog.logerr("SubtractionSensor(%s):: Errror processing values: %s, %s", self.name, str(v1), str(v2))
        return v1 - v2


class FactorSensor(AggregationSensor):
    """
    Sensor that takes two sensors and calculates one divided by the other sensor value
    """

    def __init__(self, name, dividend_sensor, divisor_sensor, publish_aggregate=False, optional=False,
                 initial_value=None):
        super(FactorSensor, self).__init__(name, [dividend_sensor, divisor_sensor], publish_aggregate=publish_aggregate,
                                           optional=optional, initial_value=initial_value)

    def _aggregate(self, sensor_values):
        return float(sensor_values[0]) / sensor_values[1]
