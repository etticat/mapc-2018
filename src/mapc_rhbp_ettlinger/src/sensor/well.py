from provider.distance_provider import DistanceProvider
from provider.well_provider import WellProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR


class WellIntegritySensor(GradientSensor):
    """
    Sensor to check the integrity of the current target well
    """

    def __init__(self, mechanism, name, agent_name):
        super(WellIntegritySensor, self).__init__(sensor_type=SENSOR.VALUE, name=name, mechanism=mechanism)

        self._well_provider = WellProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

    def calc(self):
        """
        fetches the well and returns the Integrity as a factor of the total integrity
        :return:
        """
        val = super(WellIntegritySensor, self).calc()

        if val is not None:
            for well in self._well_provider.existing_wells.values():
                if self._distance_provider.at_destination(well.pos, val.pos):
                    well_prototype = self._well_provider.get_well(val.task)

                    return well.integrity / well_prototype.integrity
        # In case the well was not found or there is no target well, return 0
        return 0
