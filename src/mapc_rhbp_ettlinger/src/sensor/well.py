from behaviour_components.sensors import Sensor
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR


class WellIntegritySensor(GradientSensor):
    """
    Sensor to check the integrity of the current target well
    """

    def __init__(self, mechanism, name, agent_name):
        super(WellIntegritySensor, self).__init__(sensor_type=SENSOR.VALUE, name=name, mechanism=mechanism)

        self._facility_provider = FacilityProvider(agent_name=agent_name)

        self._well_provider = WellProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

    def calc(self):
        """
        fetches the well and returns the Integrity as a factor of the total integrity
        :return:
        """
        val = super(WellIntegritySensor, self).calc()

        if val is not None:
            for well in self._facility_provider.own_wells.values():
                if self._distance_provider.same_location(well.pos, val.pos):
                    well_prototype = self._well_provider.get_well(val.task)

                    return well.integrity / well_prototype.integrity
        # In case the well was not found or there is no target well, return 0
        return 0


class EnoughMassiumToBuildWellSensor(Sensor):

    def __init__(self, well_choser_mechanism, name=None, optional=False, initial_value=None):
        self._well_choser_mechanism = well_choser_mechanism

        super(EnoughMassiumToBuildWellSensor, self).__init__(name, optional, initial_value)

    def sync(self):
        value = self._well_choser_mechanism.choose_well_type() is not None

        self.update(value)
        return super(EnoughMassiumToBuildWellSensor, self).sync()