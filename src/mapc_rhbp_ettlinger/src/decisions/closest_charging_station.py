from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from so_data.patterns import DecisionPattern


class ClosestChargingStationDecision(DecisionPattern):
    """
    Calculates the closest charging station
    """

    def __init__(self, agent_name):

        self._last_calculated_agent_position = None

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

        super(ClosestChargingStationDecision, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        """
        Returns the closest charging station to the agent at any given point
        :return:
        """
        if self._last_calculated_agent_position != self._distance_provider.agent_pos:
            charging_stations = self._facility_provider.get_charging_stations().values()
            closest_facility = self._distance_provider.get_closest_facility(facilities=charging_stations)

            return [closest_facility, self.state]

        else:
            return [self.value, self.state]
