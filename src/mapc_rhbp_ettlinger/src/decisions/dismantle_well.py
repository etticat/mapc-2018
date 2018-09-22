import rospy
from diagnostic_msgs.msg import KeyValue

from common_utils import etti_logging
from decisions.map_decisions import MapDecision
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.simulation_provider import SimulationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.map')


class ExistingOpponentWellsDecision(MapDecision):
    """
    Returns the closest cell with value 0 in the map.
    If no cell has value 0, it returns a random value that is among the 10% of oldest places.
    """

    def __init__(self, agent_name, buffer, frame, key, target_frames, mode=MapDecision.MODE_OLDEST_VISITED,
                 granularity=50,
                 value=None, state=None, moving=True,
                 static=False, diffusion=600, goal_radius=0.5,
                 ev_factor=0.0, ev_time=5, pick_random_of_lowest_values=False):

        self.pick_random_of_lowest_values = pick_random_of_lowest_values

        super(ExistingOpponentWellsDecision, self).__init__(buffer=buffer, frame=frame, key=key,
                                                            target_frames=target_frames, mode=mode,
                                                            granularity=granularity,
                                                            value=value,
                                                            state=state, moving=moving, static=static,
                                                            diffusion=diffusion,
                                                            goal_radius=goal_radius,
                                                            ev_factor=ev_factor, ev_time=ev_time, agent_name=agent_name)

        self._agent_name = agent_name

        self._distance_provider = DistanceProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._simulation_provider = SimulationProvider(agent_name=agent_name)

        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

    def calc_value(self):
        """
        Calculates the position
        :return:
        """

        # Get map from superclass
        super(ExistingOpponentWellsDecision, self).calc_value()

        existing_wells = []

        for well in self._facility_provider.opponent_wells.values():

            if self._simulation_provider.out_of_bounds(well.pos):
                continue

            x, y = self._distance_provider.position_to_xy(well.pos)
            simple_pos_x = int(x / self.granularity)
            simple_pos_y = int(y / self.granularity)
            well_last_seen = self._facility_provider.wells_seen_at_step[well.name]
            last_time_at_well_position = self.environment_array[simple_pos_x, simple_pos_y]

            rospy.loginfo(
                "ExistingOpponentWellsDecision(%s):: well_name: %s, well_last_seen: %d, last_time_at_well_position: %d",
                self._agent_name, well.name, well_last_seen, last_time_at_well_position)

            if well_last_seen >= last_time_at_well_position:
                existing_wells.append(well)

        # Either let every agent go to the same one
        # existing_wells.sort(key=lambda x: x.name)
        # well = existing_wells[0]
        # or let every agent go to the closest one
        well = self._distance_provider.get_closest_facility(existing_wells)
        return [well, self.state]

    def destination_not_found(self, pos):
        """
        When destination can not be reached, pick new one
        :return:
        """
        # Avoid this place until step 100000

        x, y = self._distance_provider.position_to_xy(pos)
        simple_pos_x = int(x / self.granularity)
        simple_pos_y = int(y / self.granularity)

        if self.environment_array is not None:
            self.environment_array[simple_pos_x, simple_pos_y] = 100000

        self._self_organisation_provider.send_msg(
            pos=(x,y), frame="no_route", parent_frame="agent", time=1000, payload=[
                KeyValue(key="lat", value=str(pos.lat)),
                KeyValue(key="long", value=str(pos.long))], diffusion=0.99)

        self.calc_value()