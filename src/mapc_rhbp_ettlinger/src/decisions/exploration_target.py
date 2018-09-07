import random

import numpy as np
import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import SimEnd, Position, SimStart

from common_utils.agent_utils import AgentUtils
from decisions.map_decisions import PickClosestDestinationWithLowestValueDecision, MapDecision
from provider.distance_provider import DistanceProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern


class ExplorationDecision(PickClosestDestinationWithLowestValueDecision):
    """
    Finds the point to explore next.
    sends out a message, so no one else tries to explore the place too
    """

    def __init__(self, buffer, agent_name):
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        super(ExplorationDecision, self).__init__(buffer=buffer, mode=MapDecision.MODE_OLDEST_VISITED,
                                                  frame='exploration_goal', key='destination',
                                                  target_frames=["agent", "exploration_goal", "no_route"],
                                                  pick_random_of_lowest_values=True, agent_name=agent_name)

    def create_message(self, val):
        msg = super(PickClosestDestinationWithLowestValueDecision, self).create_message(val)

        if msg is not None:
            # Inject the intended destination and use it instead of the current position for the message
            msg.p.x = val[0][0]
            msg.p.y = val[0][1]
            msg.diffusion = self.distance_provider.agent_vision
        return msg

    def destination_not_found(self, pos):
        """
        When destinatino can not be reached, pick new one
        :return:
        """
        # Avoid this place until step 100000

        x = int(round(self.distance_provider.lat_to_x(pos.lat)))
        y = int(round(self.distance_provider.lon_to_y(pos.long)))

        if self.environment_array is not None:
            self.environment_array[x / self.granulariy,y / self.granulariy] = 100000

        self._self_organisation_provider.send_msg(pos=pos, frame="no_route", parent_frame="agent",
                                                  time=100000, payload=[
                KeyValue(key="lat", value=str(pos.lat)),
                KeyValue(key="long", value=str(pos.long))], diffusion=0.01)

        self.calc_value()



class ExploreCornersDecision(DecisionPattern):
    """
    Calculates values for each cell of the map
    these values can be either the time it was last visited or how often a place has been visited
    """

    MODE_OLDEST_VISITED = "oldest_visited"
    MODE_SEEN_COUNT = "seen_count"

    def __init__(self, buffer, agent_name):
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)


        self.locations = []

        super(ExploreCornersDecision, self).__init__(buffer=buffer)

        self.current_pos = 10
        self._agent_name = agent_name

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd,
                         self.callback_sim_end)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart,
                         self.callback_sim_start)

    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self.locations = []
        min_lat = sim_start.min_lat + 0.004
        max_lat = sim_start.max_lat - 0.004
        max_lon = sim_start.max_lon - 0.004
        min_lon = sim_start.min_lon + 0.004

        lat_steps = np.linspace(min_lat, max_lat, int((max_lat - min_lat) / 0.02))
        lon_steps = np.linspace(min_lon, max_lon, int((max_lon - min_lon) / 0.02))

        for lat in lat_steps:
            self.locations.append(Position(lat=lat, long=min_lon))
        for lon in lon_steps:
            self.locations.append(Position(lat=max_lat, long=lon))
        for lat in reversed(lat_steps):
            self.locations.append(Position(lat=lat, long=max_lon))
        for lon in reversed(lon_steps):
            self.locations.append(Position(lat=min_lat, long=lon))
        self.current_pos = 10

        if "2" in self._agent_name or "4" in self._agent_name:
            self.locations = self.locations[::-1]
            self.current_pos = 40
        if "3" in self._agent_name or "4" in self._agent_name:
            self.current_pos = - self.current_pos


    def calc_value(self):

        if len(self.locations) == 0:
            return [None, self.state]

        if self._distance_provider.calculate_steps(self.locations[self.current_pos%len(self.locations)]) <= 1:
            self.current_pos = self.current_pos + 1

        return [self.locations[self.current_pos%len(self.locations)], self.state]

    def callback_sim_end(self, sim_end=None):
        self.value = None


    def destination_not_found(self, pos):
        pass
