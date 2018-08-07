import random

from decisions.map_decisions import PickClosestDestinationWithLowestValue, MapDecision
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern


class ExplorationDecision(PickClosestDestinationWithLowestValue):
    """
    Finds the point to explore next.
    sends out a message, so no one else tries to explore the place too
    """

    def __init__(self, buffer, agent_name):
        super(ExplorationDecision, self).__init__(buffer=buffer, mode=MapDecision.MODE_OLDEST_VISITED,
                                                  frame='exploration_goal', key='destination',
                                                  target_frames=["agent", "exploration_goal", "no_route"],
                                                  pick_random_of_lowest_values=True, agent_name=agent_name)

    def create_message(self, val):
        msg = super(PickClosestDestinationWithLowestValue, self).create_message(val)

        if msg is not None:
            # Inject the intended destination and use it instead of the current position for the message
            msg.p.x = val[0][0]
            msg.p.y = val[0][1]
            msg.diffusion = self.distance_provider._agent_vision

        return msg
    def destination_not_found(self):
        """
        When destinatino can not be reached, pick new one
        :return:
        """
        self.calc_value()


class ExploreCornersDecision(DecisionPattern):
    """
    Calculates values for each cell of the map
    these values can be either the time it was last visited or how often a place has beenvisited
    """

    MODE_OLDEST_VISITED = "oldest_visited"
    MODE_SEEN_COUNT = "seen_count"
    def __init__(self, buffer, agent_name):
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        super(ExploreCornersDecision, self).__init__(buffer=buffer)

    def calc_value(self):
        corners = self._simulation_provider.get_corners()

        # TODO: Go to neighbouring corner
        return [random.choice(corners), self.state]