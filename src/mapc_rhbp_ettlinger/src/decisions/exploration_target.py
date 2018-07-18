from decisions.map_decisions import PickClosestDestinationWithLowestValue, MapDecision


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
