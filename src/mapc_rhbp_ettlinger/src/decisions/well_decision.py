from decisions.map_decisions import PickClosestDestinationWithLowestValueDecision, MapDecision


class WellPositionDecision(PickClosestDestinationWithLowestValueDecision):
    """
    Selects the best position to build a well based on the place where the fewest opponents have been seen
    Sens a message so no one else builds a well there
    TODO: Prefer places that can only be reached by air
    """

    def __init__(self, buffer):
        super(WellPositionDecision, self).__init__(buffer, frame="build_well", key="destination",
                                                   target_frames=["build_well", "opponent", "no_route"],
                                                   mode=MapDecision.MODE_SEEN_COUNT)

    def create_message(self, val):
        """
        Do the calculation
        :param val:
        :return:
        """
        msg = super(PickClosestDestinationWithLowestValueDecision, self).create_message(val)

        if msg is not None:
            # Inject the intended destination and use it instead of the current position
            msg.p.x = val[0][0]
            msg.p.y = val[0][1]
            msg.diffusion = 20  # Dont build wells right next to each other

        return msg
