import numpy as np
import random
import traceback

import rospy
from common_utils import etti_logging

from provider.distance_provider import DistanceProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.self_organisation.decisions')


class MapDecision(DecisionPattern):
    MODE_OLDEST_VISITED = "oldest_visited"
    MODE_SEEN_COUNT = "seen_count"

    def __init__(self, buffer, frame, key, target_frames, mode, granulariy=500, value=None, state=None, moving=True,
                 static=False, diffusion=600, goal_radius=0.5,
                 ev_factor=0.0, ev_time=5):
        """

        :param buffer:
        :param frame:
        :param key:
        :param value:
        :param state:
        :param moving: As the goal changes sometimes
        :param static:
        :param diffusion: 600 TODO: This should always be the vision of agent
        :param goal_radius: We see everythong O.O TODO: consider this again
        :param ev_factor: 0.0 -> diffusion goes to 0 after ev_time
        :param ev_time: 5 -> after 10 steps the exploration goal is ignored by everyone
        """
        self.simulation_provider = SimulationProvider()
        self.granulariy = granulariy
        self.mode = mode
        self.target_frames = target_frames
        super(MapDecision, self).__init__(buffer, frame, key, value, state,
                                          moving, static, goal_radius, ev_factor,
                                          ev_time, diffusion)

        self.distance_provider = DistanceProvider()

    def calc_value(self):
        """
        determines maximum received value by all agent gradients
        :return: maximum number
        """

        sizex = self.distance_provider.total_distance_x
        sizey = self.distance_provider.total_distance_y

        if sizex == 0 or sizey == 0:
            rospy.logerr("MapDecision:: Could not create map. distance provider not initialized yet")
            return None

        so_messages = self._buffer.agent_set(self.target_frames, include_own=True)

        simple_size_x = int(sizex / self.granulariy)
        simple_size_y = int(sizey / self.granulariy)

        environment_array = np.zeros([simple_size_x, simple_size_y, ])

        # Calculate map
        for so_message in so_messages:
            if so_message.diffusion > 0:
                mask = self.generate_array_mask(simple_size_x, simple_size_y,
                                                so_message.p.x / self.granulariy, so_message.p.y / self.granulariy,
                                                so_message.diffusion / self.granulariy)
                if self.mode == MapDecision.MODE_SEEN_COUNT:
                    environment_array[mask] += 1
                elif self.mode == MapDecision.MODE_OLDEST_VISITED:
                    environment_array = np.maximum(mask * so_message.ev_stamp.secs, environment_array)
                else:
                    rospy.logerr("MapDecision:: Invalid mode %s", str(self.mode))

        # np.set_printoptions(linewidth=500, suppress=True)
        # rospy.logerr(str(np.flipud(environment_array)))

        return [environment_array, self.state]

    def create_message(self, val):

        msg = super(MapDecision, self).create_message(val)
        msg.ev_stamp.secs = self.simulation_provider.step

    def generate_array_mask(self, size_x, size_y, x, y, r):
        y, x = np.ogrid[-x:size_x - x, -y:size_y - y]
        mask = x * x + y * y <= r * r
        return mask

    def spread(self):
        """
        spreads message with maximum value
        :return:
        """
        try:
            super(MapDecision, self).spread()
        except Exception as e:
            print(e)
            print(traceback.format_exc())


class PickClosestDestinationWithLowestValue(MapDecision):

    def __init__(self, buffer, frame, key, target_frames, mode=MapDecision.MODE_OLDEST_VISITED, granulariy=500,
                 value=None, state=None, moving=True,
                 static=False, diffusion=600, goal_radius=0.5,
                 ev_factor=0.0, ev_time=5):

        super(PickClosestDestinationWithLowestValue, self).__init__(buffer, frame, key, target_frames, mode,
                                                                    granulariy=granulariy,
                                                                    value=value,
                                                                    state=state, moving=moving, static=static,
                                                                    diffusion=diffusion,
                                                                    goal_radius=goal_radius,
                                                                    ev_factor=ev_factor, ev_time=ev_time)

    def calc_value(self):

        res = super(PickClosestDestinationWithLowestValue, self).calc_value()

        if res is None:
            ettilog.logerr("PickClosestDestinationWithLowestValue::Map could not be loaded")
            return None

        environment_array = res[0]

        simple_size_x, simple_size_y = environment_array.shape

        pos_x, pos_y = self.distance_provider.position_to_xy(self.distance_provider.agent_pos)

        simple_pos_x = int(pos_x / self.granulariy)
        simple_pos_y = int(pos_y / self.granulariy)

        vision = max(int(self.distance_provider.agent_vision / self.granulariy), 1)

        r = vision * 2

        min_val = np.inf
        size = np.size(environment_array)

        mask = None

        while min_val != 0:
            r = r + vision

            mask = self.generate_array_mask(simple_size_x, simple_size_y, simple_pos_x, simple_pos_y, r)

            min_val = min(environment_array[mask])

            if np.sum(mask > 0) == size:
                break

        array2 = np.zeros([simple_size_x, simple_size_y, ])
        array2[environment_array == min_val] = 1
        array2[mask == False] *= 0

        ii = np.where(array2 == 1)
        tuple_list = tuple(zip(*ii))
        res = random.choice(tuple_list)

        res = tuple([i * self.granulariy for i in res])

        destination = self.distance_provider.position_from_xy(res[0], res[1])

        return [destination, self.state]


class ExplorationDecision(PickClosestDestinationWithLowestValue):

    def __init__(self, buffer):
        super(ExplorationDecision, self).__init__(buffer, mode=MapDecision.MODE_OLDEST_VISITED,
                                                  frame='exploration_goal', key='destination',
                                                  target_frames=["agent", "exploration_goal"])

    def create_message(self, val):
        msg = super(PickClosestDestinationWithLowestValue, self).create_message(val)

        if msg is not None:
            # Inject the intended destination and use it instead of the current position
            msg.p.x = val[0][0]
            msg.p.y = val[0][1]
            msg.diffusion = self.distance_provider.agent_vision

        return msg


class WellPositionDecision(PickClosestDestinationWithLowestValue):

    def __init__(self, buffer):
        super(WellPositionDecision, self).__init__(buffer, frame="build_well", key="destination", target_frames=["build_well", "opponent"],
                                                  mode=MapDecision.MODE_SEEN_COUNT)

    def create_message(self, val):
        msg = super(PickClosestDestinationWithLowestValue, self).create_message(val)

        if msg is not None:
            # Inject the intended destination and use it instead of the current position
            msg.p.x = val[0][0]
            msg.p.y = val[0][1]
            msg.diffusion = 200 # Dont build wells right next to each other
            # TODO: Use proximity here?

        return msg


class OldestCellAgeDecision(MapDecision):

    def __init__(self, buffer, init_value):
        super(OldestCellAgeDecision, self).__init__(buffer, frame='noneTODOremove', key='destination',
                                                    target_frames=["agent", "exploration_goal"],
                                                    mode=MapDecision.MODE_OLDEST_VISITED)
        self.init_value = init_value


    def calc_value(self):

        res = super(OldestCellAgeDecision, self).calc_value()

        if res is None:
            return [0.0, self.state]

        environment_array = res[0]

        min_val = np.min(environment_array)

        if min_val == 0:
            return [self.init_value, self.state]
        else:
            return min_val

class DiscoverProgressDecision(MapDecision):

    def __init__(self, buffer):
        super(DiscoverProgressDecision, self).__init__(buffer, frame='noneTODOremove', key='destination',
                                                  target_frames=["agent", "exploration_goal"],
                                                  mode=MapDecision.MODE_OLDEST_VISITED)

    def calc_value(self):

        res = super(DiscoverProgressDecision, self).calc_value()

        if res is None:
            return [0.0, self.state]

        environment_array = res[0]

        progress =  float(np.count_nonzero(environment_array)) / environment_array.size

        return [progress, self.state]