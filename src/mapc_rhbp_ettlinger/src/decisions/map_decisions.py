import numpy as np
import random
import traceback

from mac_ros_bridge.msg import SimEnd, SimStart

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils

from provider.distance_provider import DistanceProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.map')


class MapDecision(DecisionPattern):
    """
    Calculates values for each cell of the map
    these values can be either the time it was last visited or how often a place has beenvisited
    """

    MODE_OLDEST_VISITED = "oldest_visited"
    MODE_SEEN_COUNT = "seen_count"

    def __init__(self, agent_name, buffer, frame, key, target_frames, mode, granulariy=50, value=None, state=None, moving=True,
                 static=False, diffusion=600, goal_radius=0.5,
                 ev_factor=0.0, ev_time=5):
        """

        :param agent_name:
        :param buffer:
        :param frame:
        :param key:
        :param target_frames:
        :param mode:
        :param granulariy: how far the calculated points should be from another (in meters)
        :param value:
        :param state:
        :param moving:
        :param static:
        :param diffusion:
        :param goal_radius:
        :param ev_factor:
        :param ev_time:
        """
        self.simulation_provider = SimulationProvider(agent_name=agent_name)
        self.granulariy = granulariy
        self.mode = mode
        self.target_frames = target_frames
        super(MapDecision, self).__init__(buffer, frame, key, value, state,
                                          moving, static, goal_radius, ev_factor,
                                          ev_time, diffusion)

        self.distance_provider = DistanceProvider(agent_name=agent_name)
        self.environment_array = None
        self.last_messages = []

        # Reset variables when simulation ends
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart, self.sim_end_callback)

        buffer.register_listener(self.target_frames, self.process_so_message)


    def sim_end_callback(self, sim_end=None):
        self.environment_array = None
        self.value = None
        self.simple_size_x = None
        self.simple_size_y = None

    def calc_value(self):
        """
        Calculates the map
        :return: maximum number
        """

        if self.distance_provider.total_distance_x == 0 or self.distance_provider.total_distance_y == 0:
            rospy.logerr("MapDecision:: Could not create map. distance provider not initialized yet")
            return None


        self.simple_size_x = int(self.distance_provider.total_distance_x / self.granulariy)
        self.simple_size_y = int(self.distance_provider.total_distance_y / self.granulariy)

        # create the map if it has not existed yet
        if self.environment_array is None:
            self.init_environment_array()

            so_messages = self._buffer.agent_set(self.target_frames, include_own=True)
            # Go through all soMessages when initialising it for the first time
            for so_message in so_messages:
                # only take the ones into account that have not been processed yet.
                self.process_so_message(so_message)

        return [self.environment_array, self.state]

    def process_so_message(self, so_message):

        # If the map doesnt exist yet, return without doing anything
        if self.environment_array is None:
            return

        if so_message.diffusion > 0 and so_message.p.z not in self.last_messages:
            mask = self.generate_round_array_mask(self.simple_size_x, self.simple_size_y,
                                                  so_message.p.x / self.granulariy, so_message.p.y / self.granulariy,
                                                  so_message.diffusion / self.granulariy)
            if self.mode == MapDecision.MODE_SEEN_COUNT:
                self.environment_array[mask] += 1
            elif self.mode == MapDecision.MODE_OLDEST_VISITED:
                self.environment_array = np.maximum(mask * so_message.ev_stamp.secs, self.environment_array)
            else:
                rospy.logerr("MapDecision:: Invalid mode %s", str(self.mode))

    def init_environment_array(self):
        self.environment_array = np.zeros([self.simple_size_x, self.simple_size_y, ])

    def create_message(self, val):
        """
        Overwrite the message creation to inject the simulation timestamp
        :param val:
        :return:
        """
        msg = super(MapDecision, self).create_message(val)
        msg.ev_stamp.secs = self.simulation_provider._step

    def generate_round_array_mask(self, size_x, size_y, x, y, r):
        """
        Generates a round array_mask
        :param size_x: length of mask
        :param size_y: heiht of mask
        :param x: point x in mask where the circle center should be
        :param y: point y in mask where the circle center should be
        :param r: the radius of the circle
        :return:
        """

        y_og, x_og = np.ogrid[-x:size_x - x, -y:size_y - y]
        mask = x_og * x_og + y_og * y_og <= r * r
        return mask

class PickClosestDestinationWithLowestValue(MapDecision):
    """
    Returns the closest cell with value 0 in the map.
    If no cell has value 0, it returns a random value that is among the 10% of oldest palces.
    """

    def __init__(self, agent_name, buffer, frame, key, target_frames, mode=MapDecision.MODE_OLDEST_VISITED, granulariy=50,
                 value=None, state=None, moving=True,
                 static=False, diffusion=600, goal_radius=0.5,
                 ev_factor=0.0, ev_time=5, pick_random_of_lowest_values=False):

        super(PickClosestDestinationWithLowestValue, self).__init__(buffer=buffer, frame=frame, key=key, target_frames=target_frames, mode=mode,
                                                                    granulariy=granulariy,
                                                                    value=value,
                                                                    state=state, moving=moving, static=static,
                                                                    diffusion=diffusion,
                                                                    goal_radius=goal_radius,
                                                                    ev_factor=ev_factor, ev_time=ev_time, agent_name=agent_name)
        self.pick_random_of_lowest_values = pick_random_of_lowest_values

    def calc_value(self):
        """
        Calculates the position
        :return:
        """

        # Get map from superclass
        res = super(PickClosestDestinationWithLowestValue, self).calc_value()

        if res is None:
            ettilog.logerr("PickClosestDestinationWithLowestValue::Map could not be loaded")
            return None

        environment_array = res[0]

        simple_size_x, simple_size_y = environment_array.shape
        pos_x, pos_y = self.distance_provider.position_to_xy(self.distance_provider._agent_pos)
        simple_pos_x = int(pos_x / self.granulariy)
        simple_pos_y = int(pos_y / self.granulariy)
        vision = max(int(self.distance_provider._agent_vision / self.granulariy), 1)
        r = vision * 2

        min_val = np.inf
        size = np.size(environment_array)
        mask = None

        while min_val != 0:
            r = r + vision

            mask = self.generate_round_array_mask(simple_size_x, simple_size_y, simple_pos_x, simple_pos_y, r)

            min_val = min(environment_array[mask])

            if np.sum(mask > 0) == size:
                break

        array2 = np.zeros([simple_size_x, simple_size_y, ])
        if min_val == 0 or self.pick_random_of_lowest_values == False:
            array2[environment_array == min_val] = 1
        else:
            array2[environment_array < np.percentile(environment_array, 10)] = 1

        array2[mask == False] *= 0

        ii = np.where(array2 == 1)
        tuple_list = tuple(zip(*ii))
        res = random.choice(tuple_list)

        res = tuple([i * self.granulariy for i in res])

        destination = self.distance_provider.position_from_xy(res[0], res[1])

        return [destination, self.state]
