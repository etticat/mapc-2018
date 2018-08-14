#!/usr/bin/env python2
import json
import math
import socket
import traceback
import urllib2
from math import sin, cos, sqrt, atan2, radians
from numpy import mean
from urllib2 import URLError

import numpy as np
import rospy
from mac_ros_bridge.msg import SimStart, Position, Agent, ShopMsg, DumpMsg, StorageMsg, WorkshopMsg, WellMsg, \
    ChargingStationMsg, SimEnd, ResourceMsg, FacilityMsg
from mapc_rhbp_ettlinger.srv import SetGraphhopperMap

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton
from graphhopper import GraphhopperProcessHandler
from provider.simulation_provider import SimulationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.distance')


class DistanceProvider(object):
    """
    Provider, that provides information on the distance and steps to targets
    """
    __metaclass__ = Singleton

    RADIUS_EARTH_METERS = 6373000.0  # Using same approximation as server
    GRAPHHOPPER_URL_REQUEST_TIMEOUT = 1.0
    GRAPHHOPPER_DEFAULT_PORT = 8989

    def __init__(self, agent_name):

        self._initialised = False
        self._agent_name = agent_name

        self.graphhopper_port = DistanceProvider.GRAPHHOPPER_DEFAULT_PORT
        self._cell_size = 0.01
        self._can_fly = False
        self._speed = 1
        self._proximity = 0.01
        self._agent_pos = None
        self._agent_vision = 300
        self._road_distance_cache = {}
        self._facility_positions = {}
        self._map = None

        self._facility = None
        self._in_facility = False

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart,
                         self.callback_sim_start)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd,
                         self.callback_sim_end)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="agent"), Agent,
                         self.callback_agent)

        rospy.Subscriber('/shop', ShopMsg, self.callback_facility)
        rospy.Subscriber('/charging_station', ChargingStationMsg, self.callback_facility)
        rospy.Subscriber('/dump', DumpMsg, self.callback_facility)
        rospy.Subscriber('/storage', StorageMsg, self.callback_facility)
        rospy.Subscriber('/workshop', WorkshopMsg, self.callback_facility)
        rospy.Subscriber('/facilities', FacilityMsg, self.callback_resource_well_facilities)

    def callback_facility(self, facility_msg):
        """
        Keeps track of all positions of facilities
        :param facilityMsg:
        :return:
        """

        for facility in facility_msg.facilities:
            self._facility_positions[facility.name] = facility.pos

    def callback_resource_well_facilities(self, facility_msg):
        """
        Keeps track of all positions of resource and well facilities
        :param facility_msg:
        :type facility_msg: FacilityMsg
        :return:
        """

        for facility in facility_msg.resources + facility_msg.wells:
            self._facility_positions[facility.name] = facility.pos

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """
        self._agent_pos = msg.pos
        self._facility = msg.facility
        self._in_facility = msg.in_facility
        self._agent_vision = msg.vision
        self._speed = msg.speed

    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """

        self._cell_size = sim_start.cell_size
        self._can_fly = "drone" == sim_start.role.name
        self._speed = sim_start.role.base_speed
        self._proximity = sim_start.proximity

        # The agent often gets stuck at the boarders, avoid this by adding a slight margin around the borders that we do not want to touch
        self.min_lat = sim_start.min_lat + SimulationProvider.COORDINATES_MARGIN
        self.max_lat = sim_start.max_lat - SimulationProvider.COORDINATES_MARGIN
        self.min_lon = sim_start.min_lon + SimulationProvider.COORDINATES_MARGIN
        self.max_lon = sim_start.max_lon - SimulationProvider.COORDINATES_MARGIN

        # Get the difference between max and min values
        self.lat_spread = self.max_lat - self.min_lat
        self.lon_spread = self.max_lon - self.min_lon

        # Calculate the difference between min and max lat at the mean longitude
        mean_long = mean([self.min_lon, self.max_lon])
        self.total_distance_x = self.calculate_distance_air(
            Position(lat=self.min_lat, long=mean_long),
            Position(lat=self.max_lat, long=mean_long)
        )

        # Calculate the difference between min and max long at the mean latitude
        mean_lat = mean([self.min_lat, self.max_lat])
        self.total_distance_y = self.calculate_distance_air(
            Position(long=self.min_lon, lat=mean_lat),
            Position(long=self.max_lon, lat=mean_lat)
        )
        self._initialised = True
        self.set_map(sim_start.map)

    def callback_sim_end(self, sim_end):
        self._initialised = False
        self._facility_positions.clear()

    def calculate_distance(self, endPosition, start_position=None, can_fly=None):
        """
        Calculates the distance between two position.
        Depending on the agent abilities, this returns either air or road distance
        :param startPosition:
        :param endPosition:
        :return:
        """
        if can_fly is None:
            can_fly = self._can_fly

        if start_position is None:
            start_position = self.agent_pos

        if can_fly:
            # If agent can fly, return air distance
            air_distance = self.calculate_distance_air(start_position, endPosition)
            return air_distance
        else:
            # if agent can't fly return road distance
            # try:
                return self.calculate_distance_street(start_position, endPosition)
            # except LookupError as e:
            #     ettilog.logerr(e)
            # except Exception as e:
            #     ettilog.logerr("Graphhopper not started/responding. Distance for the drone used instead." + str(e))
            #     ettilog.logerr(traceback.format_exc())

            # return self.calculate_distance_air(start_position, endPosition) * 2  # Fallback

    def calculate_steps(self, end_position, use_in_facility_flag=True, start_position=None, can_fly=None, speed=None):
        """
        Returns the step needed between two positions.
        :param can_fly:
        :param start_position:
        :param use_in_facility_flag:
        :param end_position:
        :return:
        """
        if self.at_destination(end_position, use_in_facility_flag, start_position):
            # If the distance is lower than proximity, agent is at destination.
            return 0

        if speed is None:
            speed = self._speed

        # calculate the air distance
        size_ = self.calculate_distance(end_position, start_position=start_position, can_fly=can_fly) / (speed * self._cell_size)

        # Round up to next int value.
        # Agents often walk 1.0001 times the distance they are supposed to go. Therefore we subtract 0.002 to make up for this case.
        # The exact step size is not of utmost importance. 1 Step off is fine
        return math.ceil((size_ / 1000) - 0.002)  # round up except when its really close
        # TODO Maype this can be better approximated using proximity. But then need to convert from latlong eucliedian to meters

    def calculate_distance_air(self, pos1, pos2):
        """
        Returns the distance between two positions.
        Logic extracted and constants from massim server massim.protocol.scenario.city.util.LocationUtil.java
        :param pos1:
        :param pos2:
        :return:
        """

        lat1 = radians(pos1.lat)
        lon1 = radians(pos1.long)
        lat2 = radians(pos2.lat)
        lon2 = radians(pos2.long)

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return DistanceProvider.RADIUS_EARTH_METERS * c

    def calculate_distance_street(self, a, b):
        """
        Returns the street distance. If it is cached, use this value. otherwise request it from graphhopper
        Partly taken from 2017
        :param a:
        :param b:
        :return:
        """

        key = str(a) + str(b)

        if key not in self._road_distance_cache.keys():
            self._road_distance_cache[key] = self._request_street_distance(a, b)
        return self._road_distance_cache[key]

    def _request_street_distance(self, position1, position2):
        """"
        Request street distance between two positions from graphhoppper
        Taken from 2017
        :param position1: position 1
        :type position1: Position
        :param position2: position 2
        :type position2: Position
        :return: shortest street distance in km
        """
        request = 'http://localhost:{}/route?instructions=false&calc_points=false&' \
                  'points_encoded=false&point={},{}&point={},{}'.format(self.graphhopper_port,
                                                                        position1.lat, position1.long, position2.lat,
                                                                        position2.long)
        try:
            connection = urllib2.urlopen(request, timeout=DistanceProvider.GRAPHHOPPER_URL_REQUEST_TIMEOUT)
            response = connection.read().decode()
            parsed = json.loads(response)
            connection.close()
            distance = parsed['paths'][0]['distance']
            if distance:
                return distance
            else:
                raise LookupError('Graphhopper: Route not available for:' + request)

        except URLError as e:
            raise Exception("Graphhopper: URL error message for " + request + " :: " + str(e))
        except socket.timeout as e:
            raise Exception("Graphhopper socket timeout for: " + str(request))

    def set_map(self, map):
        """
        Set the current map on the map server using the ROS service interface
        Taken from 2017
        :param map: name of map to configure
        """
        ettilog.logerr("DistanceProvider(%s):: Setting map %s", self._agent_name, map)
        try:
            set_map = rospy.ServiceProxy(GraphhopperProcessHandler.MAP_SERVICE_NAME, SetGraphhopperMap)
            res = set_map(map)
            if self._map != map:  # reset cache on new map
                self._map = map
                self.graphhopper_port = res.port
                self._road_distance_cache = {}
        except rospy.ServiceException:
            ettilog.logerr("ROS service exception in set_map_service %s", traceback.format_exc())

    def calculate_positions_eucledian_distance(self, pos1, pos2):
        """
        Taken from 2017
        :param pos1:
        :param pos2:
        :return:
        """
        return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)

    def get_closest_facility(self, facilities):
        """
        Returns the closest facility to the agent
        :param facilities:
        :param position:
        :return:
        """
        # If no facilities provided, return none
        if facilities is None or len(facilities) == 0:
            return None

        # Return closest facility
        return min(facilities, key=lambda facility: self.calculate_steps(facility.pos))

    def at_destination(self, destination_pos, use_in_facility_flag, start_position=None):
        """
        Returns if pos1 and pos2 are at same location as defined in server.
        :param destination_pos:
        :return:
        """

        if use_in_facility_flag and start_position is not None:
            if self._in_facility:
                if self._facility in self._facility_positions:
                    pos = self._facility_positions[self._facility]
                    destination_reached = pos.lat == destination_pos.lat and pos.long == destination_pos.long

                    rospy.loginfo("DistanceProvider(%s):: at facility %s, destination reached: %r", self._agent_name,
                                  self._facility, destination_reached)
                    if destination_reached:
                        return True
                else:
                    rospy.logerr("DistanceProvider(%s):: current faciliy unknown: %s", self._agent_name, self._facility)
        else:
            if start_position is None:
                start_position = self.agent_pos
            return self.same_location(start_position, destination_pos)

        return False

    def lat_to_x(self, lat):
        """
        Converts a latitude value to an x value that can be used in the self organisation framework
        :param lat:
        :return:
        """
        distance_from_left_side = lat - self.min_lat
        percentage_on_screen = distance_from_left_side / self.lat_spread

        x = percentage_on_screen * self.total_distance_x

        return x

    def lon_to_y(self, lon):
        """
        Converts a longitude value to a y value that can be used in the self organisation framework
        :param lon:
        :return:
        """
        distance_from_top = lon - self.min_lon
        percentage_on_screen = distance_from_top / self.lon_spread

        y = percentage_on_screen * self.total_distance_y

        return y

    def position_to_xy(self, pos):
        """
        Converts a Position object to an x ad y values that can be used in the self organisation framework
        :param pos:
        :type pos: Position
        :return:
        """
        return self.lat_to_x(pos.lat), self.lon_to_y(pos.long)

    def y_to_lon(self, y):
        """
        Converts a y value from the self organisation framework to a lon value that can be used for massim simulation
        :param y:
        :return:
        """
        percentage_on_screen = float(y) / self.total_distance_y

        distance_from_top = self.lon_spread * percentage_on_screen

        lon = distance_from_top + self.min_lon

        return max(min(lon, self.max_lon), self.min_lon)

    def x_to_lat(self, x):
        """
        Converts an x value from the self organisation framework to a long value that can be used for massim simulation
        :param x:
        :return:
        """
        percentage_on_screen = float(x) / self.total_distance_x

        distance_from_left = self.lat_spread * percentage_on_screen

        lat = distance_from_left + self.min_lat

        return max(min(lat, self.max_lat), self.min_lat)

    def position_from_xy(self, x, y):
        """
        Converts x and y values from the self organisation framework to a Position object for the massim simulation
        :param x:
        :param y:
        :return:
        """

        return Position(lat=self.x_to_lat(x), long=self.y_to_lon(y))

    @property
    def speed(self):
        return self._speed

    @property
    def agent_pos(self):
        return self._agent_pos

    @property
    def in_facility(self):
        return self._in_facility

    def same_location(self, pos1, pos2):
        return self.calculate_positions_eucledian_distance(pos1, pos2) < self._proximity

    @property
    def initialised(self):
        return self._initialised

    @property
    def agent_vision(self):
        return self._agent_vision

    def get_steps_to_closest_facility(self, pos_speed_role_list, facility_list):
        best_facility = None
        overall_max_step = np.inf

        for facility in facility_list:
            max_step_to_facility = 0

            for pos, speed, role in pos_speed_role_list:
                steps = self.calculate_steps(end_position=facility.pos, use_in_facility_flag=False, start_position=pos, can_fly=False)
                max_step_to_facility = max(max_step_to_facility, steps)

            if max_step_to_facility < overall_max_step:
                overall_max_step = max_step_to_facility
                best_facility = facility

        return overall_max_step, best_facility
