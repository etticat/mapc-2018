#!/usr/bin/env python2
import json
import math
import socket
import traceback
import urllib2
from math import sin, cos, sqrt, atan2, radians
from urllib2 import URLError

import rospy
from mac_ros_bridge.msg import SimStart
from mapc_rhbp_ettlinger.srv import SetGraphhopperMap

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton
from graphhopper import GraphhopperProcessHandler

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.distance')


class DistanceProvider(object):
    __metaclass__ = Singleton

    RADIUS_EARTH_METERS = 6373000.0  # Using same approximation as server
    GRAPHHOPPER_URL_REQUEST_TIMEOUT = 1.0
    GRAPHHOPPER_DEFAULT_PORT = 8989

    def __init__(self):
        self.graphhopper_port = DistanceProvider.GRAPHHOPPER_DEFAULT_PORT
        self.cell_size = 0.01
        self.can_fly = False
        self.speed = 1
        self._proximity = 0.01
        self.agent_pos = None


    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        self.agent_pos = msg.pos

    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self.cell_size = sim_start.cell_size
        self.can_fly = "drone" == sim_start.role.name
        self.speed = sim_start.role.base_speed  # TODO: Update on upgrade
        self._proximity = sim_start.proximity

    def calculate_distance(self, startPosition, endPosition):
        if self.can_fly:
            return self.calculate_distance_air(startPosition, endPosition)
        else:
            try:
                return self.calculate_distance_street(startPosition, endPosition)
            except LookupError as e:
                ettilog.logwarn(e)
            except Exception as e:
                ettilog.logwarn("Graphhopper not started/responding. Distance for the drone used instead." + str(e))
                ettilog.logdebug(traceback.format_exc())

            ettilog.logwarn("DistanceProvider:: Using fallback approximation")
            return self.calculate_distance_air(startPosition, endPosition) * 2  # Fallback

    def calculate_steps(self, pos1, pos2):
        if self.calculate_positions_eucledian_distance(pos1, pos2) < self._proximity:
            return 0
        size_ = self.calculate_distance(pos1, pos2) / (self.speed * self.cell_size)
        return math.ceil((size_ / 1000) - 0.002)  # round up except when its really close
        # TODO Maype this can be better approximated using proximity. But then need to conver from latlong eucliedian to meters

    def calculate_distance_air(self, pos1, pos2):
        """
        Logic extracted from massim server massim.protocol.scenario.city.util.LocationUtil.java
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
        Taken from 2017
        :param a:
        :param b:
        :return:
        """

        key = str(a) + str(b)

        try:
            distance = self._cache[key]
        except KeyError:
            distance = self._request_street_distance(a, b)
            self._cache[key] = distance
            # ettilog.logdebug("Cache size increased to %d", len(self._cache))
        return distance

    def _request_street_distance(self, a, b):
        """"
        Calculate the shortest street route between two positions
        Taken from 2017
        :param a: position 1
        :type a: Position
        :param b: position 2
        :type b: Position
        :return: shortest street distance in km
        """
        request = 'http://localhost:{}/route?instructions=false&calc_points=false&' \
                  'points_encoded=false&point={},{}&point={},{}'.format(self.graphhopper_port,
                                                                        a.lat, a.long, b.lat, b.long)
        try:
            connection = urllib2.urlopen(request, timeout=DistanceProvider.GRAPHHOPPER_URL_REQUEST_TIMEOUT)
            response = connection.read().decode()
            parsed = json.loads(response)
            connection.close()
            distance = parsed['paths'][0]['distance']
            if distance:
                # ettilog.loginfo("Successful route to '%s'", request)
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
        :param map: name of map to configure
        """
        try:
            set_map = rospy.ServiceProxy(GraphhopperProcessHandler.MAP_SERVICE_NAME, SetGraphhopperMap)
            res = set_map(map)
            if self._map != map:  # reset cache on new map
                self._map = map
                self.graphhopper_port = res.port
                self._cache = {}
        except rospy.ServiceException:
            ettilog.logerr("ROS service exception in set_map_service %s", traceback.format_exc())

    def calculate_positions_eucledian_distance(self, pos1, pos2):

        return math.sqrt((pos1.lat - pos2.lat) ** 2 + (pos1.long - pos2.long) ** 2)

    def get_closest_facility(self, facilities, agent_position=None):
        if agent_position is None:
            agent_position = self.agent_pos

        closest_facility = None
        closest_facility_steps = 99
        for facility in facilities:
            steps = self.calculate_steps(agent_position, facility.pos)
            if steps < closest_facility_steps:
                closest_facility_steps = steps
                closest_facility = facility
        return closest_facility

    def at_same_location(self, pos1, pos2):
        return AgentUtils.calculate_distance(pos1, pos2) < self._proximity


    def lat_to_x(self, lat):
        distance_from_left_side = lat - self.min_lat
        percentage_on_screen = distance_from_left_side / self.lat_spread

        x = percentage_on_screen * self.total_distance_x

        return x

    def lon_to_y(self, lon):
        distance_from_top = lon - self.min_lon
        percentage_on_screen = distance_from_top / self.lon_spread

        y = percentage_on_screen * self.total_distance_y

        return y

    def position_to_xy(self, pos):
        """

        :param pos:
        :type pos: Position
        :return:
        """
        return self.lat_to_x(pos.lat), self.lon_to_y(pos.long)

    def y_to_lon(self, y):
        percentage_on_screen = float(y) / self.total_distance_y

        distance_from_top = self.lon_spread * percentage_on_screen

        lon = distance_from_top + self.min_lon

        return max(min(lon, self.max_lon), self.min_lon)

    def x_to_lat(self, x):
        percentage_on_screen = float(x) / self.total_distance_x

        distance_from_left = self.lat_spread * percentage_on_screen

        lat = distance_from_left + self.min_lat

        return max(min(lat, self.max_lat), self.min_lat)

    def position_from_xy(self, x,y):

        return Position(lat=self.x_to_lat(x), long=self.y_to_lon(y))