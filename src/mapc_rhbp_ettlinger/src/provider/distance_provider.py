#!/usr/bin/env python2
import json
import socket
import traceback
import urllib2
from urllib2 import URLError

import math
import rospy
from mac_ros_bridge.msg import SimStart
from mapc_rhbp_ettlinger.srv import SetGraphhopperMap
from math import sin, cos, sqrt, atan2, radians

from common_utils import rhbp_logging
from common_utils.graphhopper import GraphhopperProcessHandler
from common_utils.singleton import Singleton

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.provider.distance')

class DistanceProvider(object):
    __metaclass__ = Singleton

    RADIUS_EARTH_METERS = 6373000.0 # Using same approximation as server
    GRAPHHOPPER_URL_REQUEST_TIMEOUT = 1.0
    GRAPHHOPPER_DEFAULT_PORT = 8989

    def __init__(self):
        self.cell_size = 1.0
        self.can_fly = ""
        self._cache = {}
        self._proximity = 0
        self._map = None
        self.graphhopper_port = DistanceProvider.GRAPHHOPPER_DEFAULT_PORT


    def callback_sim_start(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """
        self.cell_size = sim_start.cell_size
        self.can_fly = "drone" == sim_start.role.name
        self.speed = sim_start.role.base_speed # TODO: Update on upgrade
        self._proximity = sim_start.proximity

    def calculate_distance(self, startPosition, endPosition):
        if self.can_fly:
            return self.calculate_distance_air(startPosition, endPosition)
        else:
            ettilog.logerr("using graphhopper")
            try:
                return self.calculate_distance_street(startPosition, endPosition)
            except LookupError as e:
                ettilog.logwarn(e)
            except Exception as e:
                ettilog.logwarn("Graphhopper not started/responding. Distance for the drone used instead." + str(e))
                ettilog.logdebug(traceback.format_exc())

            return self.calculate_distance_air(startPosition, endPosition) * 2 # Fallback

    def calculate_steps(self, pos1, pos2):
        size_ = self.calculate_distance(pos1, pos2) / (self.speed * self.cell_size)
        return math.ceil(size_ / 1000)


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

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
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
            distance = self._request_street_distance(a,b)
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
            connection = urllib2.urlopen(request, timeout = DistanceProvider.GRAPHHOPPER_URL_REQUEST_TIMEOUT)
            response = connection.read().decode()
            parsed = json.loads(response)
            connection.close()
            distance = parsed['paths'][0]['distance']
            if distance:
                # ettilog.loginfo("Successful route to '%s'", request)
                return distance
            else:
                raise LookupError('Graphhopper: Route not available for:'+request)

        except URLError as e:
            raise Exception("Graphhopper: URL error message for "+request+" :: " + str(e))
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
            if self._map != map: # reset cache on new map
                self._map = map
                self.graphhopper_port = res.port
                self._cache = {}
        except rospy.ServiceException:
            ettilog.logerr("ROS service exception in set_map_service %s", traceback.format_exc())