#!/usr/bin/env python2
from __future__ import division # force floating point division when using plain /
import urllib2
from urllib2 import URLError
import socket
import json
import math
import subprocess
import shlex
import rospkg
import traceback
from fcntl import fcntl, F_GETFL, F_SETFL
from os import O_NONBLOCK, path, environ
import threading
import rospy
from mac_utils.srv import SetGraphhopperMap, SetGraphhopperMapResponse
from mac_ros_bridge.msg import Position

GRAPHHOPPER_DEFAULT_PORT = 8989


class PathPlanner(object):
    """
    Class for calculating the agent paths/required steps
    Street distances are calculated using a graphhopper server
    """

    DISTANCE_BETWEEN_CIRCLE_OF_LATITUDES = 111.3
    DISTANCE_BETWEEN_CIRCLE_OF_LONGITUDES = 71.5
    GRAPHHOPPER_URL_REQUEST_TIMEOUT = 1.0

    def __init__(self, role, cell_size, proximity):
        """
        :param role: agent role
        """
        self.role = role

        self._cell_size = cell_size

        self._cache = {}

        self._proximity = proximity

        self._map = None

        self.graphhopper_port = GRAPHHOPPER_DEFAULT_PORT

    def _request_street_distance(self, a, b):
        """"
        Calculate the shortest street route between two positions
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
            connection = urllib2.urlopen(request, timeout=PathPlanner.GRAPHHOPPER_URL_REQUEST_TIMEOUT)
            response = connection.read().decode()
            parsed = json.loads(response)
            connection.close()
            distance = parsed['paths'][0]['distance']
            if distance:
                # rospy.loginfo("Successful route to '%s'", request)
                return distance / 1000 # convert to km
            else:
                raise LookupError('Graphhopper: Route not available for:'+request)

        except URLError as e:
            raise Exception("Graphhopper: URL error message for "+request+" :: " + str(e))
        except socket.timeout as e:
            raise Exception("Graphhopper socket timeout for: " + str(request))

    def are_same_location(self, a, b):
        """
        Check if two positions are considered to be at the same location
        :param a: Position a
        :param b: Position b
        :return: True if at the same location
        """
        return self.air_distance(a,b) <= self._proximity

    def street_distance(self, a, b):

        key = str(a) + str(b)

        try:
            distance = self._cache[key]
        except KeyError:
            distance = self._request_street_distance(a,b)
            self._cache[key] = distance
            # rospy.logdebug("Cache size increased to %d", len(self._cache))
        return distance

    def air_distance(self, pos1, pos2):
        """
        Calculate the euclidean distance between two positions
        :param pos1: position 1
        :type pos1: Position
        :param pos2: position 2
        :type pos2: Position
        :return: euclidean distance in km
        """
        return math.sqrt(
            ((pos1.lat - pos2.lat) * PathPlanner.DISTANCE_BETWEEN_CIRCLE_OF_LATITUDES) ** 2 + (
                (pos1.long - pos2.long) * PathPlanner.DISTANCE_BETWEEN_CIRCLE_OF_LONGITUDES) ** 2)

    def distance(self, pos1, pos2):
        """
        Returning the distance calculated based on the agent role (air or street)
        :param pos1: 
        :param pos2: 
        :return: distance in km
        """
        if self._is_close(pos1.lat, pos2.lat) and self._is_close(pos1.long, pos2.long):
            return 0

        if self.role.name == "drone":
            return self.air_distance(pos1, pos2)
        else:
            try:
                return self.street_distance(pos1, pos2)
            except LookupError as e:
                rospy.logwarn(e)
            except Exception as e:
                rospy.logwarn("Graphhopper not started/responding. Distance for the drone used instead." + str(e))
                rospy.logdebug(traceback.format_exc())
                pass
            return self.air_distance(pos1, pos2)

    def _is_close(self, a, b, allowed_error=0.00001):
        return abs(a - b) <= allowed_error

    def number_of_required_steps_to_facility(self, start, destination):
        """
        Calculate the minimal required number of steps for a specific distance
        :param start: position 1
        :type start: Position
        :param destination: position 2
        :type destination: Position
        :return: number of steps rounded up to the next highest integer
        """
        return int(math.ceil(self.distance(start, destination) / float(self.role.speed * self._cell_size)))

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
            rospy.logerr("ROS service exception in set_map_service %s", traceback.format_exc())


class GraphhopperProcessHandler(object):
    """
    This class is a graphhopper process wrapper that starts and stops graphhopper services
    depending on the required map
    """

    # paths are relative to package graph
    GRAPHHOPPER_PATH = '/../../third-party/graphhopper'
    GRAPHHOPPER_CMD = 'graphhopper.sh web '

    MAP_PATH = '/data/osm/'

    MAP_EXTENSION = '.osm.pbf'

    MAP_SERVICE_NAME = 'set_graphhopper_map'

    class GraphHopperProcess(object):

        def __init__(self, map_name, port, process):
            self.map_name = map_name
            self.port = port
            self.process = process

    def __init__(self, maps=[], initial_port=GRAPHHOPPER_DEFAULT_PORT):
        """
        :param map: set a map that is loaded during start
        """
        rospack = rospkg.RosPack()

        self._package_path = rospack.get_path("mac_utils")

        self._processes = {}

        self._next_port = initial_port

        self._set_map_lock = threading.Lock()

        rospy.Service(GraphhopperProcessHandler.MAP_SERVICE_NAME, SetGraphhopperMap, self._set_map_callback)

        # prestarting maps if requested
        for map_name in maps:
            try:
                if map_name:
                    self._set_map(map_name)
            except Exception as e:
                rospy.logerr(e)

    def _set_map(self, map_name):
        """
        set new map, function will either return port of running instance or will create a new instance and then return its port
        :param map_name: map name without path and extension
        :return: graphopper server port
        """
        with self._set_map_lock:
            try:
                g_process = self._processes[map_name]
            except KeyError:
                port = self._next_port
                process = self.start_graphhopper_process(map_name, port)
                self._next_port += 1
                g_process = GraphhopperProcessHandler.GraphHopperProcess(map_name=map_name, port=port, process=process)

                rospy.loginfo("Map '%s' is available on port:%d", map_name, port)

                # store process in dict
                self._processes[map_name] = g_process
            return g_process.port

    def start_graphhopper_process(self, map_name, port):
        """
        start a graphhopper process child
        :param map_name:  map name without path and extension
        :return: process handle
        """
        map_file = self._package_path + GraphhopperProcessHandler.MAP_PATH + map_name + GraphhopperProcessHandler.MAP_EXTENSION
        if path.exists(map_file):

            rospy.loginfo('Creating Graphhopper process for map %s', map_file)

            cmd = self._package_path + GraphhopperProcessHandler.GRAPHHOPPER_PATH + r'/' + GraphhopperProcessHandler.GRAPHHOPPER_CMD + map_file

            # set port as environment variable
            env = environ.copy()
            env['JETTY_PORT'] = str(port)

            process = self._start_process(command=cmd,cwd=self._package_path + GraphhopperProcessHandler.GRAPHHOPPER_PATH, env=env)
            return process

        else:
            raise Exception('Mapfile ' + map_file + ' not available')

    def _set_map_callback(self, req):

        port = self._set_map(req.map)

        response = SetGraphhopperMapResponse()

        response.port = port

        return response

    def _start_process(self, command, cwd , env):
        """
        Execute a process command
        :param command: full command string
        :param cwd: working directory
        :param env: environment
        """
        cmd = shlex.split(command)

        p = subprocess.Popen(cmd, cwd=cwd, shell=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)

        flags = fcntl(p.stdout, F_GETFL)  # get current p.stdout flags
        fcntl(p.stdout, F_SETFL, flags | O_NONBLOCK)

        # waiting until server is up and running
        while True:
            try:
                res = p.stdout.readline()
            except Exception as e:
                if e.errno == 11:  # corresponds to "Resource temporarily unavailable" and indicates we do not have more data
                    continue
                else:
                    raise e
            if 'Started server at HTTP' in res:  # this is unfortunately language specific
                p.stdout.close()
                break

        return p

    def _check_state(self):
        for gp in self._processes.values():
            try:
                ret = gp.process.poll()
                if ret: # check if the process terminated
                    rospy.logerr('Graphhopper has terminated with ret:%d. Trying a restart...', ret)
                    gp.process = self.start_graphhopper_process(map_name=gp.map_name,port=gp.port)

            except Exception as e:
                if e.errno == 11:  # corresponds to "Resource temporarily unavailable" and indicates we do not have more data
                    break
                else:
                    rospy.logerr(e)

    def _stop_processes(self):

        for gp in self._processes.values():
            gp.process.terminate()
            gp.process.wait()

    def __del__(self):
        self._stop_processes()


if __name__ == '__main__':

    rospy.init_node(name='graphhopper', log_level=rospy.INFO)

    start_maps = rospy.get_param('~start_maps', '')
    initial_port = rospy.get_param('~initial_port', GRAPHHOPPER_DEFAULT_PORT)
    start_maps = start_maps.split(',')

    gh = GraphhopperProcessHandler(maps=start_maps, initial_port=initial_port)

    # this is used for stress test debugging
    # rospy.sleep(0.1)
    # planner = PathPlanner(role=Role(name='truck', speed=100))
    # planner.set_map('london')
    # rospy.sleep(0.1)

    r = rospy.Rate(1)  # 1hz
    # request_cnt=0
    while not rospy.is_shutdown():
        gh._check_state()
        # this is used for stress test debugging
        # start = Position(lat=51.4773216248, long=-0.194169998169)
        # destination=Position(lat=51.4842300415,long=-0.160270005465)
        # for i in range(100):
        #     try:
        #         planner._request_street_distance(start,destination)
        #         request_cnt+=1
        #     except Exception as e:
        #         print(request_cnt)
        #         print(e)
        r.sleep()
