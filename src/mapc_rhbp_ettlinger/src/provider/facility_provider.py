import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg, ChargingStationMsg, ResourceMsg, SimEnd, RequestAction, \
    FacilityMsg

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton
from provider.simulation_provider import SimulationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.facility')


class FacilityProvider(object):
    """
    Keeps a list of all Facilities readily available for various components to use
    """
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self.init_runtime_variables()
        self.simulation_provider = SimulationProvider(agent_name=agent_name)

        # Reset variables when simulation ends
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd, self.init_runtime_variables)

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="request_action"), RequestAction, self.request_action_callback)
        rospy.Subscriber("/charging_station", ChargingStationMsg, self.charging_station_callback)
        rospy.Subscriber("/facilities", FacilityMsg, self.facilities_callback)

    def init_runtime_variables(self, sim_end=None):
        self.charging_stations = {}
        self.storages = {}
        self.workshops = {}
        self.resources = {}
        self.own_wells = {}
        self._opponent_wells = {}
        self._wells_seen_at_step = {}

    def request_action_callback(self, request_action):
        """
        Updates the storage and workshop dictionaries when new information comes from mac ros bridge
        :param request_action:
        :type request_action: RequestAction
        :return:
        """
        for storage in request_action.storages:
            self.storages[storage.name] = storage

        for workshop in request_action.workshops:
            self.workshops[workshop.name] = workshop

    def charging_station_callback(self, charging_station_msg):
        """
        Updates the charging_station dict when new information comes from mac ros bridge
        :param charging_station_msg:
        :type charging_station_msg: ChargingStationMsg
        :return:
        """
        for charging_station in charging_station_msg.facilities:
            self.charging_stations[charging_station.name] = charging_station

    def facilities_callback(self, facility_msg):
        """
        Updates the resource dict when new information comes from mac ros bridge
        :param facility_msg:
        :type facility_msg: FacilityMsg
        :return:
        """
        for resource in facility_msg.resources:
            self.resources[resource.name] = resource
        for well in facility_msg.wells:
            if well.team == self.simulation_provider.team:
                self.own_wells[well.name] = well
            else:
                self._opponent_wells[well.name] = well
            self._wells_seen_at_step[well.name] = facility_msg.step

    def get_storage_by_name(self, name):
        """
        Returns the requested storage object
        :param name:
        :return:
        """
        return self.storages[name]

    def get_storages(self):
        """
        returns all storages
        :return: dict
        """
        return self.storages

    def get_random_workshop(self):
        """
        Returns a random workshop
        :return: Workshop
        """
        values = self.workshops.values()
        if len(values) > 0:
            return random.choice(values)
        else:
            ettilog.logerr("FacilityProvider:: Could not pick workshops. None available")
            return None

    def get_charging_stations(self):
        """
        Returns all charging stations
        :return: dict
        """
        return self.charging_stations

    def get_resources(self):
        """
        Returns all known resources
        :return: dict
        """
        return self.resources

    @property
    def opponent_wells(self):
        return self._opponent_wells

    @property
    def wells_seen_at_step(self):
        return self._wells_seen_at_step
