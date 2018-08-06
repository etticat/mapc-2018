import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg, ChargingStationMsg, ResourceMsg, SimEnd

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.facility')


class FacilityProvider(object):
    """
    Keeps a list of all Facilities readily available for various components to use
    """
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self.init_runtime_variables()


        # Reset variables when simulation ends
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd, self.init_runtime_variables)

        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)
        rospy.Subscriber("/workshop", WorkshopMsg, self.workshop_callback)
        rospy.Subscriber("/charging_station", ChargingStationMsg, self.charging_station_callback)
        rospy.Subscriber("/resource", ResourceMsg, self.resources_callback)

    def init_runtime_variables(self, sim_end=None):
        self.charging_stations = {}
        self.storages = {}
        self.workshops = {}
        self.resources = {}

    def storage_callback(self, storageMsg):
        """
        Updates the storage dict when new information comes from mac ros bridge
        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage

    def workshop_callback(self, workshopMsg):
        """
        Updates the workshop dict when new information comes from mac ros bridge
        :param workshopMsg:
        :type workshopMsg: WorkshopMsg
        :return:
        """
        for workshop in workshopMsg.facilities:
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

    def resources_callback(self, resource_msg):
        """
        Updates the resource dict when new information comes from mac ros bridge
        :param resource_msg:
        :type resource_msg: ResourceMsg
        :return:
        """
        for resource in resource_msg.facilities:
            self.resources[resource.name] = resource

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
