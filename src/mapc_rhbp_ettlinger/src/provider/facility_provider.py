import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg, ChargingStationMsg

from common_utils import rhbp_logging
from common_utils.singleton import Singleton

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.provider.facility')

class FacilityProvider(object):
    __metaclass__ = Singleton
    """
    Keeps a list of all Facilities readily available for various components to use
    """
    def __init__(self):
        self.charging_stations = {}
        self.storages = {}
        self.workshops = {}

        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)
        rospy.Subscriber("/workshop", WorkshopMsg, self.workshop_callback)
        rospy.Subscriber("/charging_station", ChargingStationMsg, self.charging_station_callback)



    def storage_callback(self, storageMsg):
        """

        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage
    def workshop_callback(self, workshopMsg):
        """

        :param workshopMsg:
        :type workshopMsg: WorkshopMsg
        :return:
        """
        for workshop in workshopMsg.facilities:
            self.workshops[workshop.name] = workshop
    def charging_station_callback(self, charging_station_msg):
        """

        :param charging_station_msg:
        :type charging_station_msg: ChargingStationMsg
        :return:
        """
        for charging_station in charging_station_msg.facilities:
            self.charging_stations[charging_station.name] = charging_station

    def get_storage_by_name(self, name):
        return self.storages[name]

    def get_storages(self):
        return self.storages

    def get_random_workshop(self):
        return random.choice(self.workshops.values())

    def get_charging_stations(self):
        return self.charging_stations.values()