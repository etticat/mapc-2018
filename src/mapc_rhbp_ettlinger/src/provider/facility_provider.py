import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg, ChargingStationMsg, ResourceMsg

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.facility')


class FacilityProvider(object):
    __metaclass__ = Singleton
    """
    Keeps a list of all Facilities readily available for various components to use
    """

    def __init__(self):
        self.charging_stations = {}
        self.storages = {}
        self.workshops = {}
        self.resources = {}

        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)
        rospy.Subscriber("/workshop", WorkshopMsg, self.workshop_callback)
        rospy.Subscriber("/charging_station", ChargingStationMsg, self.charging_station_callback)
        rospy.Subscriber("/resource", ResourceMsg, self.resources_callback)

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

    def resources_callback(self, resource_msg):
        """

        :param resource_msg:
        :type resource_msg: ResourceMsg
        :return:
        """
        for resource in resource_msg.facilities:
            self.resources[resource.name] = resource

    def get_storage_by_name(self, name):
        return self.storages[name]

    def get_storages(self):
        return self.storages

    def get_random_workshop(self):
        values = self.workshops.values()
        if len(values) > 0:
            return random.choice(values)
        else:
            ettilog.logerr("FacilityProvider:: Could not pick workshops. None available")
            return None

    def get_charging_stations(self):
        return self.charging_stations.values()

    def get_resources(self):
        return self.resources

    def get_all_stored_items(self):
        items = {}

        storages = self.storages.values()
        for storage in storages:
            for item in storage.items:
                items[item.name] = items.get(item.name, 0) + item.stored
        return items

    def items_in_storage(self, storage_name):
        items = self.storages[storage_name].items
        return CalcUtil.get_dict_from_items(items, attrs=["stored", "delivered"])