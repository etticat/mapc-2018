import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg, ChargingStationMsg

from agent_knowledge.resource import ResourceBaseKnowledgeBase
from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.facility')


class FacilityProvider(object):
    __metaclass__ = Singleton
    """
    Keeps a list of all Facilities readily available for various components to use
    """

    def __init__(self):
        self._resource_knowledge = ResourceBaseKnowledgeBase()
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

    def get_resources(self):
        resources = self._resource_knowledge.get_resources_for_item(item="*")
        res = {}

        for resource in resources:
            res[resource.name] = resource

        return res

    def get_all_stored_items(self):
        items = {}

        storages = self.storages.values()
        for storage in storages:
            for item in storage.items:
                items[item.name] = items.get(item.name, 0) + item.stored
        return items
