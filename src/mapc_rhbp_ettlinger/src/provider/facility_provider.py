import random

import rospy
from mac_ros_bridge.msg import StorageMsg, WorkshopMsg

from common_utils import rhbp_logging
from common_utils.singleton import Singleton

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.provider.facility')

class FacilityProvider(object):
    __metaclass__ = Singleton
    """
    Keeps a list of all Facilities readily available for various components to use
    """
    def __init__(self):
        self.storages = {}
        self.workshops = {}

        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)
        rospy.Subscriber("/workshop", WorkshopMsg, self.workshop_callback)



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

    def get_storage_by_name(self, name):
        return self.storages[name]

    def get_random_storage(self):
        return random.choice(self.storages.values())
    def get_random_workshop(self):
        return random.choice(self.workshops.values())