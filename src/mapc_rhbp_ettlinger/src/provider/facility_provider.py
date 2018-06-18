import random

import rospy
from mac_ros_bridge.msg import StorageMsg


class FacilityProvider(object):
    """
    Keeps a list of all Facilities readily available for various components to use
    """
    def __init__(self):
        self.storages = {}

        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)



    def storage_callback(self, storageMsg):
        """

        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage

    def get_storage_by_name(self, name):
        return self.storages[name]

    def get_random_storage(self):
        return random.choice(self.storages.values())