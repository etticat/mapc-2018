#!/usr/bin/env python2

import rospy

from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_ros_bridge.msg import Resource, RequestAction, ResourceMsg, StorageMsg, Position, Item


class FacilityKnowledgebase(object):

    def __init__(self):
        kb_name = "knowledgeBaseNode"
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = kb_name)
        self.storages = {}

        self._pub_global_resource = rospy.Publisher('/resource_cache', ResourceMsg, queue_size=1, latch=True)
        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)

    @staticmethod
    def get_resource_tuple(item="*", name="*", lat="*", long="*"):
        return ('resource', item, name, str(lat), str(long))

    INDEX_RESOURCE_ITEM = 1
    INDEX_RESOURCE_NAME = 2
    INDEX_RESOURCE_LAT = 3
    INDEX_RESOURCE_LONG = 4

    def add_new_resource(self, resource):
        """

        :param resource: Resource node to be saved
        :type resource: Resource
        :return:
        """
        new = FacilityKnowledgebase.get_resource_tuple(
            lat=resource.pos.lat,
            long=resource.pos.long,
            item=resource.item.name,
            name=resource.name
        )

        try:
            ret_value = self.__kb_client.update(
                new, new, push_without_existing = True)
            success = True
        except Exception as e:
            rospy.logerr("add_new_task failed:\n%s", e)
            rospy.logerr("Retrying.")

    def get_resources(self, item):
        all = FacilityKnowledgebase.get_resource_tuple(item=item)
        res = []
        tuple_list = self.__kb_client.all(all)
        for resource in tuple_list:
            res.append(Resource(
                name=resource[self.INDEX_RESOURCE_NAME],
                pos=Position(
                    lat=resource[self.INDEX_RESOURCE_LAT],
                    long=resource[self.INDEX_RESOURCE_LONG]),
                item=Item(
                    name=resource[self.INDEX_RESOURCE_ITEM]
                )
            ))
        return res
    def get_resource(self, name):
        all = FacilityKnowledgebase.get_resource_tuple(name=name)
        res = None
        tuple_list = self.__kb_client.all(all) # TODO: REplace with peek
        if len(tuple_list) > 0:
            res.append(Resource(
                name=tuple_list[0][self.INDEX_RESOURCE_NAME],
                pos=Position(
                    lat=tuple_list[0][self.INDEX_RESOURCE_LAT],
                    long=tuple_list[0][self.INDEX_RESOURCE_LONG]),
                item=Item(
                    name=tuple_list[0][self.INDEX_RESOURCE_ITEM]
                )
            ))
        return res

    def save_facilities(self, msg):
        """

        :param msg:
        :type msg: RequestAction
        :return:
        """

        # Save all resource nodes in immediate parimeter
        for resource in msg.resources:
            self.add_new_resource(resource)

    def storage_callback(self, storageMsg):
        """

        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage