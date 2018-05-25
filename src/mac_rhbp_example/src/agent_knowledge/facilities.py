#!/usr/bin/env python2

import rospy

from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_ros_bridge.msg import Resource, RequestAction, ResourceMsg


def get_resource_tuple_all():
    return ('resource', '*', '*', '*')

def get_resource_tuple_item(item_name):
    return ('resource', '*', '*', item_name)

def get_resource_tupele_lat_long_item(lat, long, item, amount):
    return ('resource', str(lat), str(long), item)

class FacilityKnowledgebase():

    def __init__(self):
        kb_name = "knowledgeBaseNode"
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = kb_name)

        self._pub_global_resource = rospy.Publisher('/resource_cache', ResourceMsg, queue_size=1, latch=True)


    def add_new_resource(self, resource):
        """

        :param resource: Resource node to be saved
        :type resource: Resource
        :return:
        """
        for item in resource.items:
            new = get_resource_tupele_lat_long_item(resource.pos.lat, resource.pos.long, item.name, item.amount)

            try:
                ret_value = self.__kb_client.update(
                    new, new, push_without_existing = True)
                success = True
            except Exception as e:
                rospy.logerr("add_new_task failed:\n%s", e)
                rospy.logerr("Retrying.")

    def save_facilities(self, msg):
        """

        :param msg:
        :type msg: RequestAction
        :return:
        """

        # Save all resource nodes in immediate parimeter
        for resource in msg.resources:
            self.add_new_resource(resource)
