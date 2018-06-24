#!/usr/bin/env python2
from knowledge_base.knowledge_base_manager import KnowledgeBase
from mac_ros_bridge.msg import Resource, Position, Item

from agent_knowledge.base_knowledge import BaseKnowledgeBase
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledge_base.resource')


class ResourceBaseKnowledgeBase(BaseKnowledgeBase):
    """
    Keeps a list of all Facilities
    """

    KNOWLEDGE_BASE_NAME = KnowledgeBase.DEFAULT_NAME

    INDEX_RESOURCE_ITEM = 1
    INDEX_RESOURCE_NAME = 2
    INDEX_RESOURCE_LAT = 3
    INDEX_RESOURCE_LONG = 4

    def __init__(self):
        super(ResourceBaseKnowledgeBase, self).__init__(
            knowledge_base_name=ResourceBaseKnowledgeBase.KNOWLEDGE_BASE_NAME)

    @staticmethod
    def get_resource_tuple(item="*", name="*", lat="*", lon="*"):
        return 'resource', item, name, str(lat), str(lon)

    @staticmethod
    def generate_resource_from_fact(fact):

        return Resource(
            name=fact[ResourceBaseKnowledgeBase.INDEX_RESOURCE_NAME],
            pos=Position(
                lat=float(fact[ResourceBaseKnowledgeBase.INDEX_RESOURCE_LAT]),
                long=float(fact[ResourceBaseKnowledgeBase.INDEX_RESOURCE_LONG])),
            item=Item(
                name=fact[ResourceBaseKnowledgeBase.INDEX_RESOURCE_ITEM]))

    @staticmethod
    def generate_fact_from_resource(resource):
        """
        Generates a fact from a resource
        :param resource: The resource
        :type resource: Resource
        :return: list
        """
        return ResourceBaseKnowledgeBase.get_resource_tuple(
            name=resource.name,
            item=resource.item.name,
            lat=resource.pos.lat,
            lon=resource.pos.long
        )

    def add_new_resource(self, resource):
        """

        :param resource: Resource node to be saved
        :type resource: Resource
        :return:
        """
        new = ResourceBaseKnowledgeBase.generate_fact_from_resource(resource)
        self._kb_client.update(new, new, push_without_existing=True)

    def get_resources_for_item(self, item):
        """
        Returns all discovered resources for a given item
        :param item: The item
        :type item: str
        :return: Resource[]
        """
        all = ResourceBaseKnowledgeBase.get_resource_tuple(item=item)
        res = []
        tuple_list = self._kb_client.all(all)
        for resource in tuple_list:
            r = self.generate_resource_from_fact(resource)
            res.append(r)
        return res

    def get_resources_for_items(self, items):
        """
        Returns all discovered resources for a given item
        :param items:
        :type item: str
        :return: Resource[]
        """
        all_tuple = ResourceBaseKnowledgeBase.get_resource_tuple()
        res = []
        tuple_list = self._kb_client.all(all_tuple)
        for resource in tuple_list:
            r = self.generate_resource_from_fact(resource)
            if r.item.name in items:
                res.append(r)
        return res

    def get_resource_by_name(self, name):
        """
        Returns the resource identified by name
        :param name: The name of the resource
        :type name: str
        :return: Resource
        """
        all_tuple = ResourceBaseKnowledgeBase.get_resource_tuple(name=name)
        fact = self._kb_client.peek(all_tuple)
        if fact is not None:
            return self.generate_resource_from_fact(fact)
        else:
            return None
