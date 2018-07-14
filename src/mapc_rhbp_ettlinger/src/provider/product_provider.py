#!/usr/bin/env python2
from diagnostic_msgs.msg import KeyValue

import rospy
from mac_ros_bridge.msg import SimStart, Agent, StorageMsg
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton
from provider.facility_provider import FacilityProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.product')


class ProductProvider(object):
    __metaclass__ = Singleton

    STOCK_ITEM_TOPIC = "/stockitem/status"

    STOCK_ITEM_TYPE_STOCK = "stock"
    STOCK_ITEM_TYPE_STORAGE_STOCK = "stock"
    STOCK_ITEM_TYPE_GATHER_GOAL = "gather"
    STOCK_ITEM_TYPE_ASSEMBLY_GOAL = "assembly"
    STOCK_ITEM_TYPE_DELIVERY_GOAL = "delivery"
    STOCK_ITEM_TYPE_WELL_GOAL = "well"
    STOCK_ITEM_ALL_AGENT_TYPES = [STOCK_ITEM_TYPE_STOCK, STOCK_ITEM_TYPE_GATHER_GOAL, STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
                                  STOCK_ITEM_TYPE_DELIVERY_GOAL]

    def __init__(self, agent_name):

        self.products = {}
        self.base_ingredients = {}
        self.finished_products = {}
        self._items_in_stock = {}
        self.agent_stock_items = {
            ProductProvider.STOCK_ITEM_TYPE_STOCK: {},
            ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL: {}
        }
        self.storage_stock_items = {
            ProductProvider.STOCK_ITEM_TYPE_STOCK: {},
            ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL: {}
        }
        self.massim = {
            ProductProvider.STOCK_ITEM_TYPE_WELL_GOAL: {},
        }

        self._agent_name = agent_name
        self.storages = {}

        self.load_max = 0
        self.load_free = 0
        self.load = 0
        self.role = None
        self.useful_items_for_assembly = []

        rospy.Subscriber(
            AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start",
            SimStart,
            self._callback_sim_start)

        self._pub_stock_item = rospy.Publisher(ProductProvider.STOCK_ITEM_TOPIC, StockItem, queue_size=10)
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)
        self._sub_stock_item = rospy.Subscriber(ProductProvider.STOCK_ITEM_TOPIC, StockItem, self._callback_stock_item)
        rospy.Subscriber("/storage", StorageMsg, self.storage_callback)
        self._facility_provider = FacilityProvider()


    def storage_callback(self, storageMsg):
        """

        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage

    def _callback_stock_item(self, stock_item):
        """

        :param stock_item:
        :param msg: StockItem
        :return:
        """

        item_dict = CalcUtil.dict_from_key_int_values(stock_item.amounts)

        if stock_item.entity[0] == "a":
            self.agent_stock_items[stock_item.type][stock_item.entity] = item_dict
        elif stock_item.entity[0] == "s":
            if stock_item.type not in self.storage_stock_items:
                self.storage_stock_items[stock_item.type] = {}
            self.storage_stock_items[stock_item.type][stock_item.entity] = item_dict


    def get_stored_items(self, storage_name=None, include_job_goals=True, include_stock=True):
        items = {}

        for storage in self.storages.values():
            if storage_name == None or storage == storage_name:
                if include_stock:
                    for item in storage.items:
                        items[item.name] = items.get(item.name, 0) + item.stored
                if include_job_goals:
                    for job_dict in self.storage_stock_items.values():
                        if storage in job_dict:
                            items = CalcUtil.dict_sum(items, job_dict[storage])

        return items

    def _callback_agent(self, msg):
        """
        Retrieves the current state of stock items for use in other methods
        :param msg:
        :type msg: Agent
        :return:
        """
        items_in_stock_new = {}

        for item in self.products.keys():
            items_in_stock_new[item] = 0

        for item in msg.items:
            items_in_stock_new[item.name] = items_in_stock_new.get(item.name, 0) + item.amount

        if items_in_stock_new != self._items_in_stock:
            self._items_in_stock = items_in_stock_new
            self._pub_stock_item.publish(StockItem(
                entity=self._agent_name,
                type=ProductProvider.STOCK_ITEM_TYPE_STOCK,
                amounts=CalcUtil.key_int_values_from_dict(items_in_stock_new)
            ))

        self.load_max = msg.load_max
        self.load = msg.load
        self.load_free = msg.load_max - msg.load

    def _callback_sim_start(self, msg):
        """
        Retrieves the products of the current simulation and their details
        :param msg:
        :type msg: SimStart
        :return:
        """
        self.role = msg.role.name
        self.useful_items_for_assembly = []

        for product in msg.products:
            self.products[product.name] = product
            if len(product.consumed_items) > 0:
                self.finished_products[product.name] = product

                if self.role in product.required_roles:
                    for ingredient in product.consumed_items:
                        self.useful_items_for_assembly.append(ingredient.name)
            else:
                self.base_ingredients[product.name] = product

    def get_product_by_name(self, product):
        """
        Returns the product by name
        :param product:
        :return: Product
        """
        return self.products[product]

    def get_base_ingredients(self):
        return self.base_ingredients

    def get_finished_products(self):
        return self.finished_products

    def update_gathering_goal(self, item_to_focus):

        load_free = self.load_max - self.load

        items_to_carry = load_free / self.get_product_by_name(item_to_focus).volume

        gather_goal = {
            item_to_focus: items_to_carry
        }
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(gather_goal)
        ))
        return items_to_carry

    def remove_gathering_goal(self):
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL,
            amounts={}
        ))

    def update_assembly_goal(self, task_string, step=0):

        items_to_assemble = {}
        tasks = task_string.split(",")
        for i in range(step, len(tasks)):
            task = tasks[i].split(":")
            if task[0] == "assemble":
                product_name = task[1]
                items_to_assemble[product_name] = items_to_assemble.get(product_name, 0) + 1

                # Subtract the items that we will use for assembly.
                # Its ok if the goal is in this agent instead of the correct one
                # TODO: If finished product is also an ingredient -> When delivery overwrites assembly task of other agent
                # TODO: This might lead to them giving up the item -> Assembly will not be possible
                # TODO: Figure out a solution for this cas. e.g. just cancel assembly?
                items_to_assemble = CalcUtil.dict_sum(items_to_assemble, self.get_ingredients_of_product(product_name=product_name, amount=-1))


        self._assemble_goal = items_to_assemble
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(items_to_assemble)
        ))

    def stop_assembly(self):
        self._assemble_goal = {}
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
            amounts={}
        ))

    def update_delivery_goal(self, item_list, job_id, storage):

        items_to_assemble = {}
        item_dict = CalcUtil.dict_from_strings(item_list)
        storage_items = CalcUtil.dict_diff(item_dict, self._items_in_stock, normalize_to_zero=0)
        item_dict = CalcUtil.negate_dict(item_dict)
        storage_items = CalcUtil.negate_dict(storage_items)

        self._assemble_goal = items_to_assemble
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(item_dict)
        ))
        self._pub_stock_item.publish(StockItem(
            entity=storage,
            type=job_id,
            amounts=CalcUtil.key_int_values_from_dict(item_dict)
        ))

    def stop_delivery(self, job_id, storage):
        self._assemble_goal = {}
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL,
            amounts={}
        ))
        self._pub_stock_item.publish(StockItem(
            entity=storage,
            type=job_id,
            amounts={}
        ))

    def get_base_ingredients_in_stock(self):
        res = {}
        for item in self.base_ingredients.keys():
            res[item] = self._items_in_stock.get(item, 0)

        return res

    def get_finished_products_in_stock(self):
        res = {}
        for item in self.finished_products.keys():
            res[item] = self._items_in_stock.get(item, 0)

        return res

    def get_planned_ingredients(self):
        """
        Returns all ingredients that an agent needs to gather to meet its goal
        :return: list
        """
        res = []

        for index in self.base_ingredients:
            needed_amount = self._gather_goal.get(index, 0) - self._items_in_stock.get(index, 0)

            if needed_amount > 0:
                res.append(index)

        ettilog.loginfo("All ingredients to gather: %s", str(res))
        return res

    def get_planned_finished_products(self):
        """
        Returns all finished products, that an agent needs to assemble to meet its goal
        :return: list
        """
        res = []

        for index in self.finished_products:
            needed_amount = self._assemble_goal.get(index, 0) - self._items_in_stock.get(index, 0)

            if needed_amount > 0:
                res.append(index)

        return res

    def calculate_total_volume_dict(self, product_dict):
        """
        Calculates the total volume that a dict of products will use
        :param product_dict:
        :return:
        """

        v = 0

        for product in product_dict.keys():
            v += self.get_volume_of_item(product) * product_dict[product]

        return v

    def calculate_total_volume_list(self, product_list):
        """
        Calculates the total volume that a dict of products will use
        :param product_list:
        :return:
        """

        v = 0

        for product in product_list:
            v += self.get_volume_of_item(product)

        return v

    def total_items_in_stock(self, types):
        res = {}
        for type in types:
            for stock_item_dicts in self.agent_stock_items[type].values():
                res = CalcUtil.dict_sum(res, stock_item_dicts)
        return res

    def get_ingredients_of_product(self, product_name, amount=1):
        product = self.products[product_name]
        res = {}
        for ingredient in product.consumed_items:
            res[ingredient.name] = ingredient.amount * amount

        return res

    def get_roles_of_product(self, product_name):
        product = self.products[product_name]
        return product.required_roles

    def get_item_list(self):

        res = []
        for item, amount in self._items_in_stock.iteritems():
            for i in range(amount):
                res.append(item)
        return res

    def get_ingredients_of_product_iteratively(self, product_name, amount=1, consider_intermediate_ingredients=False):
        product = self.products[product_name]

        if len(product.consumed_items) == 0:
            return {product_name: amount}
        else:
            res = {}
            for ingredient in product.consumed_items:
                res = CalcUtil.dict_sum(res, self.get_ingredients_of_product_iteratively(ingredient.name,
                                                                                         ingredient.amount * amount,
                                                                                         consider_intermediate_ingredients=consider_intermediate_ingredients))

                if consider_intermediate_ingredients:
                    res = CalcUtil.dict_sum(res, {ingredient.name: ingredient.amount * amount})
            return res

    def get_ingredients_of_dict(self, dict, consider_intermediate_ingredients=False):
        base_ingredients = {}
        for item, count in dict.iteritems():
            base_ingredients = CalcUtil.dict_sum(base_ingredients, self.get_ingredients_of_product_iteratively(
                product_name=item, amount=count * 15,
                consider_intermediate_ingredients=consider_intermediate_ingredients))

        return base_ingredients

    def fits_in_store(self, name):
        return self.products[name].volume <= self.load_free

    def usages_for_assembly(self, item):
        return self.useful_items_for_assembly.count(item)

    def get_volume_of_item(self, item):
        return self.products[item].volume

    def get_my_stock_items(self):
        return self.agent_stock_items[ProductProvider.STOCK_ITEM_TYPE_STOCK][self._agent_name]

    def finished_product_load_factor(self):
        return self.calculate_total_volume_dict(self.get_finished_products_in_stock())/ self.load_max
