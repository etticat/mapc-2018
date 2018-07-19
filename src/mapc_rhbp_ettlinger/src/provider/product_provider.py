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
    """
    Provider, that provides information on products/items and product/item goals
    Goals describe how the stock is likely going to change if the current actions are continued
    """
    __metaclass__ = Singleton

    STORAGE_TOPIC = "/storage"
    STOCK_ITEM_TOPIC = "/stockitem/status"

    STOCK_ITEM_TYPE_STOCK = "stock"
    STOCK_ITEM_TYPE_STORAGE_STOCK = "stock"
    STOCK_ITEM_TYPE_GATHER_GOAL = "gather"
    STOCK_ITEM_TYPE_ASSEMBLY_GOAL = "assembly"
    STOCK_ITEM_TYPE_DELIVERY_GOAL = "delivery"
    STOCK_ITEM_TYPE_HOARDING_GOAL = "hoarding"
    STOCK_ITEM_TYPE_WELL_GOAL = "well"
    STOCK_ITEM_ALL_AGENT_TYPES = [STOCK_ITEM_TYPE_STOCK, STOCK_ITEM_TYPE_GATHER_GOAL, STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
                                  STOCK_ITEM_TYPE_DELIVERY_GOAL, STOCK_ITEM_TYPE_HOARDING_GOAL]

    def __init__(self, agent_name):

        # Init constructor variables
        self._agent_name = agent_name

        # Init runtime variables
        self.hoarding_destination = None
        self._product_infos = {}
        self._gatherable_items = {}
        self._assemblable_items = {}
        self._items_in_stock = {}
        self._agent_stocks = {
            ProductProvider.STOCK_ITEM_TYPE_STOCK: {},
            ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL: {}
        }
        self._storage_stocks = {
            ProductProvider.STOCK_ITEM_TYPE_STOCK: {},
            ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL: {},
            ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL: {}
        }
        self.storages = {}
        self.load_max = 0
        self.load_free = 0
        self.load = 0
        self.role = None
        self.useful_items_for_assembly = []

        # Init providers
        self._facility_provider = FacilityProvider()

        # Init subscribers and publishers
        self._pub_stock_item = rospy.Publisher(ProductProvider.STOCK_ITEM_TOPIC, StockItem, queue_size=10)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start", SimStart,
                         self._callback_sim_start)
        rospy.Subscriber(ProductProvider.STOCK_ITEM_TOPIC, StockItem, self._callback_stock_item)
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)
        rospy.Subscriber(ProductProvider.STORAGE_TOPIC, StorageMsg, self.storage_callback)

    def storage_callback(self, storageMsg):
        """
        Keeps track of all the storages for later use of storage stock
        :param storageMsg:
        :type storageMsg: StorageMsg
        :return:
        """

        for storage in storageMsg.facilities:
            self.storages[storage.name] = storage

    def _callback_stock_item(self, stock_item):
        """
        Keep track of all stock items and goals of all agents and storages
        :param stock_item:
        :param msg: StockItem
        :return:
        """

        item_dict = CalcUtil.dict_from_key_int_values(stock_item.amounts)

        if stock_item.entity[0] == "a":
            # Entities starting with a are agents
            self._agent_stocks[stock_item.type][stock_item.entity] = item_dict
        elif stock_item.entity[0] == "s":
            # Entities starting with s are storages
            if stock_item.type not in self._storage_stocks:
                self._storage_stocks[stock_item.type] = {}
            self._storage_stocks[stock_item.type][stock_item.entity] = item_dict

    def get_stored_items(self, storage_name=None, include_job_goals=True, include_stock=True,
                         include_hoarding_goal=True):
        """
        Returns the number of items in a storage and their goals
        :param storage_name: The name of the storage to get items of or None if all storages should be considered
        :type storage_name: str
        :param include_job_goals: True if job goals should be considered
        :type include_job_goals: bool
        :param include_stock: True if stock items should be considered
        :type include_stock: bool
        :param include_hoarding_goal: True if hoarding goals should be considered
        :type include_hoarding_goal:
        :return:
        """
        items = {}

        for storage in self.storages.values():
            if storage_name is None or storage.name == storage_name:
                if include_stock:
                    for item in storage.items:
                        items[item.name] = items.get(item.name, 0) + item.stored
                if include_job_goals:
                    for job_name, job_dict in self._storage_stocks.iteritems():
                        # Types starting wiht j are job goals
                        if job_name[0] == "j":
                            if storage in job_dict:
                                items = CalcUtil.dict_sum(items, job_dict[storage])
                if include_hoarding_goal:
                    for agent_name, hoarding_dict in self._storage_stocks.iteritems():
                        # Types starting wiht a are hoarding goals of agents
                        if agent_name[0] == "a":
                            if storage in hoarding_dict:
                                items = CalcUtil.dict_sum(items, hoarding_dict[storage])
        return items

    def _callback_agent(self, agent):
        """
        Retrieves the current state of stock items for use in other methods
        Also keeps track of agent information on load
        :param agent:
        :type agent: Agent
        :return:
        """
        items_in_stock_new = {}

        for item in self._product_infos.keys():
            items_in_stock_new[item] = 0

        for item in agent.items:
            items_in_stock_new[item.name] = items_in_stock_new.get(item.name, 0) + item.amount

        if items_in_stock_new != self._items_in_stock:
            # If items changed since last step, notify all other agents
            self._items_in_stock = items_in_stock_new
            self._pub_stock_item.publish(StockItem(
                entity=self._agent_name,
                type=ProductProvider.STOCK_ITEM_TYPE_STOCK,
                amounts=CalcUtil.key_int_values_from_dict(items_in_stock_new)
            ))

        self.load_max = agent.load_max
        self.load = agent.load
        self.load_free = agent.load_max - agent.load

    def _callback_sim_start(self, msg):
        """
        Saves the products of the current simulation and their details
        :param msg:
        :type msg: SimStart
        :return:
        """
        self.role = msg.role.name
        self.useful_items_for_assembly = []

        for product in msg.products:
            self._product_infos[product.name] = product
            if len(product.consumed_items) > 0:
                self._assemblable_items[product.name] = product

                if self.role in product.required_roles:
                    for ingredient in product.consumed_items:
                        self.useful_items_for_assembly.append(ingredient.name)
            else:
                self._gatherable_items[product.name] = product

    def get_product_by_name(self, product):
        """
        Returns the product by name
        :param product:
        :return: Product
        """
        return self._product_infos[product]

    def get_gatherable_ingredients(self):
        """
        Returns all items that can be gathered at a resource node
        :return:
        """
        return self._gatherable_items

    def get_assemblable_products(self):
        """
        Returns all items, that can be assembled from other items
        :return:
        """
        return self._assemblable_items

    def update_gathering_goal(self, item_to_focus):
        """
        Updates the expected stock of items after gathering
        :param item_to_focus:
        :return:
        """
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
        """
        Resets the expected stock of items after gathering
        :return:
        """
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_GATHER_GOAL,
            amounts={}
        ))

    def update_assembly_goal(self, task_string, step=0):
        """
        Updates the expected stock of items after assembly
        :param task_string:
        :param step:
        :return:
        """
        items_to_assemble = {}
        tasks = task_string.split(",")
        for i in range(step, len(tasks)):
            # Extract all items that the agent is assembling itself and add it as a goal
            task = tasks[i].split(":")
            if task[0] == "assemble":
                product_name = task[1]
                # Add the items that we will have after assembly
                # Not all of them will be deducted from the current agent, but thats ok, as we need this information only
                # aggregated over all agents
                items_to_assemble[product_name] = items_to_assemble.get(product_name, 0) + 1

                # Subtract the items that we will use for assembly.
                items_to_assemble = CalcUtil.dict_sum(items_to_assemble,
                                                      self.get_ingredients_of_product(product_name=product_name,
                                                                                      amount=-1))

        # Publish expected item stock to all agents
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(items_to_assemble)
        ))

    def stop_assembly(self):
        """
        Reset all expected items after assembly
        :return:
        """
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_ASSEMBLY_GOAL,
            amounts={}
        ))

    def update_hoarding_goal(self, finished_products_to_store, destination):
        """
        Update stock of expected items after hoarding in agent as well as storage
        :param finished_products_to_store:
        :param destination:
        :return:
        """
        if self.hoarding_destination is not None and destination is not self.hoarding_destination:
            # If wes till have an old hoarding destination saved, first tell everyone, that this destination does
            # not expect any new items
            self.stop_hoarding(destination=self.hoarding_destination)

        # Notify all agents expected items of this agent
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(CalcUtil.negate_dict(finished_products_to_store))
        ))
        # Notify all agents expected items of the storage destination
        self._pub_stock_item.publish(StockItem(
            entity=destination,
            type=self._agent_name,
            amounts=CalcUtil.key_int_values_from_dict(finished_products_to_store)
        ))

    def stop_hoarding(self, destination):
        """
        Reset hoarding goal for agent and storage
        """
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL,
            amounts={}
        ))
        self._pub_stock_item.publish(StockItem(
            entity=destination,
            type=self._agent_name,
            amounts={}
        ))

    def update_delivery_goal(self, item_list, job_id, storage):
        """
        Update the expected items after delivery for a delivery task
        :param item_list:
        :param job_id:
        :param storage:
        :return:
        """

        # create dictionary of items
        item_dict = CalcUtil.dict_from_string_list(item_list)
        # check how many have to be taken from the storage
        storage_items = CalcUtil.dict_diff(item_dict, self._items_in_stock, normalize_to_zero=False)
        # negate both goals
        item_dict = CalcUtil.negate_dict(item_dict)
        storage_items = CalcUtil.negate_dict(storage_items)

        # Publish goals
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL,
            amounts=CalcUtil.key_int_values_from_dict(item_dict)
        ))
        self._pub_stock_item.publish(StockItem(
            entity=storage,
            type=job_id + self._agent_name,
            amounts=CalcUtil.key_int_values_from_dict(storage_items)
        ))

    def stop_delivery(self, job_id, storage):
        """
        Reset expected delivery item goal for agent and storage
        :param job_id:
        :param storage:
        :return:
        """
        self._pub_stock_item.publish(StockItem(
            entity=self._agent_name,
            type=ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL,
            amounts={}
        ))
        self._pub_stock_item.publish(StockItem(
            entity=storage,
            type=job_id + self._agent_name,
            amounts={}
        ))

    def get_base_ingredients_in_stock(self):
        """
        Returns the amount of all base ingredients in stock
        :return: dict
        """
        res = {}
        for item in self._gatherable_items.keys():
            res[item] = self._items_in_stock.get(item, 0)

        return res

    def get_finished_products_in_stock(self):
        """
        Returns the amount of all finished products in stock
        :return: dict
        """
        res = {}
        for item in self._assemblable_items.keys():
            res[item] = self._items_in_stock.get(item, 0)

        return res

    def calculate_total_volume_dict(self, product_dict):
        """
        Calculates the total volume that a dict of products will use
        :param product_dict:
        :type product_dict: dict
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

    def get_agent_stock_items(self, types):
        """
        Returns a list of all items that any agent owns or expected to has in the near future
        :param types: Types that should be included in the calculation. e.g. stock itself, delivery goal, gather goal, ...
        :type types: list
        :return:
        """
        res = {}
        for type in types:
            for stock_item_dicts in self._agent_stocks[type].values():
                res = CalcUtil.dict_sum(res, stock_item_dicts)
        return res

    def get_ingredients_of_product(self, product_name, amount=1):
        """
        Returns the number of ingredients neccessary to make a specified number of items
        :param product_name:
        :param amount:
        :return:
        """
        product = self._product_infos[product_name]
        res = {}
        for ingredient in product.consumed_items:
            res[ingredient.name] = ingredient.amount * amount

        return res

    def get_roles_of_product(self, product_name):
        """
        Returns the roles neccessary to assemble a product
        :param product_name:
        :return:
        """
        product = self._product_infos[product_name]
        return product.required_roles

    def get_item_list(self):
        """
        Returns a list of all available items of the agent
        :return:
        """
        res = []
        for item, amount in self._items_in_stock.iteritems():
            for i in range(amount):
                res.append(item)
        return res

    def get_gatherable_ingredients_of_product_iteratively(self, product_name, amount=1, consider_intermediate_ingredients=False):
        """
        Returns the gatherable ingredients that are neccessary to create a certain product.
        If the item is assembled from other finished product, the ingredients of those products are returned
        :param product_name:
        :param amount:
        :param consider_intermediate_ingredients: True if the intermediary ingredients should also be returned
            e.g. item10 is made from item6. item6 is made from item3. This flat determines if item6 should be included
            in the list.
        :return: dict
        """
        product = self._product_infos[product_name]

        if len(product.consumed_items) == 0:
            return {product_name: amount}
        else:
            res = {}
            for ingredient in product.consumed_items:
                res = CalcUtil.dict_sum(res, self.get_gatherable_ingredients_of_product_iteratively(ingredient.name,
                                                                                                    ingredient.amount * amount,
                                                                                                    consider_intermediate_ingredients=consider_intermediate_ingredients))

                if consider_intermediate_ingredients:
                    res = CalcUtil.dict_sum(res, {ingredient.name: ingredient.amount * amount})
            return res

    def get_gatherable_ingredients_of_dict(self, dict, consider_intermediate_ingredients=False):
        """
        Returns the ingredients that are neccessary to make a number of items
        :param dict:
        :param consider_intermediate_ingredients: True if the intermediary ingredients should also be returned
            e.g. item10 is made from item6. item6 is made from item3. This flat determines if item6 should be included
        :return: dict
        """
        base_ingredients = {}
        for item, count in dict.iteritems():
            base_ingredients = CalcUtil.dict_sum(base_ingredients, self.get_gatherable_ingredients_of_product_iteratively(
                product_name=item, amount=count,
                consider_intermediate_ingredients=consider_intermediate_ingredients))

        return base_ingredients

    def fits_in_store(self, item_name):
        """
        Returns True if one more of specified item fits into stock
        :param item_name:
        :return:
        """
        return self._product_infos[item_name].volume <= self.load_free

    def usages_for_assembly(self, item):
        """
        Returns the number items that can be assembled form the item
        :param item:
        :return:
        """
        return self.useful_items_for_assembly.count(item)

    def get_volume_of_item(self, item):
        """
        Returns the volume of an item
        :param item:
        :return:
        """
        return self._product_infos[item].volume

    def get_stock_items(self):
        """
        returns the stock items of the agent itself
        :return:
        """
        return self._agent_stocks[ProductProvider.STOCK_ITEM_TYPE_STOCK][self._agent_name]

    def finished_product_load_factor(self):
        """
        Returns the factor describung the percentage of the storage that is used for finished products
        :return:
        """
        return self.calculate_total_volume_dict(self.get_finished_products_in_stock()) / self.load_max

    @property
    def assemblable_items(self):
        return self._assemblable_items

    @property
    def product_infos(self):
        return self._product_infos
