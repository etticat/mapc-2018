#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg

from agent_knowledge.item import StockItemBaseKnowledge
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.provider.product')


class ProductProvider(object):
    __metaclass__ = Singleton

    def __init__(self, agent_name):

        self.products = {}
        self.base_ingredients = {}
        self.finished_products = {}
        self._items_in_stock = {}
        self._gather_goal = {}
        self._assemble_goal = {}

        self._agent_name = agent_name

        self.load_max = 0
        self.load_free = 0
        self.load = 0

        rospy.Subscriber(
            AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start",
            SimStart,
            self._callback_sim_start)

        self._stock_item_knowledge_base = StockItemBaseKnowledge()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)

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
            self.save_stock_and_sock_goals()

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
        for product in msg.products:
            self.products[product.name] = product
            if len(product.consumed_items) > 0:
                self.finished_products[product.name] = product
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

    def save_stock_and_sock_goals(self):
        msg = StockItemMsg()
        for index in self._items_in_stock.keys():
            item = StockItem(agent=self._agent_name, item=index, amount=self._items_in_stock[index],
                             goal=self._gather_goal.get(index, 0) + self._assemble_goal.get(index, 0))
            self._stock_item_knowledge_base.update_stock(item)
            msg.stock_items.append(item)

    def start_gathering(self, item_to_focus):

        load_free = self.load_max - self.load

        items_to_carry = load_free / self.get_product_by_name(item_to_focus).volume

        self._gather_goal = {
            item_to_focus: items_to_carry + self._items_in_stock.get(item_to_focus, 0)
        }
        self.save_stock_and_sock_goals()

    def start_assembly(self, items_to_assemble):
        goal = {}

        for product in self.finished_products.keys():
            goal[product] = items_to_assemble.get(product, 0) + self._items_in_stock.get(product, 0)
        self._assemble_goal = goal
        self.save_stock_and_sock_goals()

    def stop_gathering(self):
        self._gather_goal = {}
        self.save_stock_and_sock_goals()

    def stop_assembly(self):
        self._assemble_goal = {}
        self.save_stock_and_sock_goals()

    def get_base_ingredients_in_stock(self):
        res = {}
        for item in self.base_ingredients.keys():
            res[item] = self._items_in_stock[item]

        return res

    def get_finished_products_in_stock(self):
        res = {}
        for item in self.base_ingredients.keys():
            res[item] = self._items_in_stock[item]

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

    def calculate_total_volume(self, product_dict):
        """
        Calculates the total volume that a dict of products will use
        :param product_dict:
        :return:
        """

        v = 0

        for product in product_dict.keys():
            product_by_name = self.get_product_by_name(product)
            v += product_by_name.volume * product_dict[product]

        return v

    def total_items_in_stock(self):
        return self._stock_item_knowledge_base.get_total_stock()

    def get_ingredients_of_product(self, product_name, amount=1):
        product = self.products[product_name]
        res = {}
        for ingredient in product.consumed_items:
            res[ingredient.name] = ingredient.amount * amount

        return res

    def get_roles_of_product(self, product_name):
        product = self.products[product_name]
        return product.required_roles

    def get_items(self):
        res = []
        for item, amount in self._items_in_stock.iteritems():
            if amount > 0:
                res.append(Item(
                    name=item,
                    amount=amount
                ))
        return res

    def get_base_ingredients_of_product_iteratively(self, product_name, amount=1):
        product = self.products[product_name]

        if len(product.consumed_items) == 0:
            return {product_name: amount}
        else:
            res = {}
            for ingredient in product.consumed_items:
                res = CalcUtil.dict_sum(res, self.get_base_ingredients_of_product_iteratively(ingredient.name,
                                                                                              ingredient.amount * amount))
            return res
