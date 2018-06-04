#!/usr/bin/env python2
import copy

import rospy
from __builtin__ import xrange
from mac_ros_bridge.msg import SimStart, Agent, Product
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import TaskKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil


class ProductProvider(object):


    def __init__(self, agent_name):
        # TODO: Make this a singleton. (And all other providers and knowledgebases. this should keep the load a little bit lower

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

        rospy.Subscriber(
            AgentUtils.get_internal_prefix(agent_name=agent_name) + "stock_goal",
            StockItemMsg,
            self._callback_stock_goal)
        self._pub_stock_msg = rospy.Publisher(
            AgentUtils.get_internal_prefix(agent_name=agent_name) + "stock_goal",
            StockItemMsg
            , queue_size=10)

        self._task_knowledgebase = TaskKnowledgebase()
        self._stock_item_knowledgebase = StockItemKnowledgebase()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)

    def _callback_stock_goal(self, stockMsg):
        """

        :param stockMsg:
        :type stockMsg: StockItemMsg
        :return:
        """
        for stock_item in stockMsg.stock_items:
            if stock_item.item in self.base_ingredients.keys():
                self._gather_goal[stock_item.item] = stock_item.goal
            elif stock_item.item in self.finished_products.keys():
                self._assemble_goal[stock_item.item] = stock_item.goal


    def _callback_agent(self, msg):
        """

        :param msg:
        :type msg: Agent
        :return:
        """
        items_in_stock_new  = {}

        for item in self.products.keys():
            items_in_stock_new[item] = 0

        for item in msg.items:
            items_in_stock_new[item.name] = items_in_stock_new.get(item.name, 0) + item.amount


        if items_in_stock_new != self._items_in_stock:
            # for index in items_in_stock_new.keys():
            #     rospy.logerr("%s: %s", str(index), str(items_in_stock_new[index]))
            self._items_in_stock = items_in_stock_new
            self.save_stock_and_goals()



        self.load_max = msg.load_max
        self.load = msg.load
        self.load_free = msg.load_max - msg.load


        # Show all ingredient volumes of all products
        # for i in self.products.keys():
        #     rospy.logerr("ProductProvider:: %s volume: %s total volume: %s", i, self.calculate_total_volume({i:1}), self.calculate_total_volume(self.get_ingredients_of_product(i)))
        rospy.loginfo("ProductProvider:: Required ingredients: %s", str(self.calculate_desired_ingredient_stock()))

    def _callback_sim_start(self, msg):
        """

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


    def get_required_finished_products_for_tasks(self):
        all_required_finished_products = self._task_knowledgebase.get_tasks(
            agent_name=self._agent_name,
            status="assigned")

        items_in_stock = copy.copy(self._items_in_stock)

        still_required_finished_products = []

        for product in all_required_finished_products:
            if (items_in_stock.get(product.item, 0) > 0): # we already have the item
                items_in_stock[product.item] -= 1
            else: # we still need the item
                still_required_finished_products.append(product.item)

        return still_required_finished_products

    def get_required_ingredients_for_tasks(self):
        all_tasks = self._task_knowledgebase.get_tasks(
            agent_name=self._agent_name,
            status="assigned")

        items_in_stock = copy.copy(self._items_in_stock)

        still_needed_ingredients = []

        while len(all_tasks ) > 0:
            task = all_tasks.pop()

            if (items_in_stock.get(task.item, 0) > 0):  # we already have the item
                items_in_stock[task.item] -= 1
            else:
                ingredients = list(self.products[task.item].consumed_items)
                while len(ingredients) > 0:
                    ingredient = ingredients.pop()

                    if len(self.products[ingredient.name].consumed_items) > 0: # If ingredient requires assembly
                        ingredients.extend(self.products[ingredient.name].consumed_items)
                    else:
                        for ingredient_index in xrange(ingredient.amount):
                            if items_in_stock.get(ingredient.name, 0) > 0:
                                items_in_stock[ingredient.name] -= 1
                            else:
                                still_needed_ingredients.append(ingredient.name)

        return still_needed_ingredients

    def get_product_by_name(self, product):
        """

        :param product:
        :return: Product
        """
        return self.products[product]

    def get_base_ingredients(self):
        return self.base_ingredients

    def save_stock_and_goals(self, publish_goals=False):
        msg = StockItemMsg()
        for index in self._items_in_stock.keys():
            item = StockItem(agent=self._agent_name, item=index, amount=self._items_in_stock[index],
                             goal=self._gather_goal.get(index, 0) + self._assemble_goal.get(index, 0))
            self._stock_item_knowledgebase.update_stock(item)
            msg.stock_items.append(item)

        if publish_goals:
            self._pub_stock_msg.publish(msg)

    def start_gathering(self, item_to_focus):

        load_free = self.load_max - self.load

        items_to_carry = load_free / self.get_product_by_name(item_to_focus).volume

        self._gather_goal = {
            item_to_focus: items_to_carry + self._items_in_stock.get(item_to_focus, 0)
        }

        self.save_stock_and_goals(publish_goals=True)
    def start_assembly(self, items_to_assemble):
        self._assemble_goal = items_to_assemble
        self.save_stock_and_goals(publish_goals=True)

    def stop_gathering(self):
        self._gather_goal = {}
        self.save_stock_and_goals(publish_goals=True)

    def stop_assembly(self):
        self._assemble_goal = {}
        self.save_stock_and_goals(publish_goals=True)

    def get_required_ingredients_for_hoarding(self):
        """
        Returns all ingredients that an agent needs to gather to meet its goal
        :return: list
        """
        res = []

        for index in self.base_ingredients:
            needed_amount = self._gather_goal.get(index, 0) - self._items_in_stock.get(index, 0)

            if needed_amount > 0:
               res.append(index)

        rospy.loginfo("All ingredients to gather: %s", str(res) )
        return res

    def get_required_finished_products_for_hoarding(self):
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

    def calculate_desired_finished_product_stock(self):
        # TODO: Take into account how much each each product is worth (how much it usually pays)
        # TODO: Take into account how much effort it is to make (moving, gathering, assembling)...

        res = {}

        for product in self.finished_products.keys():
            res[product] = 1

        return res

    def total_items_in_stock(self):
        return self._stock_item_knowledgebase.get_total_stock()

    def get_ingredients_of_product(self, product_name, amount = 1):
        product = self.products[product_name]

        if len(product.consumed_items) == 0:
            return {product_name:amount}
        else:
            res = {}
            for ingredient in product.consumed_items:
                res = CalcUtil.dict_sum(res, self.get_ingredients_of_product(ingredient.name, ingredient.amount * amount))
            return res

    def calculate_desired_ingredient_stock(self):
        desired_finished_product_stock = self.calculate_desired_finished_product_stock()
        all_items = self.total_items_in_stock()

        ingredients = {}

        keys = desired_finished_product_stock.keys()
        for finished_product in keys:
            items_taken_from_inventory = min(desired_finished_product_stock[finished_product], all_items[finished_product])
            all_items[finished_product] -= items_taken_from_inventory
            desired_finished_product_stock[finished_product] -= items_taken_from_inventory


            ingredients = CalcUtil.dict_sum(self.get_ingredients_of_product(finished_product, desired_finished_product_stock[finished_product]), ingredients)

        # TODO: Take into account products that are assembled from already assembled products

        keys = ingredients.keys()
        for ingredient in keys:
            items_taken_from_inventory = min(ingredients[ingredient], all_items[ingredient])
            ingredients[ingredient] -= items_taken_from_inventory

        return ingredients
