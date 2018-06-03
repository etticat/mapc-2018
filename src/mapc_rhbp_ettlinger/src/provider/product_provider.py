#!/usr/bin/env python2
import copy

import rospy
from __builtin__ import xrange
from mac_ros_bridge.msg import SimStart, Agent

from agent_knowledge.tasks import TaskKnowledgebase
from common_utils.agent_utils import AgentUtils


class ProductProvider(object):


    def __init__(self, agent_name):
        # TODO: Make this a singleton. (And all other providers and knowledgebases. this should keep the load a little bit lower

        self.products = {}
        self.base_ingredients = {}
        self.finished_products = {}
        self._items_in_stock = {}
        self._assembled_items = {}

        rospy.Subscriber(
            AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start",
            SimStart,
            self._callback_sim_start)

        self.task_knowledge = TaskKnowledgebase()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)



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
            for index in items_in_stock_new.keys():
                rospy.logerr("%s: %s", str(index), str(items_in_stock_new[index]))
            # TODO: Save to db
            self._items_in_stock = items_in_stock_new
        else:
            rospy.logerr("nothing new")


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

    def get_required_finished_products(self, agent_name):
        all_required_finished_products = self.task_knowledge.get_tasks(
            agent_name=agent_name,
            status="assigned")

        items_in_stock = copy.copy(self._items_in_stock)

        still_required_finished_products = []

        for product in all_required_finished_products:
            if (items_in_stock.get(product.item, 0) > 0): # we already have the item
                items_in_stock[product.item] -= 1
            else: # we still need the item
                still_required_finished_products.append(product.item)

        return still_required_finished_products

    def get_required_ingredients(self, agent_name):
        all_tasks = self.task_knowledge.get_tasks(
            agent_name=agent_name,
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
        return self.products[product]

    def get_base_ingredients(self):
        return self.base_ingredients
